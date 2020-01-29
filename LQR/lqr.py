import sys
import traceback
import numpy as np
import time
import GYRO
import smbus
import struct
import signal
import scipy.signal as ss
import pigpio
import math

#ピン番号設定
gpio_pin0 = 18#pwm A
gpio_pin1 = 19#pwm B
gpio_pin2 = 27#input1 A
gpio_pin3 = 22#input2 A
gpio_pin4 = 20#input1 B
gpio_pin5 = 21#input2 B
gpio_pin6 = 16#standby

pi = pigpio.pi()
pi.set_mode(gpio_pin0, pigpio.OUTPUT)
pi.set_mode(gpio_pin1, pigpio.OUTPUT)
pi.set_mode(gpio_pin2, pigpio.OUTPUT)
pi.set_mode(gpio_pin3, pigpio.OUTPUT)
pi.set_mode(gpio_pin4, pigpio.OUTPUT)
pi.set_mode(gpio_pin5, pigpio.OUTPUT)
pi.set_mode(gpio_pin6, pigpio.OUTPUT)
pi.write(gpio_pin6, 1) #stby
#アドレス設定
i2c = smbus.SMBus(1)
addr = 0x54

VALUE_HI = 0x00
VALUE_LO = 0x01
RESET = 0x02

#フィードバックゲイン設定
K1=900.0 #theta Gain=100
K2=2 #phi Gain
K3=700.0 #theta dot Gain=500
K4=21 #phi_dot Gain
#初期設定
angle_x=0.0
dt=0.007
u=0.0

#不感帯対策のための最大最小デューティ比設定
k=10000
MAX_DUTY=80*k
MIN_DUTY=10*k

def Get_encoder_value():
    #time.sleep(0.0002)
    temp = i2c.read_word_data(addr, VALUE_HI)
    return (struct.unpack(">H", struct.pack("<H", temp))[0])

def Encoder_Reset():
    i2c.write_byte_data(addr, RESET, True)
    

#最適レギュレータでの制御量算出    
def LQR(P_angle, W_angle, P_rate, W_rate):#
    theta = K1*(P_angle+1)
    phi = K2*(W_angle - 0)
    theta_dot = K3*(P_rate - 0)
    phi_dot = K4*(W_rate - 0)
    u=theta + theta_dot + phi + phi_dot
    return u

#デューティ比決定
def Controller(u):
    if u>MAX_DUTY:
        u=MAX_DUTY
    elif u<(-1)*MAX_DUTY:
        u=(-1)*MAX_DUTY
    elif (-1)*MIN_DUTY<u and u<0:
        u=MIN_DUTY
        pi.write(gpio_pin2, 1) #A_IN1
        pi.write(gpio_pin3, 1) #A_IN2
        pi.write(gpio_pin4, 1) #B_IN1
        pi.write(gpio_pin5, 1) #B_IN2
    elif 0<u and u<MIN_DUTY:
        u=(-1)*MIN_DUTY
        pi.write(gpio_pin2, 1) #A_IN1
        pi.write(gpio_pin3, 1) #A_IN2
        pi.write(gpio_pin4, 1) #B_IN1
        pi.write(gpio_pin5, 1) #B_IN2
    time.sleep(0.00001)
    return u

#
def Motor(d):
    if d>0: 
        pi.write(gpio_pin2, 0) #A_IN1
        pi.write(gpio_pin3, 1) #A_IN2
        pi.write(gpio_pin4, 1) #B_IN1
        pi.write(gpio_pin5, 0) #B_IN2
        pi.hardware_PWM(gpio_pin0, 10000, int(d))
        pi.hardware_PWM(gpio_pin1, 10000, int(d))
        time.sleep(0.0001)
            
    elif d<0:
        pi.write(gpio_pin2, 1) #A_IN1
        pi.write(gpio_pin3, 0) #A_IN2
        pi.write(gpio_pin4, 0) #B_IN1
        pi.write(gpio_pin5, 1) #B_IN2
        pi.hardware_PWM(gpio_pin0, 10000, abs(int(d)))
        pi.hardware_PWM(gpio_pin1, 10000, abs(int(d)))
        time.sleep(0.0001)
        
    return


if __name__=='__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    gyro = GYRO.GYRO()
    gyro.sensor_calib()
    time.sleep(1)

    prev = time.time()
    W_angle=0.0
    W_smpl=0.0
    W_rate=0.0
    Encoder_Reset()#Ready to Start
    print("Start!\n")
    time.sleep(0.1)
    pre_W_angle=float('{0:.3f}'.format(struct.unpack(">h", struct.pack(">H", Get_encoder_value()))[0]))*360/600.0
    while time.time()-prev<10:
        LoopStartTime=time.time()
        #センサー値取得
        gx=float(gyro.get_xgyro_value())

        angle_x+=gx*dt

        W_angle=float('{0:.3f}'.format(struct.unpack(">h", struct.pack(">H", Get_encoder_value()))[0]))*360/600.0
        W_rate=(W_angle - float(pre_W_angle))/dt
        u+=LQR(angle_x, float(W_angle),  float(gx), W_rate)
        pre_W_angle=W_angle
        u=Controller(u)
        Motor(u)
        LoopEndTime=time.time()
        #サンプリングタイム固定
        while LoopEndTime - LoopStartTime <= dt :
            LoopEndTime=time.time()
    pi.write(gpio_pin6, 0)
    pi.stop()

            

