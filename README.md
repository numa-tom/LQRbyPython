  #  概要
  RaspberryPiで2車輪ロボットの倒立を行うサンプルプログラムである。  
  タイヤの部分にエンコーダ、車体の角度計測にジャイロセンサを用いることを想定している。
  ジャイロセンサにフィルタはかけておらず単純積分で行っている。  
  # 制御工学
  制御工学における状態フィードバックの手法の一つである最適レギュレータ(別名：LQR)で制御を行う。
