import time
import Board

while True:
    # 参数：参数1：舵机id; 参数2：位置; 参数3：运行时间
    Board.setBusServoPulse(9, 200, 1000)  
    Board.setBusServoPulse(7, 500, 1000)
    time.sleep(1) 
    
    Board.setBusServoPulse(9, 500, 500)  # 9号舵机转到500位置，用时500ms
    time.sleep(0.5)  # 延时时间和运行时间相同
    
    Board.setBusServoPulse(9, 800, 500)  #舵机的转动范围0-240度，对应的脉宽为0-1000,即参数2的范围为0-1000
    time.sleep(0.5)
    
    Board.setBusServoPulse(7, 300, 400)
    time.sleep(0.4)

    Board.setBusServoPulse(7, 700, 800)
    time.sleep(0.8) 

    Board.setBusServoPulse(7, 500, 400)
    time.sleep(0.4)  

   
