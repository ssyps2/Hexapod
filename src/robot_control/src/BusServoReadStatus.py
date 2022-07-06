import time
import Board

leg1 = (7,8,9)
leg2 = (4,5,6)
leg3 = (1,2,3)
leg4 = (10,11,12)
leg5 = (13,14,15)
leg6 = (16,17,18)

def getBusServoStatus(leg_joint):
    Pulse1 = Board.getBusServoPulse(leg_joint[0])
    Pulse2 = Board.getBusServoPulse(leg_joint[1])
    Pulse3 = Board.getBusServoPulse(leg_joint[2])
    print('Pulse1: {}\nPulse2:  {}\nPulse:   {}\n'.format(Pulse1, Pulse2, Pulse3))
    time.sleep(0.5)

while True:   
    getBusServoStatus(leg4)
