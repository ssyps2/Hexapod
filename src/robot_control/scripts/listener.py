#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import time
import logging
import kinematics
import Board
import threading
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from vel_publisher import CONTROL_FREQ as PUB_RATE

hexapod = kinematics.hexKine(ctrl_freq = PUB_RATE)  # same with freq of publisher

# utility lambda for servo pulse and angle conversion
pulse2angle_lambda = lambda pulse: (pulse - hexapod.SERVO_HALF_PULSE) * hexapod.PULSE2ANGLE # in degree
angle2pulse_lambda = lambda angle: np.rad2deg(angle)/hexapod.PULSE2ANGLE+hexapod.SERVO_HALF_PULSE # in degree

# thread class for servo reading and writing
class servoThread (threading.Thread):
    def __init__(self, name, servoID, legID, jointID):
        threading.Thread.__init__(self)
        self.name = name
        self.servoID = servoID
        self.legID = legID
        self.jointID = jointID

    def run(self):
        while True:
            # update joint angle
            try:
                # joint1 and 2 of leg3~5 was inversed because of assembling
                if self.servoID == 'Leg3_Joint1' or self.name == 'Leg3_Joint2' or\
                self.name == 'Leg4_Joint1' or self.name == 'Leg4_Joint2' or\
                self.name == 'Leg5_Joint1' or self.name == 'Leg5_Joint2':
                    self.angle = -pulse2angle_lambda(Board.getBusServoPulse(self.servoID))
                else:
                    self.angle = pulse2angle_lambda(Board.getBusServoPulse(self.servoID))
                
                # angle in degree
                if self.angle > 120 or self.angle < -120:
                    raise Exception(f"{self.name}: received angle out of range")
                
                # send back to hexKine class instance
                hexapod.legs_joint_angle[self.legID][self.jointID] = self.angle

            except Exception:
                pass

            # send control command to servo
            try:
                self.next_joint_angle = hexapod.next_feet_joint_angle[self.legID][self.jointID]

                # joint1 and 2 of leg3~5 was inversed because of assembling
                if self.servoID == 'Leg3_Joint1' or self.name == 'Leg3_Joint2' or\
                self.name == 'Leg4_Joint1' or self.name == 'Leg4_Joint2' or\
                self.name == 'Leg5_Joint1' or self.name == 'Leg5_Joint2':
                    self.pulse = int(angle2pulse_lambda(-self.next_joint_angle))
                else:
                    self.pulse = int(angle2pulse_lambda(self.next_joint_angle))
                
                if self.pulse > 1000 or self.pulse < 0:
                    raise Exception(f"{self.name}: command pulse out of range")
                else:
                    Board.setBusServoPulse(self.servoID, self.pulse, 1)
            
            except Exception:
                pass

# create list of thread instances
servos_thread = []

for i in range(0,6):
    one_leg_servos = []

    for j in range(0,3):
        one_leg_servos.append(servoThread(name=f"Leg{i}_Joint{j}", servoID=kinematics.legs_ID[i][j], legID=i, jointID=j))
    
    servos_thread.append(one_leg_servos)
    servos_thread[i][j].start()  # start threads


## ROS part
def cmdCallback(twistMsg):
    rospy.loginfo("x:%.6f, y:%.6f, z:%.6f", twistMsg.x, twistMsg.y, twistMsg.z)

    hexapod.next_feet_joint_angle = hexapod.implementCmdGait(twistMsg.x, twistMsg.y, twistMsg.z,
        kinematics.hex_mode_e().TRIPOD)

def cmd_subscriber():
    rospy.init_node('cmd_subscriber', anonymous=True)

    rospy.Subscriber("/twist_cmd", Twist, cmdCallback)    # name reserved:/twist_cmd

    rospy.spin()

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    hexapod.initHexapod()
    time.sleep(2)
    print("init completed")

    cmd_subscriber()
