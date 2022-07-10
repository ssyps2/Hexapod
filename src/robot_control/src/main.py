#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import time
import numpy as np
import logging
import kinematics
import rospy
from geometry_msgs.msg import Twist

ik = kinematics.IK()

def cmdCallback(twistMsg):
    rospy.loginfo("x:%.6f, y:%.6f, z:%.6f", twistMsg.x, twistMsg.y, twistMsg.z)

def cmd_subscriber():
    rospy.init_node('cmd_subscriber', anonymous=True)

    rospy.Subscriber("/twist_cmd", Twist, cmdCallback)    # name reserved:/twist_cmd

    rospy.spin()

def startHexapod():
    while True:
        time.sleep(0.03)

        while True:
            try:
                pass
            except BaseException as e:
                print(e)
                break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cmd_subscriber()
    startHexapod()
