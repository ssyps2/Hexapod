#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import time
import logging
import kinematics
import rospy
from geometry_msgs.msg import Twist
from vel_publisher import CONTROL_FREQ as PUB_RATE

hexapod = kinematics.hex_kine(ctrl_freq = PUB_RATE)  # same with freq of publisher

def cmdCallback(twistMsg):
    rospy.loginfo("x:%.6f, y:%.6f, z:%.6f", twistMsg.x, twistMsg.y, twistMsg.z)
    hexapod.cmdHexapodMove(twistMsg.x, twistMsg.y, twistMsg.z, kinematics.hex_mode_e().TRIPOD)

def cmd_subscriber():
    rospy.init_node('cmd_subscriber', anonymous=True)

    rospy.Subscriber("/twist_cmd", Twist, cmdCallback)    # name reserved:/twist_cmd

    rospy.spin()

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    hexapod.initHexapod()
    time.sleep(2)

    cmd_subscriber()
