#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import logging
import kinematics
import rospy
from geometry_msgs.msg import Twist

hexapod = kinematics.hex_kine()

def cmdCallback(twistMsg):
    rospy.loginfo("x:%.6f, y:%.6f, z:%.6f", twistMsg.x, twistMsg.y, twistMsg.z)
    hexapod.cmdHexapodMove(twistMsg.x, twistMsg.y, twistMsg.z, kinematics.hex_mode_e().TRIPOD)

def cmd_subscriber():
    rospy.init_node('cmd_subscriber', anonymous=True)

    rospy.Subscriber("/twist_cmd", Twist, cmdCallback)    # name reserved:/twist_cmd

    rospy.spin()

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    cmd_subscriber()
