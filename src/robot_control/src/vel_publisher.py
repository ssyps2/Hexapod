#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    rospy.init_node('velocity_publisher', anonymous=True)

	# topic name: /hex/cmd_vel; message type: geometry_msgs::Twist
    turtle_vel_pub = rospy.Publisher('/hex/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)   # 10Hz

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.1  # max: 0.15
        vel_msg.angular.z = 0.05

        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("x:%0.2fm/s, y:%0.2fm/s, z:%0.2frad/s]", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z)

        rate.sleep() # delay in a rate of given Hz

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass


