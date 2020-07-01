#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('my_first_python_node')
    rospy.loginfo("This node has been started")
    # rospy.sleep(1)
    # rospy.loginfo("Exit now")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep() # will sleep the amount of time so that the rate stays 10Hz

    