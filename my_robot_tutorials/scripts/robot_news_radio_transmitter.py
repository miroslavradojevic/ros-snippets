#!/usr/bin/env python

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('robot_news_radio_transmitter') # , anonymous = True

    pub = rospy.Publisher("/robot_news_radio", String, queue_size=10) # topic name, message type

    rate = rospy.Rate(2) # 2 messages per second

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "This is robot news radio message"
        pub.publish(msg)
        rate.sleep()

rospy.loginfo("Node stopped")