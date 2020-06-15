#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback_ping(msg):
    # rospy.loginfo(msg.data)
    new_msg = String()
    new_msg.data = msg.data + " | " + "Nice to meet you, I am ROS"
    pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('ping_pong')
    sub = rospy.Subscriber("/ping_to_ros_topic", String, callback_ping)
    pub = rospy.Publisher("/ros_to_pong_topic", String, queue_size=10)
    rospy.spin()