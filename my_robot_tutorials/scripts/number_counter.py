#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool

counter = 0
pub = None

def callback_number(msg):
    global counter
    counter += msg.data
    new_msg = Int64()
    new_msg.data = counter
    pub.publish(new_msg)

def callback_reset_counter(req):
    if req.data:
        global counter
        counter = 0
        return True, "Counter has been sucessfully reset"
    return False, "Counter has not been reset"


if __name__ == '__main__':
    rospy.init_node('number_counter')

    # subscribe node on a topic
    sub = rospy.Subscriber("/number", Int64, callback_number)

    #  publish counter on a new topic
    pub = rospy.Publisher("/number_count", Int64, queue_size=10)

    reset_service = rospy.Service("/reset_counter", SetBool, callback_reset_counter)

    rospy.spin() # if we don't want program to exit directly
