#!/usr/bin/env python

import rospy
from rospy_tutorials.srv import AddTwoInts

if __name__ == '__main__':
    rospy.init_node("add_two_ints_client", anonymous=True)

    rospy.wait_for_service("/add_two_ints")

    while not rospy.is_shutdown():
        try:
            rate = rospy.Rate(1) #1Hz

            add_two_ints = rospy.ServiceProxy("/add_two_ints", AddTwoInts)
            #for i in range(3):
            #while True:
            response = add_two_ints(2, 6)
            rospy.loginfo("Sum is: " + str(response.sum))
            rate.sleep()

        except rospy.ServiceException as e:
            rospy.logwarn("Service failed: " + str(e))

