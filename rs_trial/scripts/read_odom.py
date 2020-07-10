#!/usr/bin/env python

import rospy
import time
import sys
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

tstamp = ""

def callback_number(msg):
    global tstamp

    log_line = ""
    
    if msg._type == "tf2_msgs/TFMessage":
        log_line = "{0}, {1:.3f}, {2:.3f}, {3:.3f}".format( \
            msg.transforms[0].header.stamp.to_sec(), \
            msg.transforms[0].transform.translation.x, \
            msg.transforms[0].transform.translation.y, \
            msg.transforms[0].transform.translation.z)

    elif msg._type == "geometry_msgs/PoseStamped":
        log_line = "{0}, {1:.3f}, {2:.3f}, {3:.3f}".format( \
            msg.header.stamp.to_sec(), \
            msg.pose.position.x, \
            msg.pose.position.y, \
            msg.pose.position.z)

    elif msg._type == "nav_msgs/Odometry":
        log_line = "{0}, {1:.3f}, {2:.3f}, {3:.3f}".format( \
            msg.header.stamp.to_sec(), \
            msg.pose.pose.position.x, \
            msg.pose.pose.position.y, \
            msg.pose.pose.position.z)

    with open(myargv[1].replace("/", "_") + "_" + tstamp + ".csv", "a") as f:
        f.write(log_line + "\n")

    rospy.loginfo(msg._type)
    rospy.loginfo(log_line)

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) != 2:
        print("Invalid number of arguments.\n" +
              "Usage: rosrun rs_trial read_odom.py /topic method\n")
        sys.exit()

    rospy.init_node('read_odom', anonymous=True)

    print("Starting up ROS node: read_odom\n")

    tstamp = time.strftime("%Y%m%d-%H%M%S")

    # if len(myargv) == 2:
    if myargv[1] == "TF":
        sub = rospy.Subscriber("/tf", TFMessage, callback_number)
    elif myargv[1] == "RTABMAP":
        sub = rospy.Subscriber("/odometry/filtered", Odometry, callback_number)
    elif myargv[1] == "ORB2":
        sub = rospy.Subscriber("/orb_slam2_rgbd/pose", PoseStamped, callback_number)
    else:
        rospy.loginfo("Method " + myargv[1] + " not recognized.")
        sys.exit()

    # else:
    #     rospy.loginfo("usage: TODO")
    #     sys.exit()

    rospy.spin()    