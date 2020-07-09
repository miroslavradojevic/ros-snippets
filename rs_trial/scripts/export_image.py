#!/usr/bin/env python
# Hints at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import roslib
import sys
import os
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self, topic_name):
        print("Listening topic: " + topic_name + "\n")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback)

    def callback(self, data):
        fdir = "{:.3f}".format(data.header.stamp.to_sec())  # str(data.header.stamp.to_sec())
        fname = fdir + self.image_sub.name.replace("/", "_") + ".png"
        fname = os.path.join(fdir, fname)

        if not os.path.exists(fdir):
            os.makedirs(fdir)

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "passthrough")  # "bgr8" "mono16"
        except CvBridgeError as e:
            print(e)

        # print(cv_image.dtype, cv_image[0, 0].dtype)

        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imwrite(fname, frame)
        else:
            # https://stackoverflow.com/questions/44606257/imwrite-16-bit-png-depth-image
            cv2.imwrite(fname, frame.astype(np.uint16))  # depth is 16 bit

        print(fname)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) != 2:
        print(
            "Invalid number of arguments.\nUsage: rosrun rs_trial export_image.py /topic\n")
        sys.exit(1)

    rospy.init_node('export_image', anonymous=True)

    print("Starting up ROS node: export_image\n")

    ic = ImageConverter(myargv[1])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Exit now")
