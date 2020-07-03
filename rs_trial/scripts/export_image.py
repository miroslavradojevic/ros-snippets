#!/usr/bin/env python
# Hints at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# from __future__ import print_function

import roslib
import sys
import os
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self, image_topic_name):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        # print(image_topic_name)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            image_topic_name, Image, self.callback)


    def callback(self, data):
        fdir = str(data.header.stamp.to_sec())
        fname = str(data.header.stamp.to_sec()) + self.image_sub.name.replace("/", "_") + ".png" # todo - fix decimal number
        fname = os.path.join(fdir, fname) 

        if not os.path.exists(fdir):
            os.makedirs(fdir)

        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough") # "bgr8" "mono16"
        except CvBridgeError as e:
          print(e)

        # print(cv_image.dtype, cv_image[0, 0].dtype)

        if len(cv_image.shape) == 3:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(fname, cv_image)
        else:
            # https://stackoverflow.com/questions/44606257/imwrite-16-bit-png-depth-image
            cv2.imwrite(fname, cv_image.astype(np.uint16)) # depth is 16 bit
        
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        
        print(fname)


def main(args):
    print(args)
    ic = image_converter(args[1])
    rospy.init_node('export_image', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #  cv2.destroyAllWindows()


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    print(myargv)
    main(myargv)
