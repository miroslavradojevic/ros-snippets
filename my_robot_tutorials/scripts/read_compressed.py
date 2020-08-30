#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from os.path import exists, isfile, islink, join, isdir
from os import mkdir, unlink, listdir
import shutil

def clear_dir(dir_path):
    for filename in listdir(dir_path):
        file_path = join(dir_path, filename)
        try:
            if isfile(file_path) or islink(file_path):
                unlink(file_path)
            elif isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete {}. Reason: {}".format(file_path, e))


def callback(ros_data):
    timestamp = ros_data.header.stamp.to_nsec()
    # print("received image of type: {:s}".format(ros_data.format))
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    fname = join(out_dir, str(timestamp) + ".jpg")
    # print("{} {} {}".format(type(image_np), image_np.shape, fname))
    cv2.imwrite(fname, image_np)

out_dir = "camera_kodak_sp360_image_compressed"

if __name__ == '__main__':
    rospy.init_node('compressed_image_reader', anonymous=True)

    if not exists(out_dir):
        mkdir(out_dir)
    else:
        clear_dir(out_dir)

    sub = rospy.Subscriber("/camera_kodak_sp360/image/compressed", CompressedImage, callback, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")