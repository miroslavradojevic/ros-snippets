#!/usr/bin/env python
import rospy
import sys
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from os.path import exists, isfile, islink, join, isdir, basename, splitext
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
    print("received image of type: {:s}".format(ros_data.format))
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    print(type(image_np), image_np.shape)
    fname = join(out_dir, "{}.{}".format(str(timestamp)[:10], str(timestamp)[10:]) + ".jpg")
    cv2.imwrite(fname, image_np)
    print("{}".format(fname))


out_dir = "CompressedImage_output"

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) != 2:
        print("Usage: rosrun my_robot_tutorials {} /compressed_image_topic_name".format(basename(__file__)))
        exit()

    node_name = splitext(basename(__file__))[0]
    rospy.init_node(node_name, anonymous=True)

    print("Starting up node: " + rospy.get_name())

    topic_name = myargv[1]
    print("Subscribe to topic: " + topic_name)

    out_dir = topic_name.replace("/", "_")
    if len(out_dir) > 0:
        out_dir = out_dir[1:]  # remove first char, originally "/" in topic names

    if not exists(out_dir):
        mkdir(out_dir)
    else:
        clear_dir(out_dir)

    sub = rospy.Subscriber(topic_name, CompressedImage, callback, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
