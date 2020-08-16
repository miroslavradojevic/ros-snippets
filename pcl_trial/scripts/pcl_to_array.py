#!/usr/bin/env python
# dump lidar data into numpy from rosbag
# https://gist.github.com/bigsnarfdude/eeb156dc7b4caca69f5b31037da54708

import rospy
import sys
import os
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

# class PointCloudConverter:
#     # def __init__(self, topic_name):
#     #     self.
#
#     def callback(self, data):
#         pcl_array = self.pointcloud2_to_array(data)
#
#     def fields_to_dtype(fields, point_step):
#         '''
#         Convert a list of PointFields to a numpy record datatype.
#         '''
#         offset = 0
#         np_dtype_list = []
#         for f in fields:
#             while offset < f.offset:
#                 # might be extra padding between fields
#                 np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
#                 offset += 1
#
#             dtype = pftype_to_nptype[f.datatype]
#             if f.count != 1:
#                 dtype = np.dtype((dtype, f.count))
#
#             np_dtype_list.append((f.name, dtype))
#             offset += pftype_sizes[f.datatype] * f.count
#
#         # might be extra padding between points
#         while offset < point_step:
#             np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
#             offset += 1
#
#         return np_dtype_list
#
#     def pointcloud2_to_array(cloud_msg, squeeze=True):
#         ''' Converts a rospy PointCloud2 message to a numpy recordarray
#         Reshapes the returned array to have shape (height, width), even if the height is 1.
#         The reason for using np.fromstring rather than struct.unpack is speed... especially
#         for large point clouds, this will be <much> faster.
#         '''
#         # construct a numpy record type equivalent to the point type of this cloud
#         dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
#
#         # parse the cloud into an array
#         cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
#
#         # remove the dummy fields that were added
#         cloud_arr = cloud_arr[
#             [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
#
#         if squeeze and cloud_msg.height == 1:
#             return np.reshape(cloud_arr, (cloud_msg.width,))
#         else:
#             return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def fields_to_dtype(fields, point_step):
    '''
    Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def msg_to_arr(msg):
    dtype_list = fields_to_dtype(msg.fields, msg.point_step)
    arr = np.fromstring(msg.data, dtype_list)

    # remove the dummy fields that were added
    arr = arr[[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if msg.height == 1:
        return np.reshape(arr, (msg.width,))
    else:
        return np.reshape(arr, (msg.height, msg.width))


def callback(msg):
    timestamp = msg.header.stamp.to_nsec()
    fdir = "{:d}".format(timestamp)
    fname = fdir + pcl_sub.name.replace("/", "_") + ".npy"
    # fname = os.path.join(fdir, fname)

    arr = msg_to_arr(msg)

    np.save(fname, arr)

    print("seq={:d}, timestamp={:d}, file={:s}".format(msg.header.seq, timestamp, fname))


if __name__ == '__main__':
    print('%s: calling main function ... ' % os.path.basename(__file__))
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) != 2:
        print("Usage: rosrun pcl_trial", __file__, "/topic")
        exit()
    node_name = 'pcl_converter'
    topic_name = myargv[1]
    rospy.init_node(node_name, anonymous=True)
    print("Starting up ROS node: " + node_name + "\n")
    # pc_con = PointCloudConverter(topic_name)
    pcl_sub = rospy.Subscriber(topic_name, PointCloud2, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Exit now")
