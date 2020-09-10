#!/usr/bin/env python
# dump lidar data into numpy from rosbag
# https://gist.github.com/bigsnarfdude/eeb156dc7b4caca69f5b31037da54708

import rospy
import sys
import os
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from utils import get_matching_indices, clear_dir
import numpy as np
from os.path import basename, splitext, abspath, dirname, join, exists

SAVE_NPY = False
DSAMPLE = True
DSAMPLE_VOX_SIZE = None # 0.05

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
    global DUMMY_FIELD_PREFIX
    dtype_list = fields_to_dtype(msg.fields, msg.point_step)
    arr = np.fromstring(msg.data, dtype_list)

    # remove the dummy fields that were added
    arr = arr[[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    arr_names = [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]

    nr_points = len(arr)
    nr_vals = len(arr_names)
    arr_vals = np.zeros((nr_points, nr_vals))

    for count, name in enumerate(arr_names):
        arr_vals[:, count] = arr[name]  # .transpose()

    return arr_vals, arr_names



def callback(msg):
    global out_dir, DSAMPLE_VOX_SIZE, DSAMPLE
    timestamp = msg.header.stamp.to_nsec()  # stamp.to_sec()
    # fdir = out_dir # "{:d}".format(timestamp)
    # fname = str(timestamp) + ".npy" # sub.name.replace("/", "_") + ".npy"

    arr_vals, arr_names = msg_to_arr(msg)
    arr_vals = arr_vals[~np.isnan(arr_vals).any(axis=1)] # filter rows with NaNs

    print("type={}, shape={} {}-{}".format(type(arr_vals), arr_vals.shape, np.amin(arr_vals), np.amax(arr_vals)))

    if SAVE_NPY:
        np.save(join(out_dir, str(timestamp) + ".npy"), arr_vals)

    if DSAMPLE:
        # From NumPy to open3d.PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(arr_vals[:, :3])  # 3 values xyz only
        if DSAMPLE_VOX_SIZE is not None:
            pcd = pcd.voxel_down_sample(voxel_size=DSAMPLE_VOX_SIZE)

        o3d.io.write_point_cloud(join(out_dir, str(timestamp) + ".pcd"), pcd)

        # compare with previous readout
        # if False:
        #     if arr_vals_prev is not None:
        #         match_indices = get_matching_indices(arr_vals_prev, np.asarray(pcd.points), 0.0001)
        #         print("{:3.2f}% indexes of previous frame match".format(100.0 * len(match_indices) / len(arr_vals_prev)))
        #
        #     arr_vals_prev = np.asarray(pcd.points)

        csv_file.write("{}, {}, {}\n".format(timestamp, len(arr_vals), len(pcd.points)))
        print("timestamp={:d}, #points={:d}, {:d} ({:3.2f}%)".format(timestamp,
                                                                     len(arr_vals),
                                                                     len(pcd.points),
                                                                     100.0 * len(pcd.points) / len(arr_vals)))
    else:
        csv_file.write("{}, {}\n".format(timestamp, len(arr_vals)))
        print("timestamp={:d}, #points={:d}".format(timestamp, len(arr_vals)))


out_file = "sequence_info.csv"
out_dir = "sequence_info"  # join(dirname(__file__), "sequence_info")
# arr_vals_prev = None

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) != 2:
        print("Usage: rosrun pcl_trial {} /velodyne_points_topic_name".format(__file__))
        exit()

    print("node_name = {}".format(__file__))
    print("basename = {}".format(basename(__file__)))
    print("node_name = {}".format(splitext(basename(__file__))[0]))

    node_name = splitext(basename(__file__))[0]  # "pcl_to_array"  # 'pcl_converter'
    rospy.init_node(node_name, anonymous=True)
    print("Starting up node: " + node_name + "\n")

    if not exists(out_dir):
        os.mkdir(out_dir)
    else:
        clear_dir(out_dir)

    csv_file = open(join(out_dir, out_file), 'w')

    topic_name = myargv[1]

    sub = rospy.Subscriber(topic_name, PointCloud2, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        csv_file.close()
        print("Shutting down PointCloud2 to array converter module")
