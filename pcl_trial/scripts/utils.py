import numpy as np
import sys
import shutil
import argparse
import open3d as o3d
from os.path import exists, splitext, join, dirname, abspath, basename, isfile, isdir, islink
from os import listdir, unlink


__author__ = "Miroslav Radojevic"
__email__ = "miroslav.radojevic@gmail.com"


def get_matching_indices(source, target, tolerance):
    '''Find source array indexes that have at least one match in target array: source Nx3, target Mx3'''
    # print(source.shape, target.shape)
    if source.shape[1] != target.shape[1]:
        return None

    source_match_idxs = []
    for i in range(source.shape[0]):
        if i % (source.shape[0] // 10) == 0:
            print("{}%".format(np.round(100.0 * i / source.shape[0], 1)))
            sys.stdout.flush()

        ee = np.nonzero(np.sum(np.square(source[i, :] - target), axis=1) <= tolerance**2)

        if len(ee) > 1:
            print("Found more than one dimension where there was supposed to be one.")

        if len(ee[0]) > 0:
            source_match_idxs.append(i) # take index of the point that had ee[0][:] indexes as match

    return source_match_idxs


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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Path to source pointcloud file (.pcd)", type=str)
    parser.add_argument("target", help="Path to target pointcloud file (.pcd)", type=str)
    args = parser.parse_args()

    if not exists(args.source) or not exists(args.target):
        exit("File could not be found")

    ext = splitext(args.source)[-1].lower()
    if ext is None or ext not in [".pcd"]:
        exit("Point-cloud file has wrong extension")

    pcd_source = o3d.io.read_point_cloud(args.source, format="pcd")
    pcd_target = o3d.io.read_point_cloud(args.target, format="pcd")

    pcd_source_arr = np.asarray(pcd_source.points)[:, :3]
    pcd_target_arr = np.asarray(pcd_target.points)[:, :3]

    match_idx = get_matching_indices(pcd_source_arr, pcd_target_arr, 0.001)

    print("#matches={:d}, {:3.4f}% total".format(len(match_idx), 100.0*len(match_idx)/len(pcd_source_arr)))

    # create new source pcd copy with matches colored in red
    pcd_source.paint_uniform_color([0.7, 0.7, 0.7])
    np.asarray(pcd_source.colors)[match_idx, :] = [1, 0, 0]
    o3d.io.write_point_cloud(join(dirname(abspath(args.source)), basename(args.source) + "_red.pcd"), pcd_source)

    # From NumPy to open3d.PointCloud
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(np.asarray(pcd_source.points)[match_idx, :])
    o3d.io.write_point_cloud(join(dirname(abspath(args.source)), basename(args.source) + "_match.pcd"), pcd1)