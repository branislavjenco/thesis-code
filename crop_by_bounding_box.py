import open3d as o3d
import glob
import argparse
import numpy as np

# Used in section about PCC Arena to crop point clouds coming from Blensor scans

# 1. load the file scanned by Blensor
# 2. find the corresponding .txt point cloud file from StanfordDataset v1.2
# 3. get bounding box from 2
# 4. remove all points outside the bounding box from 1
# 5. save is txt right away

default_file_glob = "/home/branislav/repos/thesis/s3dis_scans_no_noise/*.pcd"
default_stanford_dir = "/home/branislav/Downloads/Stanford3dDataset_v1.2"


def get_bb(data):
    max_ = data.max(axis=0)
    min_ = data.min(axis=0)
    bb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_, max_bound=max_)
    return bb


def parse_crop_file(f):
    lines = f.readlines()
    data = [[float(x) for x in line.split(" ")[:3]] for line in lines]
    return np.array(data)


def get_area_and_room(filename):
    filename = filename.split("/")[-1]
    filename = filename.replace(".pcd", "")
    area = filename[-1]
    room = filename[:-2]
    return area, room


def crop(g=default_file_glob, crop_dir=default_stanford_dir):
    files = sorted(glob.glob(g))


    for filename in files:
        print("Processing", filename)
        pcd = o3d.io.read_point_cloud(filename)
        area, room = get_area_and_room(filename)
        crop_file = crop_dir + "/" + "Area_" + area + "/" + room + "/" + room + ".txt"
        data = None
        with open(crop_file) as f:
            data = parse_crop_file(f)
        bb = get_bb(data)
        cropped = pcd.crop(bb)

        output_filename = filename.replace(".pcd", ".ply")
        o3d.io.write_point_cloud(output_filename, cropped)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-g',
        help='glob expression for the PCD files',
        default=default_file_glob
    )
    parser.add_argument(
        '-s',
        help='Location of the base Stanford dataset (point clouds)',
        default=default_stanford_dir
    )

    args = parser.parse_args()
    g = args.g
    crop_dir = args.s
    crop(g, crop_dir)


if __name__ == "__main__":
    main()

