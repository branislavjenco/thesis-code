import open3d as o3d
import glob
import numpy as np

# 1. load the file scanned by Blensor
# 2. find the corresponding .txt point cloud file from StanfordDataset v1.2
# 3. get bounding box from 2
# 4. remove all points outside the bounding box from 1
# 5. save is txt right away


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


def main():
    pattern = "/home/branislav/repos/thesis/s3dis_scans_2/*.pcd"
    crop_dir = "/home/branislav/Downloads/Stanford3dDataset_v1.2"
    files = sorted(glob.glob(pattern))


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

if __name__ == "__main__":
    main()

