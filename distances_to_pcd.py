import glob
import numpy as np
import open3d as o3d
import argparse
from pathlib import Path

# Translates a .txt "distance" file to a pcd point cloud

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for distance files")
    parser.add_argument("-o", help="Output directory")
    args = parser.parse_args()
    g = args.g
    o = args.o

    files = sorted(glob.glob(g))
    for filename in files:
        print("Processing", filename)
        filepath = Path(filename)

        # 1. read the file
        lines = []
        with open(filename) as f:
            lines = [[float(x) for x in l.rstrip().split(" ")][1:4] for l in f.readlines()]


        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.utility.Vector3dVector(np.array(lines))
        #2. use ply extension

        output_filename = filepath.name.replace(".txt", ".pcd")


        # 4. make an output path using the output dir and the new name
        output_path = Path(o).joinpath(output_filename)

        # 5. write the new file
        o3d.t.io.write_point_cloud(output_path.as_posix(), pcl)


