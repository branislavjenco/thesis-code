import glob
import open3d as o3d
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for point cloud files")
    args = parser.parse_args()
    g = args.g


    files = sorted(glob.glob(g))
    for filename in files:
        print("Processing", filename)
        # pcd = o3d.t.io.read_point_cloud(filename, format="xyzi")
        # output_filename = filename.replace(".txt", ".ply")
        # o3d.t.io.write_point_cloud(output_filename, pcd)


