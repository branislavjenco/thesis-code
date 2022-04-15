import glob
import open3d as o3d
import argparse
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for point cloud files")
    parser.add_argument("-o", help="Output directory")
    parser.add_argument("-f", help="What format are the files")
    args = parser.parse_args()
    g = args.g
    o = args.o
    f = args.f


    files = sorted(glob.glob(g))
    for filename in files:
        print("Processing", filename)
        filepath = Path(filename)
        
        # 1. read the file
        pcd = None
        if f == "txt":
            o3d.t.io.read_point_cloud(filename, format="xyz")
        elif f == "obj":
            o3d.t.io.read_point_cloud(filename)

        # 2. use ply extension
        output_filename = filepath.name.replace("." + f, ".ply")

        # 3. get the parent folder name from the path and prefix it to the file name
        area = filepath.parents[1].name
        output_filename = area + "_" + output_filename
        
        # 4. make an output path using the output dir and the new name
        output_path = Path(o).joinpath(output_filename)

        # 5. write the new file
        o3d.t.io.write_point_cloud(output_path.as_posix(), pcd)


