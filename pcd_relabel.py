# This file calls the get_plane_distances binary for all pcd files in a directory
import argparse
import glob
import subprocess

# Wrapper for the set_laser_id_as_label program which allows the user the specify a glob expression as the input

set_laser_id_as_label = "./cpp/cmake-build-debug/set_laser_id_as_label"

def get_distances(g):
    files = sorted(glob.glob(g))
    for filename in files:
        if filename.endswith(".pcd"):
            subprocess.run([set_laser_id_as_label, "-f", filename])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for .pcd files")
    args = parser.parse_args()
    g = args.g
    get_distances(g)
