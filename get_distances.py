# This file calls the get_plane_distances binary for all pcd files in a directory
import argparse
import os
import glob
import subprocess

get_plane_distances = "./cpp/cmake-build-debug/get_plane_distances"


def get_distances(g):
    files = sorted(glob.glob(g))
    for filename in files:
        if filename.endswith(".pcd"):
            distances_file = f"{os.path.splitext(filename)[0]}.txt"
            # run with -s parameter to skip cropping
            subprocess.run([get_plane_distances, "-f", filename, "-o", distances_file, "-s", "1"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for .pcd files")
    args = parser.parse_args()
    g = args.g
    get_distances(g)


