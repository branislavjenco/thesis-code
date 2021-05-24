# This file calls the get_plane_distances binary for all pcd files in a directory
import argparse
import os
import glob
import subprocess

get_plane_distances = "/home/brano/Projects/github_indoor++/cmake-build-release/get_plane_distances"


def get_distances(g):
    files = sorted(glob.glob(g))
    for filename in files:
        if filename.endswith(".pcd"):
            distances_file = f"{os.path.splitext(filename)[0]}.txt"
            subprocess.run([get_plane_distances, "-f", filename, "-o", distances_file, "-s", "1"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for .pcd files")
    args = parser.parse_args()
    g = args.g
    get_distances(g)


