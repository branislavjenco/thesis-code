# This file calls the get_plane_distances binary for all pcd files in a directory
import argparse
import os
import subprocess

parser = argparse.ArgumentParser()

parser.add_argument("-d", help="Directory with .pcd files")
get_plane_distances = "/home/brano/Projects/github_indoor++/cmake-build-release/get_plane_distances"

args = parser.parse_args()
directory = args.d

for root, dirs, files in os.walk(directory):
    for name in sorted(files):
        if name.endswith(".pcd"):
            file = os.path.join(root, name)
            distances_file = f"{os.path.splitext(file)[0]}.txt"
            subprocess.run([get_plane_distances, "-f", file, "-o", distances_file, "-s"])
