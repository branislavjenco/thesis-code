import subprocess
import glob
import argparse

# Wrapper for the get_centroids program which allows the user to specify a glob expression as the input

cmd = "./cpp/cmake-build-release/get_centroids"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for distance/pcd files")
    parser.add_argument("-t", help="1 for distance files, 0 for pcd files")
    args = parser.parse_args()
    g = args.g
    t = args.t

    files = sorted(glob.glob(g))
    for f in files:
        input_file = f
        output_file = input_file[:-4] + "_centroids.txt"
        subprocess.run([cmd, "-f", input_file, "-o", output_file, "-t", t])
