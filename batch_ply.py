import glob
import subprocess as sp
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for point cloud files")
    parser.add_argument("-x", help="Use xvfb-run")
    parser.add_argument("-cc", help="Location of CloudCompare executable")
    args = parser.parse_args()
    g = args.g
    x = args.x
    cc = args.cc

    cmd2 = cc
    cmd1 = ""
    if x == "1":
        cmd1 = "xvfb-run"

    files = sorted(glob.glob(g))
    for filename in files:
        print("Processing", filename)
        sp.run([cmd1, cmd2, "-SILENT", "-O", filename, "-C_EXPORT_FMT", "PLY", "-NO_TIMESTAMP", "-SAVE_CLOUDS" ])
    

