import os
import glob
import argparse

from reader import EvdReader, PCLWriter

# Translate BlenSor's EVD file int a PCD file

def evds_to_pcds(g):
    files = sorted(glob.glob(g))
    for filename in sorted(files):
        if filename.endswith(".evd"):
            reader = EvdReader(filename)
            new_filename = os.path.splitext(filename)[0]
            writer = PCLWriter(f"{new_filename}")
            writer.write_pcl_file(reader.get_rays())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-g',
        help = 'glob expression for the EVD files'
    )

    args = parser.parse_args()
    g = args.g
    evds_to_pcds(g)

