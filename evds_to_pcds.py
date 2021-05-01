import os
import argparse

from reader import EvdReader, PCLWriter

parser = argparse.ArgumentParser()
parser.add_argument(
    '-d',
    help = 'directory with EVD files'
)

args = parser.parse_args()
d = args.d

for root, dirs, files in os.walk(d):
    for name in sorted(files):
        if name.endswith(".evd"):
            file = os.path.join(root, name)
            reader = EvdReader(file)
            writer = PCLWriter(f"{os.path.splitext(file)[0]}")
            writer.write_pcl_file(reader.get_rays())
