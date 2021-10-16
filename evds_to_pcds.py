import os
import argparse

from reader import EvdReader, PCLWriter


def evds_to_pcds(d):
    for root, dirs, files in os.walk(d):
        for name in sorted(files):
            if name.endswith(".evd"):
                _file = os.path.join(root, name)
                reader = EvdReader(_file)
                writer = PCLWriter(f"{os.path.splitext(_file)[0]}")
                writer.write_pcl_file(reader.get_rays())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-d',
        help = 'directory with EVD files'
    )

    args = parser.parse_args()
    d = args.d
    evds_to_pcds(d)

