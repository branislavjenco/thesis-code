import os
import argparse

from reader import EvdReader, TXTWriter


def evds_to_txts(d):
    for root, dirs, files in os.walk(d):
        for name in sorted(files):
            if name.endswith(".evd"):
                _file = os.path.join(root, name)
                reader = EvdReader(_file)
                writer = TXTWriter(f"{os.path.splitext(_file)[0]}.txt")
                writer.write_txt_file(reader.get_rays())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-d',
        help = 'directory with EVD files'
    )

    args = parser.parse_args()
    d = args.d
    evds_to_txts(d)

