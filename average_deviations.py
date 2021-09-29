import numpy as np
import glob
import argparse
import re


def load_stddev_file(name):
    return np.loadtxt(name)


parser = argparse.ArgumentParser()
parser.add_argument(
    '-g',
    help='input files as glob pattern'
)
parser.add_argument(
    '-o',
    help='output directory'
)
args = parser.parse_args()
input_glob = args.g
output_dir = args.o

files = sorted(glob.glob(input_glob))
print(files)

for kind in ["wall", "floor"]:
    averages_per_distance = np.zeros([5, 16])
    for distance in range(5):
        averages = np.zeros(16)
        r = re.compile(f"{distance+1}m_\d_{kind}_with_angles_stddevs.txt$")
        current_distance_files = list(filter(r.search, files))
        print(current_distance_files)
        for run in range(0, len(current_distance_files)):
            stddevs = load_stddev_file(current_distance_files[run])
            averages = averages + stddevs
        averages_per_distance[distance, :] = averages / float(len(current_distance_files))
    np.savetxt(output_dir + kind + ".txt", averages_per_distance)
