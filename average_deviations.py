import numpy as np
import argparse


def load_stddev_file(name):
    return np.loadtxt(name)


parser = argparse.ArgumentParser()
parser.add_argument(
    '-i',
    help='input directory'
)
parser.add_argument(
    '-o',
    help='output directory'
)
parser.add_argument(
    '-r',
    help='number of runs per sensor location'
)
args = parser.parse_args()
input_dir = args.i
output_dir = args.o
runs = int(args.r)

if not input_dir.endswith("/"):
    input_dir = input_dir + "/"

for kind in ["wall", "floor"]:
    averages_per_distance = np.zeros([5, 16])
    for distance in range(5):
        averages = np.zeros(16)
        for run in range(1, runs+1):
            filename = f"{distance+1}m_{run}_distances_{kind}_stddevs.txt"
            stddevs = load_stddev_file(input_dir + filename)
            averages = averages + stddevs
        averages_per_distance[distance, :] = averages / float(runs)
    np.savetxt(output_dir + kind + ".txt", averages_per_distance)
