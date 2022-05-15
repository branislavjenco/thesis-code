import numpy as np
import argparse
import glob

# This script was used to group the planar residual (distances) data
# It has been superseeded by the code in the noise_distributions notebook


def load_measurement_file(name):
    return np.loadtxt(name)


laser_id_to_angle = {
    0: -15,
    1: 1,
    2: -13,
    3: 3,
    4: -11,
    5: 5,
    6: -9,
    7: 7,
    8: -7,
    9: 9,
    10: -5,
    11: 11,
    12: -3,
    13: 13,
    14: -1,
    15: 15
}

laser_angle_to_id = {y:x for x,y in laser_id_to_angle.items()}
sorted_angles = sorted(laser_angle_to_id.keys())


def compute_laser_deviations(measurement):
    deviations_per_laser = np.zeros(16)
    for angle in sorted_angles:
        # get distances and get their stddev per laser
        distances_for_angle = measurement[np.where(measurement[:,5] == angle)][:, 0]
        if len(distances_for_angle) == 0:
            distances_for_angle = [0]
        deviations_per_laser[laser_angle_to_id[angle]] = np.std(distances_for_angle)
    return deviations_per_laser

parser = argparse.ArgumentParser()
parser.add_argument(
    '-g',
    help = 'glob - use a wildcard to specify multiple txt files'
)

args = parser.parse_args()
g = args.g

files = sorted(glob.glob(g))
for filename in files:
    if filename.endswith(".txt"):
        measurement = load_measurement_file(filename)
        devs = compute_laser_deviations(measurement)
        new_filename = filename.replace(".txt", "_stddevs.txt")
        np.savetxt(new_filename, devs)


