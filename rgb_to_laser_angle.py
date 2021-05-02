# This file simply rewrites the distances files so that the lasers in the last column are represented by
# their angle w.r.t to the sensor as opposed to the rgb color that we captured by Blensor
import math
import argparse
import glob

laser_angles_vlp16 = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]


# Takes in a single 0-15 integer value for laser id
def laser_id_to_color(laser_id):
    color = laser_id / len(laser_angles_vlp16)
    color = int(color * 255) # this gets saved in the .evd file and also as a label in the PCD file
    return color


# Takes in a single 0-255 integer value for color
def color_to_laser_id(color):
    laser_id = (color / 255) * len(laser_angles_vlp16)
    return math.ceil(laser_id)


def color_to_laser_angle(color):
    return laser_angles_vlp16[color_to_laser_id(color)]

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
        output_filename = filename[:-4] + "_with_angles.txt"
        with open(filename) as input_file:
            with open(output_filename, "w") as output_file:
                for line in input_file.readlines():
                    tokens = line.split(" ")
                    tokens[5] = str(color_to_laser_angle(int(tokens[5])))
                    output_file.write(" ".join(tokens) + "\n")


