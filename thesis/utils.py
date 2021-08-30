import numpy as np

# LUT between the laser ids and the laser angles.
# The position in the 16-element array corresponds with the laser id
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

# And vice versa
laser_angle_to_id = {y:x for x,y in laser_id_to_angle.items()}

sorted_angles = sorted(laser_angle_to_id.keys())
sorted_ids_by_angle = [laser_angle_to_id[angle] for angle in sorted_angles]


def load_measurement_file(name):
    return np.loadtxt(name)