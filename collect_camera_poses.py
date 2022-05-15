# Collect the poses for the cameras in the whole s3dis dataset
import json
from collections import defaultdict
import argparse
import glob

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", help="Glob expression for the pose json files")
    parser.add_argument("-o", help="Output file")
    args = parser.parse_args()
    g = args.g
    o = args.o

    locations = defaultdict(set)
    files = sorted(glob.glob(g))
    for file in files:
        with open(file) as f:
            data = json.load(f)
            room = data["room"]
            locations[room].add(tuple(data["camera_location"]))

    for room in locations.keys():
        locations[room] = list(locations[room])

    with open(o, 'w') as f:
        json.dump(locations, f)

