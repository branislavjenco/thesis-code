import bpy
import bmesh
import sys
import os
import argparse
import json
import blensor
from math import radians, pi
import random
argv = sys.argv

if "--" not in argv:
    argv = []  # as if no args are passed
else:
    argv = argv[argv.index("--") + 1:]  # get all args after "--"

parser = argparse.ArgumentParser()

parser.add_argument(
    '-o',
    help = 'Output directory',
    default = "/home/branislav/repos/thesis/s3dis_scans/"
)

parser.add_argument(
    '-l',
    help = 'json file with locations in array',
)

parser.add_argument(
    '-f',
    help = 'obj file with mesh',
)

args = parser.parse_args(argv)
output_dir = args.o
locations_file = "/home/branislav/repos/thesis/locations_pruned.json"
mesh_file = "/home/branislav/repos/s3dis/area_1/3d/rgb.obj"

locations = []
with open(locations_file) as f:
    locations = json.load(f)

bpy.context.scene.render.fps = 24
sweep_duration = 24 * 4
rotation_start = -60 + 90
rotation_end = 60 + 90


poses = [{
    "position": loc,
    "rotation_start": (radians(rotation_start), 0, 0),
    "rotation_end": (radians(rotation_end), 0, 0)
} for loc in locations.values()]

VLP16 = "vlp16"


def assert_group(group_name):
    if group_name in bpy.data.groups:
        group = bpy.data.groups[group_name]
    else:
        group = bpy.data.groups.new(group_name)

    return group

# 1. Clear scene entirely
bpy.ops.wm.open_mainfile(filepath="/home/branislav/repos/thesis/empty.blend")
scn = bpy.context.scene

# 2. Import Area mesh
imported_mesh = bpy.ops.import_scene.obj(filepath=mesh_file)

# 3. Add all current objects into the room group
room_group = assert_group("room")
for obj in bpy.data.objects:
    room_group.objects.link(obj)

# 4. Add camera (lidar)
print("Adding scanner to scene")
scanner_data = bpy.data.cameras.new(VLP16)
scanner_obj = bpy.data.objects.new(VLP16, scanner_data)
scanner_obj.scan_type = "velodyne"
scanner_obj.velodyne_model = VLP16
scanner_obj.ref_dist = scanner_obj.velodyne_ref_dist
scanner_obj.ref_limit = scanner_obj.velodyne_ref_limit
scanner_obj.ref_slope = scanner_obj.velodyne_ref_slope
scanner_obj.local_coordinates = False
scn.objects.link(scanner_obj)
scn.camera = scanner_obj
bpy.context.scene.objects.active = scanner_obj


# 5. Prune the locations from their original number
# L = min(len(locations), 700)
# pruned_locations = []
# for i in range(0, L, 200):
#     pruned_locations.append(locations[i])
# locations = pruned_locations


# 6. Set up animation
for i, loc in enumerate(poses):
    # Set scanner to location
    scanner_obj.location = loc["position"]
    scanner_obj.keyframe_insert(data_path="location", frame=sweep_duration * i)

    scanner_obj.rotation_euler = loc["rotation_start"]
    scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * i)

    scanner_obj.rotation_euler = loc["rotation_end"]
    scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * (i + 1) - 1)

for fcurve in scanner_obj.animation_data.action.fcurves:
    if fcurve.data_path == 'location':
        for kfp in fcurve.keyframe_points:
            kfp.interpolation = 'CONSTANT'
    else:
        for kfp in fcurve.keyframe_points:
            kfp.interpolation = 'BEZIER'

# 7. Scan
blensor.evd.output_labels = True
print("Scanning")
# The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
for i, room in enumerate(locations.keys()):
    output_filename = output_dir + f"{room}.evd"
    blensor.blendodyne.scan_range(scanner_obj,
                                  filename=output_filename,
                                  frame_start=sweep_duration * i,
                                  frame_end=sweep_duration * (i + 1) - 1,
                                  world_transformation=scanner_obj.matrix_world,
                                  add_blender_mesh=True,
                                  depth_map=True)

