import bpy
import os
import sys
import argparse
import blensor
from math import radians

'''
1. Add two planes
2. select them one by one and set their location and rotation
3. add a scanner like it's done below
4. set it to the five locations and add the keyframes, almost same code as below
'''


argv = sys.argv

if "--" not in argv:
    argv = []  # as if no args are passed
else:
    argv = argv[argv.index("--") + 1:]  # get all args after "--"

parser = argparse.ArgumentParser()

parser.add_argument(
    '-e',
    help = 'Error keyword (none, ray, distance, incidence)',
    default = "none"
)

args = parser.parse_args(argv)
error = args.e
print(f"Using error type: {error}")

# the real scans were roughly 1.6 meters wide and 2.2 meters tall

VLP16 = "vlp16"
bpy.context.scene.render.fps = 120
sweep_duration = 120 * 4
runs_per_distance = 1
scan_types = ["wall", "floor"]
scanner_locations = ((0, 5, 1), (0, 4, 1), (0, 3, 1) ,(0, 2, 1) ,(0, 1, 1))
# scanner_locations = [(0, 5, 1)]
rotation_start = -60 + 90
rotation_end = 60 + 90


def make_plane(name="plane", vertices=None):
    if vertices is None:
        vertices = [(-1, -1, 0), (-1, 1, 0), (1, 1, 0), (1, -1, 0)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    faces = [(0, 1, 2, 3)]
    new_mesh = bpy.data.meshes.new(name)
    new_mesh.from_pydata(vertices, edges, faces)
    new_mesh.update()
    new_object = bpy.data.objects.new(name, new_mesh)
    bpy.context.scene.objects.link(new_object)
    return new_object


def remove_object(name):
    try:
       o = bpy.data.objects[name]
    except KeyError:
        return
    bpy.data.objects.remove(o, do_unlink=True)


def add_wall():
    remove_object("wall")
    wall = make_plane("wall", vertices=[(-0.8, 0, 0), (-0.8, 0.0, 2.2), (0.8, 0.0, 2.2), (0.8, 0.0, 0.0)])
    wall.name = "wall"


def assert_floor(sensor_distance):
    remove_object("floor")
    floor = make_plane("floor", vertices=[(-0.8, sensor_distance, 0), (-0.8, 0, 0), (0.8, 0, 0), (0.8, sensor_distance, 0)])
    floor.name = "floor"


def add_scanner():
    print("Adding scanner to scene")
    scanner_data = bpy.data.cameras.new(VLP16)
    scanner_obj = bpy.data.objects.new(VLP16, scanner_data)
    scanner_obj.scan_type = "velodyne"
    scanner_obj.velodyne_model = VLP16
    scanner_obj.ref_dist = scanner_obj.velodyne_ref_dist
    scanner_obj.ref_limit = scanner_obj.velodyne_ref_limit
    scanner_obj.ref_slope = scanner_obj.velodyne_ref_slope
    scanner_obj.local_coordinates = False

    bpy.context.scene.objects.link(scanner_obj)
    bpy.context.scene.camera = scanner_obj
    bpy.context.scene.objects.active = scanner_obj

    # Set up animation
    for i, loc in enumerate(scanner_locations):
        # Set scanner to location
        scanner_obj.location = loc
        scanner_obj.keyframe_insert(data_path="location", frame=sweep_duration * i)

        scanner_obj.rotation_euler = (radians(rotation_start), 0, radians(180))
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * i)

        scanner_obj.rotation_euler = (radians(rotation_end), 0, radians(180))
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * (i + 1) - 1)

    # Set up animation curves
    for fcurve in scanner_obj.animation_data.action.fcurves:
        if fcurve.data_path == 'location':
            for kfp in fcurve.keyframe_points:
                kfp.interpolation = 'CONSTANT'
        else:
            for kfp in fcurve.keyframe_points:
                kfp.interpolation = 'BEZIER'

    return scanner_obj


def scan_and_save(scanner_obj, runs, scan_type):
    print("Scanning")
    blensor.evd.output_labels = True
    if scan_type == "wall":
        add_wall()
    for run in range(runs):
        for i, loc in enumerate(scanner_locations):
            # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
            dist = loc[1]
            if scan_type == "floor":
                # The floor size depends to where we now put the sensor
                # (so that we don't have to crop the point cloud later)
                assert_floor(dist)
            params = blensor.blendodyne.vlp16_parameters
            use_incidence_angle = True if error == "incidence" else False
            add_beam_divergence = True if error == "divergence" else False
            if error == "none":
                noise_mu = 0.0
                noise_sigma = 0.0
                params["distance_bias_noise_mu"] = 0.0
                params["distance_bias_noise_sigma"] = 0.0
            elif error == "default":
                noise_mu = 0.0
                noise_sigma = 0.01
                params["distance_bias_noise_mu"] = 0.0
                params["distance_bias_noise_sigma"] = 0.078
            elif error == "base_vlp16":
                noise_mu = 0.0
                noise_sigma = 0.01
                params["distance_bias_noise_mu"] = 0.0
                params["distance_bias_noise_sigma"] = 0.014
            else:
                noise_mu = params["noise_mu"]
                noise_sigma = params["noise_sigma"]

            blensor.blendodyne.scan_range(scanner_obj,
                                          angle_resolution=params["angle_resolution"],
                                          rotation_speed=params["rotation_speed"],
                                          max_distance=params["max_dist"],
                                          noise_mu=noise_mu,
                                          noise_sigma=noise_sigma,
                                          filename=f"/home/branislav/repos/thesis/new_virtual_error_measurements/{error}/{dist}m_{run}_{scan_type}.evd",
                                          frame_start=sweep_duration * i,
                                          frame_end=sweep_duration * (i + 1) - 1,
                                          world_transformation=scanner_obj.matrix_world,
                                          use_incidence_angle=use_incidence_angle,
                                          add_beam_divergence=add_beam_divergence,
                                          output_laser_id_as_color=True
                                          )


for scan_type in scan_types:
    bpy.ops.wm.open_mainfile(filepath="/home/branislav/repos/thesis/empty.blend")
    scanner_obj = add_scanner()
    scan_and_save(scanner_obj, runs_per_distance, scan_type)

