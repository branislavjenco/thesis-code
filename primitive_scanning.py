import bpy
import bmesh
import sys
import os
import argparse
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
    default = "/home/branislav/repos/thesis/primitives"
)


args = parser.parse_args(argv)
output_dir = args.o

bpy.context.scene.render.fps = 24
sweep_duration = 24 * 4
rotation_start = -60 + 90
rotation_end = 60 + 90

locations = [{
    "position": (0.0, -3.0, 1.0),
    "rotation_start": (radians(rotation_start), 0, 0),
    "rotation_end": (radians(rotation_end), 0, 0)
}, {
    "position": (0.0, 3.0, 1.0),
    "rotation_start": (radians(rotation_start), 0, radians(180)),
    "rotation_end": (radians(rotation_end), 0, radians(180))
}]

VLP16 = "vlp16"


def set_color(obj, rgba):
    obj.color = rgba
    mat = bpy.data.materials.new(f"Mat-{str(rgba)}")
    mat.diffuse_color = (rgba[0], rgba[1], rgba[2])
    mat.specular_color = (rgba[0], rgba[1], rgba[2])
    mat.use_shadeless = True
    obj.data.materials.append(mat)

def make_cuboid(position, rotation, scale, name="cube"):
    # Origin point transformation settings
    mesh_offset = position
    origin_offset = (0, 0, 0)

    def vert(x,y,z):
        """ Make a vertex """

        return (x + origin_offset[0], y + origin_offset[1], z + origin_offset[2])

    verts = [vert(1.0, 1.0, -1.0),
             vert(1.0, -1.0, -1.0),
             vert(-1.0, -1.0, -1.0),
             vert(-1.0, 1.0, -1.0),
             vert(1.0, 1.0, 1.0),
             vert(1.0, -1.0, 1.0),
             vert(-1.0, -1.0, 1.0),
             vert(-1.0, 1.0, 1.0)]


    faces = [(0, 1, 2, 3),
             (4, 7, 6, 5),
             (0, 4, 5, 1),
             (1, 5, 6, 2),
             (2, 6, 7, 3),
             (4, 0, 3, 7)]


    # -----------------------------------------------------------------------------
    # Add Object to Scene

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.objects.link(obj)

    # -----------------------------------------------------------------------------
    # Offset mesh to move origin point

    obj.location = [(i * -1) + mesh_offset[j] for j, i in enumerate(origin_offset)]
    obj.rotation_euler = rotation
    obj.scale = scale
    set_color(obj, (1, 0, 0, 1))
    return obj

def make_random_cuboid():
    position = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0.0, 2.0))
    rotation = (random.uniform(0, pi), random.uniform(0, pi), random.uniform(0, pi))
    scale = (random.uniform(0.1, 2.0), random.uniform(0.1, 2.0), random.uniform(0.1, 2.0))

    return make_cuboid(position, rotation, scale)


def make_cylinder(position, rotation, radius, length, name="cylinder"):
    mesh_data = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        segments=100,
        diameter1=radius,
        diameter2=radius,
        depth=length)
        
    bm.to_mesh(mesh_data)
    bm.free()

    mesh_obj = bpy.data.objects.new(mesh_data.name, mesh_data)
    bpy.context.scene.objects.link(mesh_obj)
    mesh_obj.location = position
    mesh_obj.rotation_euler = rotation
    set_color(mesh_obj, (0, 1, 0, 1))
    return mesh_obj

def make_random_cylinder():
    position = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0.0, 2.0))
    rotation = (random.uniform(0, pi), random.uniform(0, pi), random.uniform(0, pi))
    radius = random.uniform(0.1, 1.0)
    length = random.uniform(0.1, 10.0)

    return make_cylinder(position, rotation, radius, length)

def scan_random_object(iteration):
    # Clear scene entirely
    bpy.ops.wm.open_mainfile(filepath="/home/branislav/repos/thesis/empty.blend")
    scn = bpy.context.scene

    primitive = None
    print("Creating primitive mesh.")
    if random.uniform(0, 1) > 0.5:
        make_random_cuboid()
        primitive = "Cuboid"
    else:
        make_random_cylinder()
        primitive = "Cylinder"


    # Add camera (lidar)
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

    # Set up animation
    for i, loc in enumerate(locations):
        # Set scanner to location
        scanner_obj.location = loc["position"]
        scanner_obj.keyframe_insert(data_path="location", frame=sweep_duration * i)

        scanner_obj.rotation_euler = loc["rotation_start"]
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * i)

        scanner_obj.rotation_euler = loc["rotation_end"]
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * (i + 1) - 1)

    print("Scanning")

    for fcurve in scanner_obj.animation_data.action.fcurves:
        if fcurve.data_path == 'location':
            for kfp in fcurve.keyframe_points:
                kfp.interpolation = 'CONSTANT'
        else:
            for kfp in fcurve.keyframe_points:
                kfp.interpolation = 'BEZIER'

    # Do scanning
    blensor.evd.output_labels = True
    print("Scanning")
    # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
    output_filename = output_dir + f"/{primitive}-{iteration}.evd"
    blensor.blendodyne.scan_range(scanner_obj,
                                  filename=output_filename,
                                  frame_start=0,
                                  frame_end=len(locations)*sweep_duration,
                                  world_transformation=scanner_obj.matrix_world)

for i in range(10):
    scan_random_object(i)
