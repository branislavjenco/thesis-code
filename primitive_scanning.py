import bpy
import bmesh
import sys
import os
import argparse
import blensor
from math import radians, pi
import random
argv = sys.argv

# Testing programmatic adding of primitives into scene to be scanned by BlenSor. Ultimately not used in the thesis.

color_cuboid = (1, 0, 0, 1) # red
color_cylinder = (0, 1, 0, 1) # green
color_floor = (0, 0, 1, 1) # blue
color_ceiling = (1, 1, 0, 1) # yellow
color_wall = (1, 0, 1, 1) # magenta

if "--" not in argv:
    argv = []  # as if no args are passed
else:
    argv = argv[argv.index("--") + 1:]  # get all args after "--"

parser = argparse.ArgumentParser()

parser.add_argument(
    '-o',
    help = 'Output directory',
    default = "/home/branislav/repos/thesis/primitives2"
)

args = parser.parse_args(argv)
output_dir = args.o

bpy.context.scene.render.fps = 24
sweep_duration = 24 * 4
rotation_start = -60 + 90
rotation_end = 60 + 90

locations = [{
   "position": (0.0, 1.5, 1.0),
   "rotation_start": (radians(rotation_start), 0, 0),
   "rotation_end": (radians(rotation_end), 0, 0)
},{
   "position": (0.0, -1.5, 1.0),
   "rotation_start": (radians(rotation_start), 0, 0),
   "rotation_end": (radians(rotation_end), 0, 0)
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

    #obj.location = [(i * -1) + mesh_offset[j] for j, i in enumerate(origin_offset)]
    obj.rotation_euler = rotation
    obj.scale = scale
    set_color(obj, color_cuboid)
    return obj

def make_random_cuboid():
    position = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0.0, 2.0))
    # rotation = (random.uniform(0, pi), random.uniform(0, pi), random.uniform(0, pi))
    rotation = (0.0, 0.0, random.uniform(0, pi)) # contrain round Z
    scale = (random.uniform(0.1, 1.0), random.uniform(0.1, 1.0), random.uniform(0.1, 1.0))

    return make_cuboid(position, rotation, scale)

def make_room():
    # Origin point transformation settings
    origin_offset = (0, 0, 0)
    floor_level = 0.0
    ceiling_level = 2.2
    extent_x = (random.uniform(-2, -3), random.uniform(2, 3))
    extent_y = (random.uniform(-2, -3), random.uniform(2, 3))

    def vert(x, y, z):
        """ Make a vertex """

        return (x + origin_offset[0], y + origin_offset[1], z + origin_offset[2])

    vs = [vert(extent_x[1], extent_y[1], floor_level),
             vert(extent_x[1], extent_y[0], floor_level),
             vert(extent_x[0], extent_y[0], floor_level),
             vert(extent_x[0], extent_y[1], floor_level),
             vert(extent_x[1], extent_y[1], ceiling_level),
             vert(extent_x[1], extent_y[0], ceiling_level),
             vert(extent_x[0], extent_y[0], ceiling_level),
             vert(extent_x[0], extent_y[1], ceiling_level)]

    floor_v = [vs[0], vs[1], vs[2], vs[3]]
    floor_f = [(0,1,2,3)]
    mesh = bpy.data.meshes.new("floor")
    mesh.from_pydata(floor_v, [], floor_f)
    obj = bpy.data.objects.new("floor", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_floor)

    ceiling_v = [vs[4], vs[5], vs[6], vs[7]]
    ceiling_f = [(0,1,2,3)]
    mesh = bpy.data.meshes.new("ceiling")
    mesh.from_pydata(ceiling_v, [], ceiling_f)
    obj = bpy.data.objects.new("ceiling", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_ceiling)

    wall1_v = [vs[0], vs[4], vs[5], vs[1]]
    wall1_f = [(0, 1, 2, 3)]
    mesh = bpy.data.meshes.new("wall1")
    mesh.from_pydata(wall1_v, [], wall1_f)
    obj = bpy.data.objects.new("wall1", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_wall)

    wall2_v = [vs[1], vs[5], vs[6], vs[2]]
    wall2_f = [(0, 1, 2, 3)]
    mesh = bpy.data.meshes.new("wall2")
    mesh.from_pydata(wall2_v, [], wall2_f)
    obj = bpy.data.objects.new("wall2", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_wall)

    wall3_v = [vs[2], vs[6], vs[7], vs[3]]
    wall3_f = [(0, 1, 2, 3)]
    mesh = bpy.data.meshes.new("wall3")
    mesh.from_pydata(wall3_v, [], wall3_f)
    obj = bpy.data.objects.new("wall3", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_wall)

    wall4_v = [vs[4], vs[0], vs[3], vs[7]]
    wall4_f = [(0, 1, 2, 3)]
    mesh = bpy.data.meshes.new("wall4")
    mesh.from_pydata(wall4_v, [], wall4_f)
    obj = bpy.data.objects.new("wall4", mesh)
    bpy.context.scene.objects.link(obj)
    set_color(obj, color_wall)

    # obj.location = [(i * -1) + mesh_offset[j] for j, i in enumerate(origin_offset)]
    # return obj

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
    #mesh_obj.location = position
    mesh_obj.rotation_euler = rotation
    set_color(mesh_obj, color_cylinder)
    return mesh_obj

def make_random_cylinder():
    position = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0.0, 2.0))
    rotation = (0, 0, 0)
    radius = random.uniform(0.1, 1.0)
    length = random.uniform(0.1, 2.0)

    return make_cylinder(position, rotation, radius, length)

def scan_random_object(iteration):
    # Clear scene entirely
    bpy.ops.wm.open_mainfile(filepath="/home/branislav/repos/thesis/empty.blend")
    scn = bpy.context.scene

    primitive = None
    make_room()
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
    params = blensor.blendodyne.vlp16_parameters
    print("Scanning")
    # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
    output_filename = output_dir + f"/{iteration}.evd"
    blensor.blendodyne.scan_range(scanner_obj,
                                  angle_resolution=params["angle_resolution"],
                                  rotation_speed=params["rotation_speed"],
                                  filename=output_filename,
                                  frame_start=0,
                                  frame_end=len(locations)*sweep_duration,
                                  world_transformation=scanner_obj.matrix_world,
                                  add_blender_mesh=False,
                                  depth_map=False)

for i in range(1):
    scan_random_object(i)
