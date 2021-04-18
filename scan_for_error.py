import bpy
import os
import blensor
from math import radians


frame_start = 0
frame_end = 72
sweep_duration = 72
VLP16 = "vlp16"

# Clear scene entirely
bpy.ops.wm.open_mainfile(filepath="/home/brano/Projects/thesis/empty.blend")
scn = bpy.context.scene

'''
1. Add two planes
2. select them one by one and set their location and rotation
3. add a scanner like it's done below
4. set it to the five locations and add the keyframes, almost same code as below
'''

def make_plane(name="plane"):
    vertices = [(-1, -1, 0), (-1, 1, 0), (1, 1, 0), (1, -1, 0)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    faces = [(0, 1, 2, 3)]
    new_mesh = bpy.data.meshes.new(name)
    new_mesh.from_pydata(vertices, edges, faces)
    new_mesh.update()
    new_object = bpy.data.objects.new(name, new_mesh)
    bpy.context.scene.objects.link(new_object)
    return new_object

wall = make_plane("wall")
floor = make_plane("floor")

wall.name = "wall"
wall.scale = (2.0, 5.0, 1.0)
wall.location = (0.0, 0.0, 0.0)
wall.rotation_euler = (radians(90.0), 0.0, 0.0)
floor.name = "floor"
floor.location = (0.0, -5.0, 0.0)
floor.rotation_euler = (0, 0, 0)
floor.scale = (2.0, 5.0, 1.0)


scanner_locations = ((0, -5, 1), (0, -4, 1), (0, -3, 1) ,(0, -2, 1) ,(0, -1, 1))

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

rotation_start = -30 + 90
rotation_end = 30 + 90

# Set up animation
for i, loc in enumerate(scanner_locations):
    # Set scanner to location
    scanner_obj.location = loc
    scanner_obj.keyframe_insert(data_path="location", frame=sweep_duration * i)

    scanner_obj.rotation_euler = (radians(rotation_start), 0, 0)
    scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * i)

    scanner_obj.rotation_euler = (radians(rotation_end), 0, 0)
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
for i, loc in enumerate(scanner_locations):
    print("Scanning")
    # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
    blensor.blendodyne.scan_range(scanner_obj,
                                  filename=f"/home/brano/Projects/thesis/virtual_error_measurements/test_{i}.evd",
                                  frame_start=sweep_duration * i,
                                  frame_end=sweep_duration * (i + 1) - 1,
                                  world_transformation=scanner_obj.matrix_world, output_laser_id_as_color=True)

