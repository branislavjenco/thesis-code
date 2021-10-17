import bpy
import sys
import os
import argparse
import blensor
from math import radians
"""
Workings of the blensor scanning:
- when doing a scan range from the UI, it first touches a file that is the exact filename
i.e. test.pcd will touch test.pcd, that's why there is an empty test.pcd after scanning
- normally it adds a "last frame" to it, which is just packed -1 afaict, dunno why
- otherwise each scan in range is testxxx.pcd and test_noisyxxx.pcd with frame as xxx
- NO IDEA where test suffix-less file test comes from - it comes from the appendEvdFile method, where the if test
works such that it writes PCL and evd, however it doesn't have the extension...

Info on EVD format is here:
https://www.blensor.org/file_format.html
"""
# get the args passed to blender after "--", all of which are ignored by
# blender so scripts may receive their own arguments
argv = sys.argv

if "--" not in argv:
    argv = []  # as if no args are passed
else:
    argv = argv[argv.index("--") + 1:]  # get all args after "--"

parser = argparse.ArgumentParser()
parser.add_argument(
    '-d',
    help = 'Directory with Room PLY files'
)

parser.add_argument(
    '-p',
    help = 'Directory with room TXT files for scanner positions'
)


args = parser.parse_args(argv)
rooms_dir = args.d
scanner_positions_dir = args.p


frame_start = 0
frame_end = 144
sweep_duration = 72
VLP16 = "vlp16"

def scan_room(room_filepath, scanner_positions_filepath):
    # Clear scene entirely
    bpy.ops.wm.open_mainfile(filepath="/home/branisj/thesis/empty.blend")
    scn = bpy.context.scene

    print("Loading room mesh.")
    obj_files = []

    # this was there for when the room is split into several different files
    #for root, dirs, files in os.walk("/home/branisj/3D-FRONT-ToolBox/scripts/output/Bedroom-6979.ply"):
    #    for name in files:
    #        if name.endswith(".ply") and not name.endswith("_full.ply"):
    #            bpy.ops.import_mesh.ply(filepath=os.path.join(root, name))

    bpy.ops.import_mesh.ply(filepath=room_filepath)

    # This was there for dealing with the colors
    # Commented until needed again, if ever
    #for obj in bpy.data.objects:
    #    color = obj.data.vertex_colors[0].data[0].color
    #    mat = bpy.data.materials.new(name=obj.name)
    #    mat.diffuse_color = (color[0], color[1], color[2])
    #    mat.specular_color = (color[0], color[1], color[2])
    #    mat.use_vertex_color_paint = True
    #    mat.use_shadeless = True
    #    obj.data.materials.append(mat)
    # room = bpy.context.scene.objects.active
    # mat.use_nodes=True
    # nodes=mat.node_tree.nodes
    # for node in nodes:
    #     nodes.remove(node)
    # output_node = nodes.new(type='ShaderNodeOutputMaterial')
    # background_node = nodes.new(type='ShaderNodeBackground')
    # attr_node = nodes.new(type='ShaderNodeAttribute')
    # attr_node.attribute_name = "Col"
    #
    # links = mat.node_tree.links
    # links.new(attr_node.outputs[0], background_node.inputs[0])
    # links.new(background_node.outputs[0], output_node.inputs[0])


    # Get locations from file
    print("Loading locations file")
    locations = []
    with open(scanner_positions_filepath) as loc_f:
        lines = loc_f.readlines()
        for line in lines:
            x = line.strip().split(" ")
            locations.append((float(x[0]), 1.0, float(x[2])))
    rotation_start = -30
    rotation_end = 30

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
    count = 0
    for loc in locations:
        # Set scanner to location
        scanner_obj.location = loc
        scanner_obj.keyframe_insert(data_path="location", frame=sweep_duration * count)

        scanner_obj.rotation_euler = (radians(rotation_start), 0, 0)
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * count)

        scanner_obj.rotation_euler = (radians(rotation_end), 0, 0)
        scanner_obj.keyframe_insert(data_path="rotation_euler", frame=sweep_duration * (count + 1) - 1)

        count = count + 1

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
    for i, loc in enumerate(locations):
        print("Scanning")
        # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
        output_filename = room_filepath.replace(".ply", ".evd")
        blensor.blendodyne.scan_range(scanner_obj,
                                      filename=output_filename,
                                      frame_start=sweep_duration * i,
                                      frame_end=sweep_duration * (i + 1) - 1,
                                      world_transformation=scanner_obj.matrix_world)


room_files = {}
for root, dirs, files in os.walk(rooms_dir, topdown=False):
    for name in files:
        if not name.endswith(".ply"):
            continue
        identifier = name.replace(".ply", "")
        room_files[identifier] = {
            "room_file": os.path.join(root, name)    
        }

for root, dirs, files in os.walk(scanner_positions_dir):
    for name in files:
        if not name.endswith(".txt"):
            continue
        identifier = name.replace(".txt", "")
        room_files[identifier]["positions_file"] = os.path.join(root, name)


for room_id in room_files:
    scan_room(room_files[room_id]["room_file"], room_files[room_id]["positions_file"])
