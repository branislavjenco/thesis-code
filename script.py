import bpy
import os
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
frame_start = 0
frame_end = 72
sweep_duration = 72
VLP16 = "vlp16"

# Clear scene entirely
bpy.ops.wm.open_mainfile(filepath="/home/brano/Projects/thesis/empty.blend")
scn = bpy.context.scene

print("Loading room mesh.")
obj_files = []
for root, dirs, files in os.walk("/home/brano/Projects/thesis/outputs/5ca4c392-dcc6-4dc1-a607-44b66785ac6d/LivingDiningRoom-48804"):
    for name in files:
        if name.endswith(".ply") and not name.endswith("_full.ply"):
            bpy.ops.import_mesh.ply(filepath=os.path.join(root, name))


for obj in bpy.data.objects:
    color = obj.data.vertex_colors[0].data[0].color
    mat = bpy.data.materials.new(name=obj.name)
    mat.diffuse_color = (color[0], color[1], color[2])
    mat.specular_color = (color[0], color[1], color[2])
    mat.use_vertex_color_paint = True
    mat.use_shadeless = True
    obj.data.materials.append(mat)
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
with open("/home/brano/Projects/thesis/docker/output/LivingDiningRoom-48804_full.txt") as loc_f:
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
for i, loc in enumerate(locations):
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
for i, loc in enumerate(locations):
    print("Scanning")
    # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
    blensor.blendodyne.scan_range(scanner_obj,
                                  filename=f"/home/brano/Projects/thesis/scans/test_{i}.evd",
                                  frame_start=sweep_duration * i,
                                  frame_end=sweep_duration * (i + 1) - 1,
                                  world_transformation=scanner_obj.matrix_world)

