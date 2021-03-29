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
VLP16 = "vlp16"

# Clear scene entirely
bpy.ops.wm.open_mainfile(filepath="/home/brano/Projects/3D-FRONT-ToolBox/empty.blend")
scn = bpy.context.scene

print("Loading room mesh.")
obj_files = []
for root, dirs, files in os.walk("/home/brano/Projects/3D-FRONT-ToolBox/scripts/outputs/5ca4c392-dcc6-4dc1-a607-44b66785ac6d/LivingDiningRoom-48804"):
    for name in files:
        if name.endswith(".ply"):
            bpy.ops.import_mesh.ply(filepath=os.path.join(root, name))


for obj in bpy.data.objects:
    color = obj.data.vertex_colors[0].data[0].color
    mat = bpy.data.materials.new(name=obj.name)
    mat.diffuse_color = (color[0], color[1], color[2])
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
with open("/home/brano/Projects/3D-FRONT-ToolBox/docker2/output/LivingDiningRoom-48804.txt") as loc_f:
    lines = loc_f.readlines()
    for line in lines:
        x = line.strip().split(" ")
        locations.append((float(x[0]), 1.0, float(x[2])))
rotation_start = 60
rotation_end = 120

# Add camera (lidar)
print("Adding scanner to scene")
scanner_data = bpy.data.cameras.new(VLP16)
scanner_obj = bpy.data.objects.new(VLP16, scanner_data)
scanner_obj.scan_type = "velodyne"
scanner_obj.velodyne_model = VLP16
scanner_obj.ref_dist = scanner_obj.velodyne_ref_dist
scanner_obj.ref_limit = scanner_obj.velodyne_ref_limit
scanner_obj.ref_slope = scanner_obj.velodyne_ref_slope
scanner_obj.local_coordinates = True

scn.objects.link(scanner_obj)
scn.camera = scanner_obj
bpy.context.scene.objects.active = scanner_obj

for i, loc in enumerate(locations):
    # Set scanner to location
    scanner_obj.location = loc
    print(f"Scanner at location: {loc}")

    blensor.evd.output_labels = True
    blensor.evd.frame_counter = frame_start

    scanner_obj.rotation_euler = (radians(rotation_start), 0, 0)
    scanner_obj.keyframe_insert(data_path="rotation_euler", frame=frame_start)

    scanner_obj.rotation_euler = (radians(rotation_end), 0, 0)
    scanner_obj.keyframe_insert(data_path="rotation_euler", frame=frame_end)

    print("Scanning")
    # The world transformation might need to be a parameter in the future (for connecting to our existing code for getting the transformation in real life)
    blensor.blendodyne.scan_range(scanner_obj,
                                  filename=f"/home/brano/Projects/3D-FRONT-ToolBox/synth_scans/test_{i}.evd",
                                  frame_start=frame_start,
                                  frame_end=frame_end,
                                  world_transformation=scanner_obj.matrix_world)

