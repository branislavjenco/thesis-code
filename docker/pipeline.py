import trimesh
import pymesh
import os
import numpy as np

#TODO make the input folder configurable
def remove_duplicate_vertices(vertices, rtol=1e-02):
    dedup_verts = []
    duplicates = {}
    for i, v1 in enumerate(vertices):
        for j, v2 in enumerate(vertices):
            if np.allclose(v1, v2, rtol=rtol) and i != j:
                duplicates[i] = j
    for i, v in enumerate(vertices):
        if i not in duplicates.keys() and i not in duplicates.values():
            dedup_verts.append(v)
    return dedup_verts


def merge_close_vertices(vertices, distance=0.5):
    dedup_verts = []
    duplicates = {}
    for i, v1 in enumerate(vertices):
        for j, v2 in enumerate(vertices):
            if np.linalg.norm(v1 - v2) < distance and i != j:
                duplicates[i] = j
    for i, v in enumerate(vertices):
        if i in duplicates.keys():
            j = duplicates[i]
            new_vertex = [(vertices[i][0] + vertices[j][0]) / 2.0, (vertices[i][1] + vertices[j][1]) / 2.0]
            del duplicates[j]
            dedup_verts.append(np.array(new_vertex))
        elif i not in duplicates.values():
            dedup_verts.append(v)
    return dedup_verts

# Find the necessary files
obj_files = []
for root, dirs, files in os.walk("./input"):
    for name in sorted(files):
        if name.endswith("_full.ply"):
            obj_files.append((name, os.path.join(root, name)))


for name, filepath in obj_files:
    with open(filepath, "rb") as f:
        room = trimesh.load_mesh(f, file_type="ply")
    if room is None:
        continue

    print(name, filepath)

    # Extract floor
    bottom_mesh = room.slice_plane((0, 1.5, 0), (0, -1, 0))
    upward_faces = [np.isclose(f[0], 0.0) and np.isclose(f[1], 1.0) and np.isclose(f[2], 0.0) for f in bottom_mesh.face_normals]
    bottom_mesh.update_faces(upward_faces)
    try:
        bottom_mesh = bottom_mesh.process(validate=True)
    except BaseException:
        print(f"Couldn't process mesh {name} in {filepath}, moving on to the next one.")
        continue
    bottom_mesh.fill_holes()
    bottom_mesh.remove_duplicate_faces()
    bottom_mesh.remove_infinite_values()
    bottom_mesh.merge_vertices()

    # project
    new_vertices = []
    for vt in bottom_mesh.vertices:
        new_vertices.append([vt[0], 0., vt[2]])
    new_vertices = np.array(new_vertices)
    bottom_mesh_flattened = trimesh.Trimesh(new_vertices, bottom_mesh.faces)

    # TODO: figure out how to convert trimesh to pymesh (or get rid of trimesh)
    bottom_mesh_flattened.export(f'./output/tmp.ply')
    p_bottom_mesh = pymesh.load_mesh(f'./output/tmp.ply')
    p_bottom_mesh = pymesh.resolve_self_intersection(p_bottom_mesh)
    pymesh.save_mesh(f"./output/{os.path.splitext(name)[0]}.ply", p_bottom_mesh)

    with open(f"./output/{os.path.splitext(name)[0]}.ply", "rb") as f:
        cleaned_floor = trimesh.load_mesh(f, file_type="ply")

    # Process the model
    cleaned_floor = cleaned_floor.process(validate=True)
    cleaned_floor = cleaned_floor.simplify_quadratic_decimation(10)
    outline = cleaned_floor.outline()
    plan, transformation = outline.to_planar()
    plan = plan.process()

    # Get medial axis
    axis = plan.medial_axis()
    axis3d = axis.to_3D(transformation)

    # Get vertices from medial axis
    verts = []
    for e in axis3d.entities:
        to_add = []
        for p in e.end_points:
            to_add.append(axis3d.vertices[p])
        verts.extend(to_add)

    dedup_verts = remove_duplicate_vertices(verts, rtol=1e-02)

    # Get only vertices which are in the middle of the room
    middle_verts = []
    for v in dedup_verts:
        middle = True
        for vp in outline.vertices:
            if np.allclose(v, vp, atol=0.2):
                middle = False
                break
        if middle:
            middle_verts.append(v)

    print(middle_verts)
    middle_verts = merge_close_vertices(middle_verts, distance=0.2)

    # middle verts now are the points where to put the laser
    np.savetxt(f"./output/{os.path.splitext(name)[0]}.txt", np.array(middle_verts))


