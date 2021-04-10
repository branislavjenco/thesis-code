import json
import trimesh
import numpy as np
import math
import os
import argparse
import math
import igl
from random import randint
from shutil import copyfile

def get_random_rgb():
    return (randint(0, 255), randint(0, 255), randint(0, 255))

RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
CEILING_COLOR = RED
FLOOR_COLOR = GREEN
WALL_COLOR = BLUE
IRRELEVANT_COLOR = BLACK

color_map = {
    "Baseboard": IRRELEVANT_COLOR,
    "Cabinet": IRRELEVANT_COLOR,
    "Ceiling": CEILING_COLOR,
    "CustomizedCeiling": IRRELEVANT_COLOR, # this is the various moulding around the ceiling
    "CustomizedFeatureWall": WALL_COLOR, # sometimes the inner walls are this instead of WallInner
    "Door": IRRELEVANT_COLOR,
    "Floor": FLOOR_COLOR,
    "Hole": IRRELEVANT_COLOR,
    "Pocket": IRRELEVANT_COLOR,
    "WallBottom": IRRELEVANT_COLOR,
    "WallInner": WALL_COLOR,
    "WallOuter": IRRELEVANT_COLOR,
    "WallTop": IRRELEVANT_COLOR,
    "Window": IRRELEVANT_COLOR,
    "LightBand": IRRELEVANT_COLOR,
    "SlabSide": IRRELEVANT_COLOR
}

def split_path(paths):
    filepath,tempfilename = os.path.split(paths)
    filename,extension = os.path.splitext(tempfilename)
    return filepath,filename,extension


def write_obj_with_tex(savepath, vert, face, vtex, ftcoor, imgpath=None):
    filepath2,filename,extension = split_path(savepath)
    with open(savepath,'w') as fid:
        fid.write('mtllib '+filename+'.mtl\n')
        fid.write('usemtl a\n')
        for v in vert:
            fid.write('v %f %f %f\n' % (v[0],v[1],v[2]))
        for vt in vtex:
            fid.write('vt %f %f\n' % (vt[0],vt[1]))
        face = face + 1
        ftcoor = ftcoor + 1
        for f,ft in zip(face,ftcoor):
            fid.write('f %d/%d %d/%d %d/%d\n' % (f[0],ft[0],f[1],ft[1],f[2],ft[2]))
    filepath, filename2, extension = split_path(imgpath)
    if os.path.exists(imgpath) and not os.path.exists(filepath2+'/'+filename+extension):
        copyfile(imgpath, filepath2+'/'+filename+extension)
    if imgpath is not None:
        with open(filepath2+'/'+filename+'.mtl','w') as fid:
            fid.write('newmtl a\n')
            fid.write('map_Kd '+filename+extension)

def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])



parser = argparse.ArgumentParser()
parser.add_argument(
        '--future_path',
        default = './3D-FUTURE-model',
        help = 'path to 3D FUTURE'
        )
parser.add_argument(
        '--json_path',
        default = './3D-FRONT',
        help = 'path to 3D FRONT'
        )

parser.add_argument(
        '--save_path',
        default = './outputs',
        help = 'path to save result dir'
        )

args = parser.parse_args()

files = os.listdir(args.json_path)

if not os.path.exists(args.save_path):
    os.mkdir(args.save_path)

for m in files:
    with open(args.json_path+'/'+m, 'r', encoding='utf-8') as f:
        data = json.load(f)
        model_jid = []
        model_uid = []
        model_bbox= []

        mesh_uid = []
        mesh_xyz = []
        mesh_faces = []
        mesh_types = []
        if not os.path.exists(args.save_path+'/'+m[:-5]):
            os.mkdir(args.save_path+'/'+m[:-5])
        print(m[:-5])
        for ff in data['furniture']:
            if 'valid' in ff and ff['valid']:
                model_uid.append(ff['uid'])
                model_jid.append(ff['jid'])
                model_bbox.append(ff['bbox'])
        for mm in data['mesh']:
            mesh_uid.append(mm['uid'])
            mesh_xyz.append(np.reshape(mm['xyz'], [-1, 3]))
            mesh_faces.append(np.reshape(mm['faces'], [-1, 3]))
            mesh_types.append(mm['type'])
        scene = data['scene']
        room = scene['room']
        for r in room:
            room_id = r['instanceid']
            meshes=[]
            if not os.path.exists(args.save_path+'/' + m[:-5]+'/'+room_id):
                os.mkdir(args.save_path+'/' + m[:-5] + '/' + room_id)
            children = r['children']
            number = 1
            mesh_t = None
            count = 0
            for c in children:

                ref = c['ref']
                type = 'f'
                try:
                    idx = model_uid.index(ref)
                    if os.path.exists(args.future_path+'/' + model_jid[idx]):
                        v, vt, _, faces, ftc, _ = igl.read_obj(args.future_path+'/' + model_jid[idx] + '/raw_model.obj')
                        # bbox = np.max(v, axis=0) - np.min(v, axis=0)
                        # s = bbox / model_bbox[idx]
                        # v = v / s
                except:
                    try:
                        idx = mesh_uid.index(ref)
                    except:
                        continue
                    v = mesh_xyz[idx]
                    faces = mesh_faces[idx]
                    type='m'
                    mesh_t = mesh_types[idx]

                pos = c['pos']
                rot = c['rot']
                scale = c['scale']
                v = v.astype(np.float64) * scale
                ref = [0,0,1]
                axis = np.cross(ref, rot[1:])
                theta = np.arccos(np.dot(ref, rot[1:]))*2
                if np.sum(axis) != 0 and not math.isnan(theta):
                    R = rotation_matrix(axis, theta)
                    v = np.transpose(v)
                    v = np.matmul(R, v)
                    v = np.transpose(v)

                v = v + pos
                if type == 'f':
                    # furniture
                    #write_obj_with_tex(args.save_path+'/' + m[:-5]+'/'+room_id+'/' + str(number) + '_' +model_jid[idx] + '.obj', v, faces, vt, ftc, args.future_path+'/' + model_jid[idx] + '/texture.png')
                    number = number + 1
                else:
                    face_color = [color_map[mesh_t] for _ in faces]
                    mesh = trimesh.Trimesh(v, faces)
                    vertex_colors = trimesh.visual.color.face_to_vertex_color(mesh, face_color)
                    vis = trimesh.visual.color.ColorVisuals(mesh, vertex_colors=vertex_colors)
                    mesh.visual = vis
                    meshes.append(mesh)
                    mesh.export(args.save_path+'/'+ m[:-5] + '/' + room_id + '/' + str(count) + '.ply')
                count += 1

            if len(meshes) > 0:
                temp = trimesh.util.concatenate(meshes)
                temp.export(args.save_path+'/'+ m[:-5] + '/' + room_id + '/' + room_id + '_full.ply')

