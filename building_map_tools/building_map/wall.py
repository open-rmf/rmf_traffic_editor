import math
import os
import shutil

import numpy as np

from xml.etree.ElementTree import SubElement
from ament_index_python.packages import get_package_share_directory

from .edge import Edge
from .param_value import ParamValue


class Wall:
    def __init__(self, yaml_node, texture_name=""):

        self.wall_height = 2.5  # meters
        self.wall_thickness = 0.1  # meters
        self.cap_thickness = 0.11  # meters
        self.cap_height = 0.02  # meters
        self.transformed_vertices = [] # check if use vertex indics?? todo
        self.walls = []
        
        # texture filtering
        # for backward compatability. if "texture_name" is not
        # specified in wall.params, assume default tex wall.png
        for wall in yaml_node:
            if "texture_name" in wall.params:
                if texture_name == wall.params["texture_name"].value:
                    self.walls.append(wall)
            else:
                if texture_name == "":
                    self.walls.append(wall)

        # todo: to clean "wall.png" texture which is confusing...
        if texture_name == "":
            self.texture_name = "wall" #default png
        else:
            self.texture_name = texture_name

    def __str__(self):
        return f'wall ({len(self.walls)} with tex {self.texture_name})'

    def __repr__(self):
        return self.__str__()

    # todo: not in used, remove?
    def generate_wall_box_geometry(self, wall, parent_ele, item):
        x1, y1 = self.transformed_vertices[wall.start_idx]
        x2, y2 = self.transformed_vertices[wall.end_idx]

        dx = x1 - x2
        dy = y1 - y2
        length = math.sqrt(dx*dx + dy*dy) + self.wall_thickness
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        wall_yaw = math.atan2(dy, dx)

        if item == 'wall':
            box_thickness = self.wall_thickness
            box_height = self.wall_height
            cz = self.wall_height / 2.0
        elif item == 'cap':
            box_thickness = self.cap_thickness
            box_height = self.cap_height
            cz = self.wall_height
        else:
            raise RuntimeError('requested unknown wall item type  :(')

        geometry_ele = SubElement(parent_ele, 'geometry')

        box_ele = SubElement(geometry_ele, 'box')

        size_ele = SubElement(box_ele, 'size')
        size_ele.text = f'{length} {box_thickness} {box_height}'

        pose_ele = SubElement(parent_ele, 'pose')
        pose_ele.text = f'{cx} {cy} {cz} 0 0 {wall_yaw}'

    def generate_wall_visual_mesh(self, model_name, model_path):
        print(f'generate_wall_visual_mesh({model_name}, {model_path})')

        meshes_path = f'{model_path}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_model_rel_path = f'meshes/wall_{self.texture_name}.obj'
        obj_path = os.path.join(model_path, obj_model_rel_path)
        print(f'  generating {obj_path}')
                
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib wall_{self.texture_name}.mtl\n')
            f.write(f'o walls\n')

            h = self.wall_height  # todo: allow per-segment height?

            # calculate faces for all the wall segments
            wall_verts = np.array([])
            texture_lengths = [0]

            # normal #1 is the vertical normal for wall "caps"
            norms = np.array([[0, 0, 1]])

            for wall in self.walls:

                wx1, wy1 = self.transformed_vertices[wall.start_idx].xy()
                wx2, wy2 = self.transformed_vertices[wall.end_idx].xy()

                wdx = wx2 - wx1
                wdy = wy2 - wy1
                wlen = math.sqrt(wdx*wdx + wdy*wdy)
                wcx = (wx1 + wx2) / 2.0
                wcy = (wy1 + wy2) / 2.0
                wyaw = math.atan2(wdy, wdx)

                # calculate the 4 corners of the wall footprint
                t2 = self.wall_thickness / 2.0

                wall_footprint_at_origin = np.array([
                    [-wlen / 2.0 - t2, t2],
                    [wlen / 2.0 + t2, t2],
                    [wlen / 2.0 + t2, -t2],
                    [-wlen / 2.0 - t2, -t2]])

                # now rotate the wall footprint
                rot = np.array([
                    [math.cos(wyaw), math.sin(wyaw)],
                    [-math.sin(wyaw), math.cos(wyaw)]])

                rot_verts = wall_footprint_at_origin.dot(rot)

                # finally, translate the wall segment vertices
                v = rot_verts + np.array([[wcx, wcy]])

                segment_norms_at_origin = np.array([
                    [0, 1],
                    [-1, 0],
                    [0, -1],
                    [1, 0]])
                segment_norms = segment_norms_at_origin.dot(rot)

                if not wall_verts.any():
                    wall_verts = v
                else:
                    wall_verts = np.vstack((wall_verts, v))

                # add z=0 to all segment norms
                norms = np.vstack((
                    norms,
                    np.hstack((segment_norms, np.zeros((4, 1))))
                ))

                # in the future we may have texture tiles of different scale,
                # but for now let's assume 1-meter x 1-meter tiles, so we don't
                # need to scale the texture coordinates currently.
                texture_lengths.append(wlen)

            for v in wall_verts:
                f.write(f'v {v[0]:.4f} {v[1]:.4f} 0.000\n')
                f.write(f'v {v[0]:.4f} {v[1]:.4f} {h:.4f}\n')

            for length in texture_lengths:
                f.write(f'vt {length:.4f} 0.000\n')
                f.write(f'vt {length:.4f} 1.000\n')

            for norm in norms:
                f.write(f'vn {norm[0]:.4f} {norm[1]:.4f} {norm[2]:.4f}\n')

            f.write(f'usemtl wall_{self.texture_name}\n')
            f.write('s off\n')
            f.write('g walls\n')

            # finally we can wind the actual 8 face triangles
            for w in range(0, len(self.walls)):
                # first the side facing 'north' before rotation
                f.write(
                    f'f {w*8+1}/1/{w*4+2}'
                    f' {w*8+2}/2/{w*4+2}'
                    f' {w*8+3}/1/{w*4+2}\n')
                f.write(
                    f'f {w*8+4}/2/{w*4+2}'
                    f' {w*8+3}/1/{w*4+2}'
                    f' {w*8+2}/2/{w*4+2}\n')

                # now the 'east' side
                f.write(
                    f'f {w*8+3}/1/{w*4+3}'
                    f' {w*8+4}/2/{w*4+3}'
                    f' {w*8+5}/1/{w*4+3}\n')
                f.write(
                    f'f {w*8+6}/2/{w*4+3}'
                    f' {w*8+5}/1/{w*4+3}'
                    f' {w*8+4}/2/{w*4+3}\n')

                # now the 'south' side
                f.write(
                    f'f {w*8+5}/1/{w*4+4}'
                    f' {w*8+6}/2/{w*4+4}'
                    f' {w*8+7}/1/{w*4+4}\n')
                f.write(
                    f'f {w*8+8}/2/{w*4+4}'
                    f' {w*8+7}/1/{w*4+4}'
                    f' {w*8+6}/2/{w*4+4}\n')

                # now the 'west' side
                f.write(
                    f'f {w*8+7}/1/{w*4+5}'
                    f' {w*8+8}/2/{w*4+5}'
                    f' {w*8+1}/1/{w*4+5}\n')
                f.write(
                    f'f {w*8+2}/2/{w*4+5}'
                    f' {w*8+1}/1/{w*4+5}'
                    f' {w*8+8}/2/{w*4+5}\n')

                # now the top "cap" of this wall segment
                f.write(f'f {w*8+2}/1/1 {w*8+6}/1/1 {w*8+4}/1/1\n')
                f.write(f'f {w*8+2}/1/1 {w*8+8}/1/1 {w*8+6}/1/1\n')

        mtl_path = f'{meshes_path}/wall_{self.texture_name}.mtl'
        print(f'  generating {mtl_path}')
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl wall_{self.texture_name}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd {self.texture_name}.png\n')

        print(f'  copying wall textures into {meshes_path}')
        texture_path_source = os.path.join(
            get_package_share_directory('building_map_tools'),
            f'textures/{self.texture_name}.png')
        texture_path_dest = f'{meshes_path}/{self.texture_name}.png'
        shutil.copyfile(texture_path_source, texture_path_dest)

    def generate(
        self,
        model_ele,
        model_name,
        model_path,
        transformed_vertices):
        
        print(f'generating wall tex: {self.texture_name} on floor: {model_name}')
        if not self.walls:
            return

        self.transformed_vertices = transformed_vertices

        link_ele = SubElement(model_ele, 'link', {'name': 'walls'})
        self.generate_wall_visual_mesh(model_name, model_path)

        obj_path = f'model://{model_name}/meshes/wall_{self.texture_name}.obj'

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', f'wall_{self.texture_name}')

        v_geom_ele = SubElement(visual_ele, 'geometry')

        v_mesh_ele = SubElement(v_geom_ele, 'mesh')
        v_mesh_uri_ele = SubElement(v_mesh_ele, 'uri')
        v_mesh_uri_ele.text = obj_path

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', 'collision')

        c_geom_ele = SubElement(collision_ele, 'geometry')
        c_mesh_ele = SubElement(c_geom_ele, 'mesh')
        c_mesh_uri_ele = SubElement(c_mesh_ele, 'uri')
        c_mesh_uri_ele.text = obj_path

        c_surface_ele = SubElement(collision_ele, 'surface')
        c_contact_ele = SubElement(c_surface_ele, 'contact')
        c_bitmask_ele = SubElement(c_contact_ele, 'collide_bitmask')
        c_bitmask_ele.text = '0x01'
