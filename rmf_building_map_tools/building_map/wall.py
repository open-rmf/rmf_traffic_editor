import math
import os

import numpy as np

from xml.etree.ElementTree import SubElement

from .edge import Edge
from .material_utils import add_pbr_material, copy_texture, get_pbr_textures
from .param_value import ParamValue


class Wall:
    def __init__(self, yaml_node, wall_params):

        self.wall_height = 2.5  # meters
        self.wall_thickness = 0.1  # meters
        self.transformed_vertices = []  # check if use vertex indics?? todo
        self.walls = []
        self.wall_cnt = 0
        self.texture_name = wall_params['texture_name']
        self.alpha = wall_params['alpha']  # val 0.0-1.0 transparency of wall
        self.pbr_textures = get_pbr_textures(wall_params)
        if 'texture_height' in wall_params:  # check for new parameters
            self.texture_height = wall_params['texture_height'].value
            self.texture_width = wall_params['texture_width'].value
            self.texture_scale = wall_params['texture_scale'].value
        else:
            self.texture_height = self.wall_height
            self.texture_width = 1
            self.texture_scale = 1

        # Wall filtering according to wall_params
        for checked_wall in yaml_node:
            if wall_params == checked_wall.params:
                self.walls.append(checked_wall)

    def __str__(self):
        return f'wall {self.wall_cnt} with param {self.texture_name}, \
            {self.alpha}'

    def __repr__(self):
        return self.__str__()

    def generate_wall_visual_mesh(self, model_name, model_path,
                                  texture_filename):
        print(f'generate_wall_visual_mesh({model_name}, {model_path})')

        meshes_path = f'{model_path}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_model_rel_path = f'meshes/wall_{self.wall_cnt}.obj'
        obj_path = os.path.join(model_path, obj_model_rel_path)
        print(f'  generating {obj_path}')

        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib wall_{self.wall_cnt}.mtl\n')
            f.write(f'o walls\n')

            h = self.wall_height  # todo: allow per-segment height?

            # calculate faces for all the wall segments
            wall_verts = np.array([])
            texture_lengths = []

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
                # but for now we assume 2.5-meter x 1-meter tiles, so we don't
                # need to scale the texture coordinates currently.
                texture_lengths.append(wlen)

            """
            Instead of having potentially-huge wall vertex values,
            let's instead translate them all towards the origin
            and then push the entire wall OBJ to its final location
            so that Gazebo/Ignition "Move To" command is useful.
            """

            self.wall_mesh_offsets = np.min(wall_verts, axis=0)
            wall_verts = wall_verts - self.wall_mesh_offsets

            for v in wall_verts:
                f.write(f'v {v[0]:.4f} {v[1]:.4f} 0.000\n')
                f.write(f'v {v[0]:.4f} {v[1]:.4f} {h:.4f}\n')

            vt_h = h/(self.texture_height/self.texture_width)
            s = self.texture_scale
            if s == 0:  # full wall (by height)
                s = vt_h

            t = self.wall_thickness
            for length in texture_lengths:
                f.write(f'vt 0.000 0.000\n')
                f.write(f'vt 0.000 {(vt_h/s):.4f}\n')
                f.write(f'vt {(length/s):.4f} 0.000\n')
                f.write(f'vt {(length/s):.4f} {(vt_h/s):.4f}\n')
                f.write(f'vt {((length + t)/s):.4f} 0.000\n')
                f.write(f'vt {((length + t)/s):.4f} {(vt_h/s):.4f}\n')
                f.write(f'vt {((2*length + t)/s):.4f} 0.000\n')
                f.write(f'vt {((2*length + t)/s):.4f} {(vt_h/s):.4f}\n')

            for norm in norms:
                f.write(f'vn {norm[0]:.4f} {norm[1]:.4f} {norm[2]:.4f}\n')

            f.write(f'usemtl wall_{self.wall_cnt}\n')
            f.write('s off\n')
            f.write('g walls\n')

            # finally we can wind the actual 8 face triangles
            for w in range(0, len(self.walls)):
                # first the side facing 'north' before rotation
                f.write(
                    f'f {w*8+1}/{w*8+1}/{w*4+2}'
                    f' {w*8+2}/{w*8+2}/{w*4+2}'
                    f' {w*8+3}/{w*8+3}/{w*4+2}\n')
                f.write(
                    f'f {w*8+4}/{w*8+4}/{w*4+2}'
                    f' {w*8+3}/{w*8+3}/{w*4+2}'
                    f' {w*8+2}/{w*8+2}/{w*4+2}\n')

                # now the 'east' side
                f.write(
                    f'f {w*8+3}/{w*8+3}/{w*4+3}'
                    f' {w*8+4}/{w*8+4}/{w*4+3}'
                    f' {w*8+5}/{w*8+5}/{w*4+3}\n')
                f.write(
                    f'f {w*8+6}/{w*8+6}/{w*4+3}'
                    f' {w*8+5}/{w*8+5}/{w*4+3}'
                    f' {w*8+4}/{w*8+4}/{w*4+3}\n')

                # now the 'south' side
                f.write(
                    f'f {w*8+5}/{w*8+5}/{w*4+4}'
                    f' {w*8+6}/{w*8+6}/{w*4+4}'
                    f' {w*8+7}/{w*8+7}/{w*4+4}\n')
                f.write(
                    f'f {w*8+8}/{w*8+8}/{w*4+4}'
                    f' {w*8+7}/{w*8+7}/{w*4+4}'
                    f' {w*8+6}/{w*8+6}/{w*4+4}\n')

                # now the 'west' side
                f.write(
                    f'f {w*8+7}/{w*8+7}/{w*4+5}'
                    f' {w*8+8}/{w*8+8}/{w*4+5}'
                    f' {w*8+1}/{w*8+1}/{w*4+5}\n')
                f.write(
                    f'f {w*8+2}/{w*8+2}/{w*4+5}'
                    f' {w*8+1}/{w*8+1}/{w*4+5}'
                    f' {w*8+8}/{w*8+8}/{w*4+5}\n')

                # now the top "cap" of this wall segment
                f.write(f'f {w*8+2}/1/1 {w*8+6}/1/1 {w*8+4}/1/1\n')
                f.write(f'f {w*8+2}/1/1 {w*8+8}/1/1 {w*8+6}/1/1\n')

        mtl_path = f'{meshes_path}/wall_{self.wall_cnt}.mtl'
        print(f'  generating {mtl_path}')
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl wall_{self.wall_cnt}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write(f'd {self.alpha.value}\n')  # alpha
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd {texture_filename}\n')

    def generate(
        self,
        model_ele,
        wall_cnt,
        model_name,
        model_path,
        transformed_vertices
    ):
        self.wall_cnt = wall_cnt
        print(f'generating wall: {self.wall_cnt} on floor: {model_name}')
        if not self.walls:
            return

        self.transformed_vertices = transformed_vertices

        link_ele = SubElement(
            model_ele, 'link', {'name': f'wall_{self.wall_cnt}'})

        obj_path = f'model://{model_name}/meshes/wall_{self.wall_cnt}.obj'

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', f'wall_{self.wall_cnt}')

        # Note: Material visual profile in the sdf will override the profile
        # in .mtl in the obj file. TODO will need to soon allign both profiles
        v_geom_ele = SubElement(visual_ele, 'geometry')
        v_transparency_ele = SubElement(visual_ele, 'transparency')
        v_transparency_ele.text = str(1.0 - self.alpha.value)

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

        texture_filename = add_pbr_material(
                visual_ele, model_name, f'wall_{wall_cnt}',
                self.texture_name.value, f'{model_path}/meshes',
                self.pbr_textures)

        self.generate_wall_visual_mesh(model_name, model_path,
                                       texture_filename)

        pose_ele = SubElement(link_ele, 'pose')
        link_x = self.wall_mesh_offsets[0]
        link_y = self.wall_mesh_offsets[1]
        pose_ele.text = f'{link_x} {link_y} 0 0 0 0'
