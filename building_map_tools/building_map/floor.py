import math
import os
import shutil

from tripy import tripy
from xml.etree.ElementTree import SubElement
from ament_index_python.packages import get_package_share_directory


class Floor:
    def __init__(self, yaml_node, level_vertices):
        self.vertices = []
        self.thickness = 0.1
        for v_idx in yaml_node['vertices']:
            v = level_vertices[v_idx]
            self.vertices.append([v.x, v.y])

    def __str__(self):
        return f'floor ({len(self.vertices)} vertices)'

    def __repr__(self):
        return self.__str__()

    def generate_geometry(self, parent_ele):
        pose_ele = SubElement(parent_ele, 'pose')
        pose_ele.text = f'0 0 {-self.thickness} 0 0 0'

        geometry_ele = SubElement(parent_ele, 'geometry')

        polyline_ele = SubElement(geometry_ele, 'polyline')

        height_ele = SubElement(polyline_ele, 'height')
        height_ele.text = f'{self.thickness}'

        for vertex in self.vertices:
            point_ele = SubElement(polyline_ele, 'point')
            point_ele.text = f'{vertex[0]} {vertex[1]}'

    def find_vertex_idx(self, x, y):
        for v_idx, v in enumerate(self.vertices):
            dx = x - v[0]
            dy = y - v[1]
            d = math.sqrt(dx*dx + dy*dy)
            if d < 0.0001:
                return v_idx
        raise RuntimeError("Couldn't find vertex index!")

    def generate(self, model_ele, floor_cnt, model_name, model_path):
        print(f'generating floor polygon {floor_cnt} on floor')
        # for v in self.vertices:
        #     print(f'  {v[0]} {v[1]}')

        link_ele = SubElement(model_ele, 'link')
        link_ele.set('name', f'floor_{floor_cnt}')

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', 'visual')

        visual_geometry_ele = SubElement(visual_ele, 'geometry')

        mesh_ele = SubElement(visual_geometry_ele, 'mesh')
        mesh_uri_ele = SubElement(mesh_ele, 'uri')
        meshes_path = f'{model_path}/meshes'
        obj_model_rel_path = f'meshes/floor_{floor_cnt}.obj'
        mesh_uri_ele.text = f'model://{model_name}/{obj_model_rel_path}'

        '''
        material_ele = SubElement(visual_ele, 'material')
        material_script_ele = SubElement(material_ele, 'script')
        material_script_name_ele = SubElement(material_script_ele, 'name')
        material_script_name_ele.text = 'SossSimulation/SimpleFloor'

        self.generate_geometry(visual_ele)
        '''

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', 'collision')

        surface_ele = SubElement(collision_ele, 'surface')
        contact_ele = SubElement(surface_ele, 'contact')
        collide_bitmask_ele = SubElement(contact_ele, 'collide_bitmask')
        collide_bitmask_ele.text = '0x01'

        self.generate_geometry(collision_ele)

        triangles = tripy.earclip(self.vertices)

        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_path = f'{model_path}/{obj_model_rel_path}'
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib floor_{floor_cnt}.mtl\n')
            f.write(f'o floor_{floor_cnt}\n')

            # this assumes that the vertices are in "correct" (OBJ) winding
            # ordering already. todo: detect if the winding order is
            # inverted and re-wind appropriately
            for v in self.vertices:
                f.write(f'v {v[0]} {v[1]} 0\n')

            # in the future we may have texture tiles of a different size,
            # but for now let's assume 1-meter x 1-meter tiles, so we don't
            # need to scale the texture coordinates currently.
            for v in self.vertices:
                f.write(f'vt {v[0]} {v[1]} 0\n')

            # our floors are always flat (for now), so we only have one normal
            f.write(f'vn 0 0 1\n')

            f.write(f'usemtl floor_{floor_cnt}\n')
            f.write('s off\n')

            for triangle in triangles:
                # todo... clean this up. For now, wind the triangles both ways

                f.write('f')
                for tri_vertex in triangle:
                   v_idx = self.find_vertex_idx(tri_vertex[0], tri_vertex[1])
                   f.write(f' {v_idx+1}/{v_idx+1}/1')
                f.write('\n')

                f.write('f')
                for tri_vertex in reversed(triangle):
                   v_idx = self.find_vertex_idx(tri_vertex[0], tri_vertex[1])
                   f.write(f' {v_idx+1}/{v_idx+1}/1')
                f.write('\n')

        print(f'  wrote {obj_path}')

        mtl_path = f'{model_path}/meshes/floor_{floor_cnt}.mtl'
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl floor_{floor_cnt}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd floor_{floor_cnt}.png\n')

        print(f'  wrote {mtl_path}')

        # todo: read texture parameter somehow from YAML
        # for now, just use blue linoleum
        # todo: use ament_resource_index somehow to calculate this path
        texture_path_source = os.path.join(
            get_package_share_directory('building_map_tools'),
            'textures/blue_linoleum_high_contrast.png')
        texture_path_dest = f'{model_path}/meshes/floor_{floor_cnt}.png'
        shutil.copyfile(texture_path_source, texture_path_dest)
        print(f'  wrote {texture_path_dest}')
