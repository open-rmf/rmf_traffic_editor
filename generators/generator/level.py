import math
import os
import shutil

from xml.etree.ElementTree import ElementTree, Element, SubElement
from .etree_utils import indent_etree

from .edge import Edge
from .floor import Floor
from .model import Model
from .vertex import Vertex


class Level:
    def __init__(self, yaml_node, name):
        self.name = name
        self.drawing_name = yaml_node['drawing']['filename']
        self.wall_height = 2.5  # meters
        self.wall_thickness = 0.1  # meters
        self.cap_thickness = 0.11  # meters
        self.cap_height = 0.02  # meters

        self.vertices = []
        for vertex_yaml in yaml_node['vertices']:
            self.vertices.append(Vertex(vertex_yaml))

        self.meas = self.parse_edge_sequence(yaml_node['measurements'])

        # use the measurements to estimate scale for this level
        scale_cnt = 0
        scale_sum = 0
        for m in self.meas:
            scale_cnt += 1
            scale_sum += m.params['distance'].value / m.length
        self.scale = scale_sum / float(scale_cnt)
        print(f'level {self.name} scale estimated as {self.scale}')

        # scale the vertex list
        for p in self.vertices:
            p.x *= self.scale
            p.y *= self.scale

        if 'lanes' in yaml_node:
          self.lanes = self.parse_edge_sequence(yaml_node['lanes'])

        if 'walls' in yaml_node:
          self.walls = self.parse_edge_sequence(yaml_node['walls'])

        self.models = []
        if 'models' in yaml_node:
            for model_yaml in yaml_node['models']:
                self.models.append(Model(model_yaml, self.scale))

        self.floors = []
        for floor_yaml in yaml_node['floors']:
            self.floors.append(Floor(floor_yaml, self.vertices))

    def parse_edge_sequence(self, sequence_yaml):
        edges = []
        for edge in sequence_yaml:
            edges.append(Edge(edge, self.vertices))
        return edges

    def generate_wall_box_geometry(self, wall, parent_ele, item):
        x1 = self.vertices[wall.start_idx].x
        y1 = self.vertices[wall.start_idx].y
        x2 = self.vertices[wall.end_idx].x
        y2 = self.vertices[wall.end_idx].y
        dx = x1 - x2
        dy = y1 - y2
        length = math.sqrt(dx*dx + dy*dy)
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

    def generate_wall(self, wall, link_ele, wall_cnt):
        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', f'walls_{wall_cnt}')
        self.generate_wall_box_geometry(wall, visual_ele, 'wall')
        material_ele = SubElement(visual_ele, 'material')
        script_ele = SubElement(material_ele, 'script')
        name_ele = SubElement(script_ele, 'name')
        name_ele.text = 'SossSimulation/SimpleWall'

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', f'walls_{wall_cnt}')
        self.generate_wall_box_geometry(wall, collision_ele, 'wall')
        surface_ele = SubElement(collision_ele, 'surface')
        contact_ele = SubElement(surface_ele, 'contact')
        bitmask_ele = SubElement(contact_ele, 'collide_bitmask')
        bitmask_ele.text = '0x01'

        cap_visual_ele = SubElement(link_ele, 'visual')
        cap_visual_ele.set('name', f'cap_{wall_cnt}')
        self.generate_wall_box_geometry(wall, cap_visual_ele, 'cap')
        cap_material_ele = SubElement(cap_visual_ele, 'material')
        cap_script_ele = SubElement(cap_material_ele, 'script')
        cap_script_name_ele = SubElement(cap_script_ele, 'name')
        cap_script_name_ele.text = 'Gazebo/Black'

    def generate_wall_visual_mesh(self, model_name, model_path):
        print(f'generate_wall_visual_mesh({model_name}, {model_path})')
        meshes_path = f'{model_path}/meshes'

        obj_model_rel_path = 'meshes/walls.obj'
        obj_path = os.path.join(model_path, obj_model_rel_path)
        print(f'  generating {obj_path}')
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib wall.mtl\n')
            f.write(f'o walls\n')

            # calculate faces for all the wall segments
            faces = []
            wall_cnt = 0
            for wall in self.walls:
                wall_cnt += 1
                #self.generate_wall(wall, link_ele, wall_cnt)

                wx1 = self.vertices[wall.start_idx].x
                wy1 = self.vertices[wall.start_idx].y
                wx2 = self.vertices[wall.end_idx].x
                wy2 = self.vertices[wall.end_idx].y
                wdx = wx1 - wx2
                wdy = wy1 - wy2
                wlen = math.sqrt(wdx*wdx + wdy*wdy)
                wcx = (wx1 + wx2) / 2.0
                wcy = (wy1 + wy2) / 2.0
                wyaw = math.atan2(wdy, wdx)

                box_thickness = self.wall_thickness
                box_height = self.wall_height
                cz = self.wall_height / 2.0

            '''
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
            '''



        mtl_path = f'{meshes_path}/wall.mtl'
        print(f'  generating {mtl_path}')
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl wall\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd wall.png\n')

        print(f'  copying wall textures into {meshes_path}')
        texture_path_source = 'textures/wall.png'
        texture_path_dest = f'{meshes_path}/wall.png'
        shutil.copyfile(texture_path_source, texture_path_dest)

    def generate_walls(self, model_ele, model_name, model_path):
        link_ele = SubElement(model_ele, 'link', {'name': 'walls'})
        self.generate_wall_visual_mesh(model_name, model_path)

        wall_cnt = 0
        for wall in self.walls:
            wall_cnt += 1
            self.generate_wall(wall, link_ele, wall_cnt)

    def generate_sdf_models(self, world_ele):
        model_cnt = 0
        for model in self.models:
            model_cnt += 1
            model.generate(world_ele, model_cnt)

    def generate_floors(self, world_ele, model_name, model_path):
        floor_cnt = 0
        for floor in self.floors:
            floor_cnt += 1
            floor.generate(world_ele, floor_cnt, model_name, model_path)
 
    def write_sdf(self, model_name, model_path):
        sdf_ele = Element('sdf', {'version': '1.6'})
        
        model_ele = SubElement(sdf_ele, 'model', {'name': model_name})

        static_ele = SubElement(model_ele, 'static')
        static_ele.text = 'true'

        self.generate_floors(model_ele, model_name, model_path)
        self.generate_walls(model_ele, model_name, model_path)

        sdf_tree = ElementTree(sdf_ele)
        indent_etree(sdf_ele)
        sdf_path = os.path.join(model_path, 'model.sdf')
        sdf_tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
        print(f'  wrote {sdf_path}')

    def generate_sdf_model(self, model_name, model_path):
        print(f'generating model of level {self.name} in {model_path}')
        config_fn = os.path.join(model_path, 'model.config')
        self.write_config(model_name, config_fn)
        print(f'  wrote {config_fn}')

        self.write_sdf(model_name, model_path)

    def write_config(self, model_name, path):
        config_ele = Element('model')

        name_ele = SubElement(config_ele, 'name')
        name_ele.text = model_name

        version_ele = SubElement(config_ele, 'version')
        version_ele.text = '1.0.0'

        sdf_ele = SubElement(config_ele, 'sdf', {'version': '1.6'})
        sdf_ele.text = 'model.sdf'

        author_ele = SubElement(config_ele, 'author')
        author_name_ele = SubElement(author_ele, 'name')
        author_name_ele.text = 'automatically generated from the Great Editor'
        author_email_ele = SubElement(author_ele, 'email')
        author_email_ele.text = 'info@openrobotics.org'

        description_ele = SubElement(config_ele, 'description')
        description_ele.text = f'level {model_name} (automatically generated)'

        config_tree = ElementTree(config_ele)
        indent_etree(config_ele)
        config_tree.write(path, encoding='utf-8', xml_declaration=True)

    def generate_nav_graph(self, graph_idx):
        """ Generate a graph without unnecessary (non-lane) vertices """
        # first remap the vertices. Store both directions; we'll need them
        next_idx = 0
        vidx_to_mapped_idx = {}
        mapped_idx_to_vidx = {}
        for l in self.lanes:
            if l.params['graph_idx'].value != graph_idx:
                continue
            if l.start_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[l.start_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = l.start_idx
                next_idx += 1
            if l.end_idx not in vidx_to_mapped_idx:
                vidx_to_mapped_idx[l.end_idx] = next_idx
                mapped_idx_to_vidx[next_idx] = l.end_idx
                next_idx += 1
        # print(vidx_to_mapped_idx)
        # print(mapped_idx_to_vidx)

        # now output the mapped vertices (in order)
        nav_data = {}
        nav_data['vertices'] = []
        for i in range(0, next_idx):
            v = self.vertices[mapped_idx_to_vidx[i]]
            nav_data['vertices'].append([v.x, v.y, v.name])

        nav_data['lanes'] = []
        for l in self.lanes:
            if l.params['graph_idx'].value != graph_idx:
                continue
            start_idx = vidx_to_mapped_idx[l.start_idx]
            end_idx = vidx_to_mapped_idx[l.end_idx]
            nav_data['lanes'].append(
                [start_idx, end_idx, l.orientation()])
            if l.is_bidirectional():
                nav_data['lanes'].append(
                    [end_idx, start_idx, l.reverse_orientation()])
        return nav_data
