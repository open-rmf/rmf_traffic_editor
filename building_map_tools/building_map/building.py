import os
from xml.etree.ElementTree import Element, SubElement

from .level import Level


class Building:
    def __init__(self, yaml_node):
        if 'building_name' in yaml_node:
          self.name = yaml_node['building_name']
        else:
          self.name = yaml_node['name']
        print(f'building name: {self.name}')

        self.levels = {}
        for level_name, level_yaml in yaml_node['levels'].items():
            self.levels[level_name] = Level(level_yaml, level_name)

    def __str__(self):
        s = ''
        for level_name, level_data in self.levels.items():
            s += f'{level_name}: ({len(level_data.vertices)} vertices) '
        return s

    def generate_nav_graphs(self):
        """ Returns a dict of all non-empty nav graphs """
        print("generating nav data")
        nav_graphs = {}
        # at the moment, graphs are numbered 0..9
        # iterate through and generate any non-empty graphs
        for i in range(0, 9):
            g = {}
            g['building_name'] = self.name
            g['levels'] = {}
            empty = True
            for level_name, level in self.levels.items():
                level_graph = level.generate_nav_graph(i)
                g['levels'][level_name] = level_graph
                if level_graph['lanes']:
                    empty = False
            if not empty:
                nav_graphs[f'{i}'] = g
        return nav_graphs

    def generate_sdf_world(self):
        """ Return an etree of this Building in SDF XML """

        sdf = Element('sdf', {'version': '1.6'})

        world = SubElement(sdf, 'world')
        world.set('name', 'default')

        gui = SubElement(world, 'gui')

        user_camera = SubElement(gui, 'camera')
        user_camera.set('name', 'user_camera')

        user_camera_pose = SubElement(user_camera, 'pose')
        center_xy = self.center()
        user_camera_pose.text = f'{center_xy[0]} {center_xy[1]-20} 10 0 0.6 1.57'

        scene = SubElement(world, 'scene')
        ambient_ele = SubElement(scene, 'ambient')
        ambient_ele.text = '0.8 0.8 0.8 1.0'
        background_ele = SubElement(scene, 'background')
        background_ele.text = '0 0 0'

        include_sun = SubElement(world, 'include')
        sun_uri = SubElement(include_sun, 'uri')
        sun_uri.text = 'model://sun'

        for level_name, level in self.levels.items():
            level.generate_sdf_models(world)  # todo: a better name
            level.generate_doors(world)

            level_include_ele = SubElement(world, 'include')
            level_model_name = f'{self.name}_{level_name}'
            name_ele = SubElement(level_include_ele, 'name')
            name_ele.text = level_model_name
            uri_ele = SubElement(level_include_ele, 'uri')
            uri_ele.text = f'model://{level_model_name}'
            pose_ele = SubElement(level_include_ele, 'pose')
            pose_ele.text = '0 0 0 0 0 0'

        return sdf

    def generate_sdf_models(self, models_path):
        for level_name, level in self.levels.items():
            model_name = f'{self.name}_{level_name}'
            model_path = os.path.join(models_path, model_name)
            if not os.path.exists(model_path):
                os.makedirs(model_path)

            level.generate_sdf_model(model_name, model_path)

    def center(self):
        # todo: something smarter in the future. For now just the center
        # of the first level
        return self.levels[list(self.levels.keys())[0]].center()
