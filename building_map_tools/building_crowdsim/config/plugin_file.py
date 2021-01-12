import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element


class Plugin (Element):
    def __init__(self):
        Element.__init__(self, 'plugin')
        self.attributes['filename'] = 'libcrowd_simulator.so'
        self.attributes['name'] = 'crowd_simulation'

    def load_from_yaml(self, yaml_node):
        internal_agent_names = {}
        for level_name, cur_level in yaml_node.items():
            floor = Floor(level_name)
            floor.load_from_yaml(cur_level, internal_agent_names)
            self.sub_elements.append(floor)


class Floor (Element):
    def __init__(self, level_name):
        Element.__init__(self, 'floor')
        self.attributes['name'] = level_name

        self.behavior_file_element = LeafElement('behavior_file')
        self.behavior_file_element.text = 'behavior_file.xml'

        self.scene_file_element = LeafElement('scene_file')
        self.scene_file_element.text = 'scene_file.xml'

        self.update_time_step_element = LeafElement('update_time_step')
        self.update_time_step_element.text = '0.1'

        self.sub_elements.append(self.behavior_file_element)
        self.sub_elements.append(self.scene_file_element)
        self.sub_elements.append(self.update_time_step_element)

    def load_from_yaml(self, cur_level, internal_agent_names={}):
        if cur_level.enable_crowdsim == 1:
            yaml_node = cur_level.crowd_sim_yaml
            human_yaml = cur_level.crowd_sim_human_yaml
            if 'enable' not in yaml_node:
                raise ValueError("Missing 'enable' in yaml_node")
            if 'update_time_step' not in yaml_node:
                raise ValueError("Missing 'update_time_step' in yaml_node")
            self.update_time_step_element.text = str(
                float(yaml_node['update_time_step']))

            if 'model_types' not in yaml_node:
                raise ValueError("Invalid 'model_types' in yaml_node")
            for model_type in yaml_node['model_types']:
                cur_model_type = ModelType()
                cur_model_type.load_from_yaml(model_type)
                self.add_model_type(cur_model_type)

            # get all the external agent names from 'agent_groups'
            if 'external_agent_groups' not in yaml_node:
                return
            external_agent_groups = yaml_node['external_agent_groups']
            for group in external_agent_groups:
                if len(group['agents_name']) > 0:
                    self.add_external_list(group['agents_name'])

            # get all the internal agent names from 'human_yaml'
            for internal_agent in human_yaml:
                if 'agent_group_id' in internal_agent:
                    name = internal_agent['name']
                    if name not in internal_agent_names:
                        internal_agent_names[name] = 0
                    else:
                        internal_agent_names[name] += 1
                    suffix = '_' + str(internal_agent_names[name] + 1)\
                        if internal_agent_names[name] > 0 else ''
                    self.add_internal_agent(name+suffix)

    def add_model_type(self, model_type):
        assert(isinstance(model_type, ModelType))
        self.sub_elements.append(model_type)

    def add_external_agent(self, name):
        external_agent_element = LeafElement('external_agent')
        external_agent_element.text = name
        self.sub_elements.append(external_agent_element)

    def add_internal_agent(self, name):
        internal_agent_element = LeafElement('internal_agent')
        internal_agent_element.text = name
        self.sub_elements.append(internal_agent_element)

    def add_external_list(self, name_list):
        for name in name_list:
            self.add_external_agent(name)


class ModelType (Element):
    def __init__(self):
        Element.__init__(self, 'model_type')
        self.type_name = LeafElement('typename')
        self.animation_speed = LeafElement('animation_speed')
        self.animation = LeafElement('animation')

        self.sub_elements.append(self.type_name)
        self.sub_elements.append(self.animation)
        self.sub_elements.append(self.animation_speed)

    def load_from_yaml(self, yaml_node):
        for key in yaml_node:
            self.set_tags(key, yaml_node[key])

    def set_tags(self, key, value):
        if key == "typename":
            self.type_name.text = str(value)
        elif key == "animation_speed":
            self.animation_speed.text = str(value)
        elif key == "animation":
            self.animation.text = str(value)
        else:
            raise ValueError(
                "Invalid params provided in model_type: [" + key + "]")

    def output_xml_element(self):
        if not self.type_name.text or\
           not self.animation.text or\
           not self.animation_speed.text:
            raise ValueError("Incomplete 'model_type' element")

        return Element.output_xml_element(self)
