import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element


class Plugin (Element):
    def __init__(self):
        Element.__init__(self, 'plugin')
        self.attributes['filename'] = 'libcrowd_simulator.so'
        self.attributes['name'] = 'crowd_simulation'

        self.behavior_file_element = LeafElement('behavior_file')
        self.behavior_file_element.text = 'behavior_file.xml'

        self.scene_file_element = LeafElement('scene_file')
        self.scene_file_element.text = 'scene_file.xml'

        self.update_time_step_element = LeafElement('update_time_step')
        self.update_time_step_element.text = '0.1'

        self.sub_elements.append(self.behavior_file_element)
        self.sub_elements.append(self.scene_file_element)
        self.sub_elements.append(self.update_time_step_element)

    def load_from_yaml(self, yaml_node):
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
        if 'agent_groups' not in yaml_node:
            return
        agent_groups = yaml_node['agent_groups']
        for group in agent_groups:
            if len(group['agents_name']) > 0:
                self.add_external_list(group['agents_name'])

    def add_model_type(self, model_type):
        assert(isinstance(model_type, ModelType))
        self.sub_elements.append(model_type)

    def add_external_agent(self, name):
        external_agent_element = LeafElement('external_agent')
        external_agent_element.text = name
        self.sub_elements.append(external_agent_element)

    def add_external_list(self, name_list):
        for name in name_list:
            self.add_external_agent(name)


class ModelType (Element):
    def __init__(self):
        Element.__init__(self, 'model_type')
        self.type_name = LeafElement('typename')
        self.animation_speed = LeafElement('animation_speed')
        self.animation = LeafElement('animation')
        self.model_uri = LeafElement('filename')
        self.init_pose = LeafElement('initial_pose')

        self.sub_elements.append(self.type_name)
        self.sub_elements.append(self.animation)
        self.sub_elements.append(self.animation_speed)
        self.sub_elements.append(self.model_uri)
        self.sub_elements.append(self.init_pose)

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
        elif key == "model_uri":
            self.model_uri.text = str(value)
        elif key == "init_pose":
            pose_str = ""
            for num in value:
                pose_str += str(num) + " "
            self.init_pose.text = str(pose_str[0:-1])
        else:
            raise ValueError(
                "Invalid params provided in model_type: [" + key + "]")

    def output_xml_element(self):
        if not self.type_name.text or\
           not self.animation.text or\
           not self.animation_speed.text or\
           not self.model_uri or\
           not self.init_pose:
            raise ValueError("Incomplete 'model_type' element")

        return Element.output_xml_element(self)
