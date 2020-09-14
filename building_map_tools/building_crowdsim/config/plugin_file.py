import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element


class Plugin (Element):
    def __init__(self, model_env):
        Element.__init__(self, 'plugin')
        if model_env == "gazebo":
            self.attributes['filename'] = 'libcrowd_simulator.so'
        elif model_env == 'ign':
            self.attributes['filename'] = 'libcrowd_simulator_ign.so'
        else:
            raise ValueError(
                "Unknown 'model_env' provided to initialize Plugin: [" +
                model_env + "]")

        self.model_env = model_env
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
            if len(group['agents_name']) == 0:
                continue
            self.add_external_list(group['agents_name'])

    def add_model_type(self, model_type):
        assert(isinstance(model_type, ModelType))
        model_type.model_env = self.model_env
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
        # for gazebo model
        self.initial_pose_gazebo = LeafElement('initial_pose')
        self.gazebo_filename = LeafElement('filename')
        # for ign model
        self.initial_pose_ign = LeafElement('initial_pose')
        self.ign_filename = LeafElement('filename')

        self.sub_elements.append(self.type_name)
        self.sub_elements.append(self.animation)
        self.sub_elements.append(self.animation_speed)

        self.model_env = None

    def load_from_yaml(self, yaml_node):
        for key in yaml_node:
            self.set_tags(key, yaml_node[key])

    def set_tags(self, key, value):
        if key == "typename":
            self.type_name.text = str(value)
            return
        if key == "animation_speed":
            self.animation_speed.text = str(value)
            return
        if key == "animation":
            self.animation.text = str(value)
            return
        if key == "gazebo":
            for k in value:
                self.set_gazebo_model(k, value[k])
            return
        if key == "ign":
            for k in value:
                self.set_ign_model(k, value[k])
            return
        raise ValueError(
            "Invalid params provided in model_type: [" + key + "]")

    def set_gazebo_model(self, key, value):
        if key == 'pose':
            content = ''
            for number in value:
                content += str(number) + ' '
            content = content[0:-1]
            self.initial_pose_gazebo.text = content
            return

        if key == 'filename':
            self.gazebo_filename.text = str(value)
            return

        raise ValueError(
            "invalid params provided for gazebo model:[" + key + "]")

    def set_ign_model(self, key, value):
        if key == 'pose':
            content = ''
            for number in value:
                content += str(number) + ' '
            content = content[0:-1]
            self.initial_pose_ign.text = content
            return

        if key == 'model_file_path':
            self.ign_filename.text = str(value)
            return

        raise ValueError(
            "invalid params provided for gazebo model:[" + key + "]")

    def output_xml_element(self):
        if not self.type_name.text or\
           not self.animation.text or\
           not self.animation_speed.text:
            raise ValueError("Incomplete 'model_type' element")

        if self.model_env != "gazebo" and self.model_env != "ign":
            raise ValueError(
                "Unknown 'model_env' [" +
                self.model_env +
                "], please select 'gazebo' or 'ign'")

        if self.model_env == "gazebo":
            if not self.gazebo_filename.text or\
               not self.initial_pose_gazebo.text:
                raise ValueError("Incomplete 'gazebo' model in 'model_type'")
            self.sub_elements.append(self.initial_pose_gazebo)
            self.sub_elements.append(self.gazebo_filename)
            return Element.output_xml_element(self)

        if self.model_env == "ign":
            if not self.ign_filename.text or\
               not self.initial_pose_ign.text:
                raise ValueError("Incomplete 'ign' model in 'model_type'")
            self.sub_elements.append(self.initial_pose_ign)
            self.sub_elements.append(self.gazebo_filename)
            return Element.output_xml_element(self)
