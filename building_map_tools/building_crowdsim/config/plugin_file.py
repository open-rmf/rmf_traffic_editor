import xml.etree.ElementTree as ET

from configfile_generator.leaf_element import LeafElement, Element

class Plugin (Element) :
    def __init__(self, model_env) :
        Element.__init__(self, 'plugin')
        if model_env == "gazebo" :
            self.addAttribute('filename', 'libcrowd_simulator.so')
        elif model_env == 'ign' :
            self.addAttribute('filename', 'libcrowd_simulator_ign.so')
        else :
            raise ValueError("Unknown 'model_env' provided to initialize Plugin: [" + model_env + "]")
        
        self._model_env = model_env
        
        self.addAttribute('name', 'crowd_simulation')

        self._behavior_file = LeafElement('behavior_file')
        self._behavior_file.setText('behavior_file.xml')

        self._scene_file = LeafElement('scene_file')
        self._scene_file.setText('scene_file.xml')

        self._update_time_step = LeafElement('update_time_step')
        self._update_time_step.setText('0.1')

        self.addSubElement(self._behavior_file)
        self.addSubElement(self._scene_file)
        self.addSubElement(self._update_time_step)
    
    def setBehaviorFileName(self, behavior_file_name) :
        self._behavior_file.setText(str(behavior_file_name))

    def setSceneFileName(self, scene_file_name) :
        self._scene_file.setText(str(scene_file_name))

    def setUpdateTimeStep(self, update_time_step) :
        self._update_time_step.setText(str(update_time_step))

    def addModelType(self, model_type) :
        if not hasattr(model_type, "outputXmlElement") :
            raise ValueError("model_type provided is not an element!")
        model_type.setModelEnv(self._model_env)
        self.addSubElement(model_type)
    
    def addExternalAgent(self, name) :
        external_agent = LeafElement('external_agent')
        external_agent.setText(name)
        self.addSubElement(external_agent)

    def addExternalAgentList(self, name_list):
        for name in name_list:
            self.addExternalAgent(name)
    

#########################################################
class ModelType (Element) :

    def __init__(self) :
        Element.__init__(self, 'model_type')
        self._type_name = LeafElement('typename')
        self._animation_speed = LeafElement('animation_speed')
        self._animation = LeafElement('animation')
        # for gazebo model
        self._initial_pose_gazebo = LeafElement('initial_pose')
        self._filename = LeafElement('filename') 
        # for ign model
        self._initial_pose_ign = LeafElement('initial_pose')
        self._model_file_path = LeafElement('model_file_path')

        self.addSubElement(self._type_name)
        self.addSubElement(self._animation)
        self.addSubElement(self._animation_speed)
        
        self._model_env = None

    def setModelEnv(self, model_env) :
        self._model_env = model_env

    def setElement(self, key, value) :
        if key == "typename" :
            self._type_name.setText(value)
            return

        if key == "animation_speed" :
            self._animation_speed.setText(value)
            return
        
        if key == "animation" :
            self._animation.setText(value)
            return
        
        if key == "gazebo" :
            for k in value :
                self.setGazeboModel(k, value[k])
            return
        
        if key == "ign" :
            for k in value :
                self.setIgnModel(k, value[k])
            return
        
        raise ValueError("invalid params provided in model_type: [" + key + "]")

    def setGazeboModel(self, key, value) :
        if key == 'initial_pose' :
            # value would be [x, y, z, pitch, roll, yaw]
            content = ''
            for number in value :
                content += str(number) + ' '
            content = content[0:-1] #delete the last ' '
            self._initial_pose_gazebo.setText(content)
            return
        
        if key == 'filename' :
            self._filename.setText(value)
            return

        raise ValueError('invalid params provided for gazebo model:[' + key + ']')

    def setIgnModel(self, key, value) :
        if key == 'initial_pose' :
            # value would be [x, y, z, pitch, roll, yaw]
            content = ''
            for number in value :
                content += str(number) + ' '
            content = content[0:-1] #delete the last ' '
            self._initial_pose_ign.setText(content)
            return
        
        if key == 'model_file_path' :
            self._model_file_path.setText(value)
            return

        raise ValueError('invalid params provided for gazebo model:[' + key + ']')

    def outputXmlElement(self):
        if not self._type_name.getText() or not self._animation.getText() or not self._animation_speed.getText() :
            raise ValueError("Incomplete 'model_type' element")

        if self._model_env != "gazebo" and self._model_env != "ign" :
            raise ValueError("Unknown 'model_env' [" + self._model_env + "], please select 'gazebo' or 'ign'")
        
        if self._model_env == "gazebo" :
            if not self._filename.getText() or not self._initial_pose_gazebo.getText() :
                raise ValueError("Incomplete 'gazebo' model in 'model_type'")
            self.addSubElement(self._initial_pose_gazebo)
            self.addSubElement(self._filename)

            return Element.outputXmlElement(self)
            

        if self._model_env == "ign" :
            if not self._model_file_path.getText() or not self._initial_pose_ign.getText() :
                raise ValueError("Incomplete 'ign' model in 'model_type'")
            self.addSubElement(self._initial_pose_ign)
            self.addSubElement(self._model_file_path)

            return Element.outputXmlElement(self)
            