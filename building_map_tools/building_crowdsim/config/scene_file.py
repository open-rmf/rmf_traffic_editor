import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element

#########################################################
class SceneFile (Element):
    
    def __init__(self) :
        Element.__init__(self, 'Experiment')
        self.addAttribute('version', 2.0)

    def addSpatialQuery(self):
        self._spatialQuery = SpatialQuery()
        self.addSubElement(self._spatialQuery)
    
    def addCommon(self):
        self._common = Common()
        self.addSubElement(self._common)

    def addObstacleSet(self, file_name, class_id):
        self._obstacleSet = ObstacleSet()
        self._obstacleSet.setNavMeshFile(file_name)
        self._obstacleSet.setClassId(class_id)
        self.addSubElement(self._obstacleSet)

    def addAgentProfile(self, profile_name) :
        tmp = AgentProfile(profile_name)
        self.addSubElement(tmp)
        return tmp

    def addAgentGroup(self, profile_name, state_name) :
        tmp = AgentGroup(profile_name, state_name)
        self.addSubElement( tmp )
        return tmp

#########################################################
class SpatialQuery (LeafElement) :
    def __init__(self) :
        LeafElement.__init__(self, 'SpatialQuery')
        self.addAttribute('type', 'kd-tree')
        self.addAttribute('test_visibility', 'false')
    
#########################################################
class Common (LeafElement):
    def __init__(self) :
        LeafElement.__init__(self, 'Common')
        self.addAttribute('time_step', 0.1)
    
#########################################################
class ObstacleSet (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'ObstacleSet')
        self.addAttribute('type', 'nav_mesh')
        self.addAttribute('file_name', '')
        self.addAttribute('class', -1)

    def setNavMeshFile(self, file_name):
        self.addAttribute('file_name', file_name)

    def setClassId(self, id):
        self.addAttribute('class', id)

#########################################################
class ProfileCommon (LeafElement) :
    def __init__(self):
        LeafElement.__init__(self, 'Common')
        self.addAttribute('class', 0)
        self.addAttribute('max_accel', 5)
        self.addAttribute('max_angle_vel', 360)
        self.addAttribute('max_neighbors', 10)
        self.addAttribute('max_speed', 2)
        self.addAttribute('neighbor_dist', 5)
        self.addAttribute('obstacleSet', None)
        self.addAttribute('pref_speed', 0)
        self.addAttribute('r', 0.2)

    def setMaxAccel(self, max_accel):
        self.addAttribute('max_accel', max_accel)

    def setR(self, r):
        self.addAttribute('r', 0.2)

    def setPrefSpeed(self, pref_speed):
        self.addAttribute('pref_speed', pref_speed)
    
    def setMaxSpeed(self, max_speed):
        self.addAttribute('max_speed', max_speed)

    def setClass(self, classid):
        self.addAttribute('class', classid)
    
    def setObstacleSet(self, obstacle_set):
        self.addAttribute('obstacle_set', obstacle_set)

#########################################################
class ProfileORCA (LeafElement) :
    def __init__(self):
        LeafElement.__init__(self, 'ORCA')
        self.addAttribute('tau', 3.0)
        self.addAttribute('tauObst', 0.15)

#########################################################
class ProfileSelector (LeafElement) :
    def __init__(self, name) :
        LeafElement.__init__(self, 'ProfileSelector')
        self.addAttribute('type', 'const')
        self.addAttribute('name', name)

#########################################################
class StateSelector (LeafElement) :
    def __init__(self, name) :
        LeafElement.__init__(self, 'StateSelector')
        self.addAttribute('type', 'const')
        self.addAttribute('name', name)

#########################################################
class Agent (LeafElement) :
    def __init__(self, x, y) :
        LeafElement.__init__(self, 'Agent')
        self.addAttribute('p_x', x)
        self.addAttribute('p_y', y)

#########################################################
class AgentGenerator (Element) :
    def __init__(self):
        Element.__init__(self, 'Generator')
        self.addAttribute('type', 'explicit')
    
    def addAgent(self, x, y):
        self.addSubElement(Agent(x, y))

#########################################################
class AgentProfile (Element):
    def __init__(self, name):
        Element.__init__(self, 'AgentProfile')
        self.addAttribute('name', name)
        self._profileCommon = ProfileCommon()
        self._profileORCA = ProfileORCA()
        self.addSubElement(self._profileCommon)
        self.addSubElement(self._profileORCA)

    def setProfileCommon(self, key, value):
        self._profileCommon.addAttribute(key, value)

    def setProfileORCA(self, key, value):
        self._profileORCA.addAttribute(key, value)
        
#########################################################
class AgentGroup (Element):

    def __init__(self, profile_type, state_name) :
        Element.__init__(self, 'AgentGroup')
        self._profileSelector = ProfileSelector(profile_type)
        self._stateSelector = StateSelector(state_name)
        self._generator = AgentGenerator()
        self.addSubElement(self._profileSelector)
        self.addSubElement(self._stateSelector)
        self.addSubElement(self._generator)

    def addAgent(self, x, y):
        self._generator.addAgent(x, y)


#########################################################
if __name__ == '__main__':

    root = SceneFile()
    root.addSpatialQuery()
    root.addCommon()
    tmp = root.addAgentProfile('walking')
    tmp.setProfileCommon('pref_speed', 2)
    tmp = root.addAgentGroup('human', 'walking')
    tmp.addAgent(0.1, 0.2)
    tmp.addAgent(0.2, 0.3)

    ET.dump(root.outputXmlElement())