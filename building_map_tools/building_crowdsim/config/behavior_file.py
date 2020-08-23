import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element

#########################################################
class BehaviorFile (Element):
    
    def __init__(self) :
        Element.__init__(self, 'BFSM')

    def addState(self, state) :
        if not hasattr(state, "outputXmlElement"):
            raise ValueError("state provided is not an Element")
        self.addSubElement(state)

    def addTransition(self, transition) :
        if not hasattr(transition, "outputXmlElement"):
            raise ValueError("transition provided is not an Element")
        self.addSubElement(transition)

    def addGoalSet(self, goal_set) :
        if not hasattr(goal_set, "outputXmlElement"):
            raise ValueError("transition provided is not an Element")
        self.addSubElement(goal_set)

#########################################################
class BehaviorState (Element):
    def __init__(self) :
        Element.__init__(self, 'State')
        self.addAttribute('name', '')
        self.addAttribute('final', 0)
        self._goalSelector = GoalSelector()
        self._velComponent = VelComponent()

    def setStateName(self, name) :
        if len(name) != 0 :
            self.addAttribute('name', name)
        else :
            raise ValueError("invalid name provided for state name")

    def getStateName(self) :
        # from LeafElement
        return self._attrib['name']

    def setGoalSelector(self, goal_selector) :
        if not hasattr(goal_selector, "outputXmlElement"):
            raise ValueError("goal_selector provided is not an element!")
        self._goalSelector = goal_selector
        self.addSubElement(self._goalSelector)

    def setVelComponent(self, vel_component) :
        if not hasattr(vel_component, "outputXmlElement"):
            raise ValueError("vel_component provided is not an element!")
        self._velComponent = vel_component
        self.addSubElement(self._velComponent)
            
    def setFinalState(self) : 
        self.addAttribute('final', 1)
    
    def setUnFinalState(self) :
        self.addAttribute('final', 0)
    

#########################################################
class GoalSelector (LeafElement):

    def __init__(self) : 
        LeafElement.__init__(self, 'GoalSelector')
        self.addAttribute('dist', 'u')
        self.addAttribute('type', 'weighted')
        self.addAttribute('goal_set', -1) # not set
    
    def setGoalSetId(self, goal_set_id) :
        self.addAttribute('goal_set', goal_set_id)

#########################################################
class VelComponent (LeafElement): 

    def __init__(self) : 
        LeafElement.__init__(self, 'VelComponent')
        self.addAttribute('type', 'nav_mesh')
        self.addAttribute('heading_threshold', 15)
        self.addAttribute('file_name', '') # not set

    def setNavMeshFile(self, file_name) : 
        self.addAttribute('file_name', file_name)

#########################################################
class StateTransition (Element):

    def __init__(self) :
        Element.__init__(self, 'Transition')
        self.addAttribute('from', '')
        self.addAttribute('to', '')
        self._condition = TransitionCondition() 
        self._target = TransitionTarget()
    
    def setFromState(self, state) :
        if state.getStateName() != '' :
            self.addAttribute('from', state.getStateName())
        else :
            raise ValueError("The transition FromState provided is not a xml element")

    def setFromStateName(self, state_name) :
        if len( state_name ) > 0 :
            self.addAttribute('from', state_name)
        else :
            raise ValueError("Invalid state name for state transition")

    def setToState(self, state):
        if state.getStateName() != '' :
            self.addAttribute('to', state.getStateName())
        else :
            raise ValueError("The transition ToState provided is not a xml element")
    
    def setToStateName(self, state_name) :
        # to state can be None, as there might be a Target
        if not state_name:
            return
        if len( state_name ) > 0 :
            self.addAttribute('to', state_name)
        else :
            raise ValueError("Invalid state name for state transition")

    def parseCondition(self, condition_params) :
        # recommended method to set a condition for transition
        self._condition.parseCondition(condition_params)
        self.addSubElement(self._condition)

    def parseTarget(self, target_params) :
        self._target.parseTarget(target_params)
        self.addSubElement(self._target)

#########################################################
class TransitionCondition (Element):

    def __init__(self) :
        Element.__init__(self, 'Condition')
        self.addAttribute('type', 'auto')
    
    def setType(self, condition_type):
        self.addAttribute('type', condition_type)

    def parseCondition(self, params) :
        # params in dictionary formate
        if 'type' in params :
            self.setType( params['type'] )
        else:
            print("No condition type configure! 'auto' type will be configured instead!")
            return
        
        # leaf condition for goal_reached, only 'distance' provided
        if params['type'] == 'goal_reached' :
            if not 'distance' in params:
                raise ValueError("missing 'distance' for 'goal_reached' type condition")
            self.setGoalReachDistance(params['distance'])
            return
        
        # leaf condition for timer
        if params['type'] == 'timer' :
            if not 'per_agent' in params :
                raise ValueError("missing 'per_agent' [bool] for 'timer' type condition.")
            
            self.addAttribute('per_agent', params['per_agent'])
            
            if not 'dist' in params:
                raise ValueError("missing 'dist' ['u' for uniform and 'c' for const distribution] for 'timer' type condition.")
            
            self.addAttribute('dist', params['dist'])
            if params['dist'] == 'u' :
                if not 'min' in params:
                    raise ValueError("missing 'min' for 'dist'='u'")
                if not 'max' in params:
                    raise ValueError("missing 'max' for 'dist'='u'")
                if not float(params['min']) > 0 or not float(params['max']) > 0 or not float(params['max']) >= float(params['min']) :
                    raise ValueError("invalid 'min' and 'max' value for 'timer' condition")
                else :
                    self.addAttribute('min', params['min'])
                    self.addAttribute('max', params['max'])
            elif params['dist'] == 'c' :
                if not 'value' in params :
                    raise ValueError("missing 'value' for 'dist'='c'")
                if not float(params['value']) > 0 :
                    raise ValueError("invalid 'value' for 'timer' condition")
                self.addAttribute('value', params['value'])
            else :
                raise ValueError("'dist' must be configured as 'u' or 'c'")
            return
        
        if params['type'] == 'not':
            if not 'condition1' in params :
                raise ValueError("a subcondition 'condition1' must be included.")
            sub_condition = TransitionCondition()
            sub_condition.parseCondition( params['condition1'] )
            self.addSubElement(sub_condition)
        
        if params['type'] == 'and' or params['type'] == 'or':
            if not 'condition1' in params or not 'condition2' in params:
                raise ValueError("two subconditions 'condition1' and 'condition2' must be included.")
            condition1 = TransitionCondition()
            condition1.parseCondition( params['condition1'] )
            condition2 = TransitionCondition()
            condition2.parseCondition( params['condition2'] )
            self.addSubElement(condition1)
            self.addSubElement(condition2)

    
    def setGoalReachDistance(self, dist) :
        if(float(dist) > 0) :
            self.addAttribute('distance', dist)
        else :
            raise ValueError('invalid condition distance provided for TransitionCondition')


#########################################################
class TransitionTarget (Element) :
    
    def __init__(self) :
        Element.__init__(self, 'Target')
        self.addAttribute('type', 'prob')

    def setType(self, target_type) :
        # should not attempt to changeg the type of transition target
        self.addAttribute('type', target_type)

    def parseTarget(self, params):
        # params in [{}, {}, {}] format
        if not params:
            return
        
        for state_item in params:
            tmp = TargetState()
            tmp.parseState(state_item)
            self.addSubElement(tmp)

#########################################################
class TargetState (LeafElement) :

    def __init__(self):
        LeafElement.__init__(self, 'State')
        
    def parseState(self, params):
        if not 'weight' in params or not 'name' in params :
            raise ValueError("'weight' and 'name' are required for Target State")
        self.addAttribute('weight', params['weight'])
        self.addAttribute('name', params['name'])

#########################################################
class GoalSet (Element): 

    def __init__(self) :
        Element.__init__(self, 'GoalSet')
        self.addAttribute('id', -1)
        self._goalList = []
        self._goal_area = set()
        self._capacity = 1

    def setId(self, id) :
        self.addAttribute('id', str(id))
    
    def addGoalArea(self, area) :
        self._goal_area.add(area)

    def setCapacity(self, capacity) :
        self._capacity = capacity

    def addGoal(self, goal) :
        goal.setId( len(self._goalList) )
        self._goalList.append(goal)
        self.addSubElement(goal)


#########################################################
class Goal (LeafElement):
    def __init__(self, id = -1) :
        # default id = -1, need to initialize
        LeafElement.__init__(self, 'Goal')
        self.addAttribute('id', -1)
        self.addAttribute('type', 'point')
        self.addAttribute('x', 0.0)
        self.addAttribute('y', 0.0)
        self.addAttribute('weight', 1.0)
        self.addAttribute('capacity', 1)

    def setCoord(self, x, y) :
        self.addAttribute('x', float(x))
        self.addAttribute('y', float(y))

    def setWeight(self, weight) :
        self.addAttribute('weight', float(weight))

    def setId(self, id) :
        self.addAttribute('id', int(id))

    def setCapacity(self, capacity) : 
        self.addAttribute('capacity', capacity)
