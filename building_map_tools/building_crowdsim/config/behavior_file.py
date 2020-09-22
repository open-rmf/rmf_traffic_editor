import xml.etree.ElementTree as ET

from .leaf_element import LeafElement, Element


class BehaviorFile (Element):
    def __init__(self):
        Element.__init__(self, 'BFSM')

    def add_state(self, state):
        if not hasattr(state, "output_xml_element"):
            raise ValueError("state provided is not an Element")
        self.sub_elements.append(state)

    def add_transition(self, transition):
        if not hasattr(transition, "output_xml_element"):
            raise ValueError("transition provided is not an Element")
        self.sub_elements.append(transition)

    def add_goal_set(self, goal_set):
        if not hasattr(goal_set, "output_xml_element"):
            raise ValueError("transition provided is not an Element")
        self.sub_elements.append(goal_set)


class BehaviorState (Element):
    def __init__(self):
        Element.__init__(self, 'State')
        self.attributes['name'] = ''
        self.attributes['final'] = 0
        self.goal_selector = GoalSelector()
        self.vel_component = VelComponent()

    def is_valid(self):
        if not self.attributes['name']:
            return False
        if self.attributes['final'] == 1:
            return True
        if self.goal_selector.attributes['goal_set'] < 0:
            return False
        if not self.vel_component.attributes['file_name']:
            return False
        return True

    def load_from_yaml(self, yaml_node):
        if 'name' not in yaml_node or\
           'goal_set' not in yaml_node or\
           'navmesh_file_name' not in yaml_node or\
           'final' not in yaml_node:
            raise ValueError("Invalid Behavior State Yaml Node!")
        self.attributes['name'] = yaml_node['name']
        if int(yaml_node['final']) == 1:
            self.attributes['final'] = 1
            return
        self.attributes['final'] = 0
        self.goal_selector.attributes['goal_set'] = int(yaml_node['goal_set'])
        self.vel_component.attributes['file_name'] =\
            yaml_node['navmesh_file_name']
        self.sub_elements.append(self.goal_selector)
        self.sub_elements.append(self.vel_component)


class GoalSelector (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'GoalSelector')
        self.attributes['dist'] = 'u'
        self.attributes['type'] = 'weighted'
        self.attributes['goal_set'] = -1


class VelComponent (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'VelComponent')
        self.attributes['type'] = 'nav_mesh'
        self.attributes['heading_threshold'] = 15
        self.attributes['file_name'] = ''


class StateTransition (Element):
    def __init__(self):
        Element.__init__(self, 'Transition')
        # from state name
        self.attributes['from'] = ''
        # to state name
        self.attributes['to'] = ''
        self.condition = TransitionCondition()
        self.target = TransitionTarget()
        self.sub_elements.append(self.condition)
        self.sub_elements.append(self.target)

    def is_valid(self):
        if not self.attributes['from']:
            return False
        if not self.attributes['to'] and\
           not self.target.is_valid():
            return False
        return self.condition.is_valid()

    def load_from_yaml(self, yaml_node):
        if 'from' not in yaml_node or\
           'Condition' not in yaml_node:
            raise ValueError("Invalid Transition Yaml")
        if 'to' not in yaml_node and\
           'Target' not in yaml_node:
            raise ValueError(
                "Transition must provide either a valid 'to' State" +
                "or a valid 'Target' Set")
        self.attributes['from'] = yaml_node['from']
        if yaml_node['to']:
            self.attributes['to'] = yaml_node['to']
        self.condition.parse_condition(yaml_node['Condition'])
        if yaml_node['Target']:
            self.target.parse_target(yaml_node['Target'])


class TransitionCondition (Element):
    def __init__(self):
        Element.__init__(self, 'Condition')
        # default type='auto' means condition always met
        self.attributes['type'] = 'auto'

    def parse_condition(self, params):
        if 'type' in params:
            self.attributes['type'] = params['type']
        else:
            print("No condition type configure!" +
                  " 'auto' type will be configured instead!")
            return

        if params['type'] == 'goal_reached':
            if 'distance' not in params:
                raise ValueError(
                    "Missing 'distance' for 'goal_reached' type condition")
            if float(params['distance']) < 0.0:
                raise ValueError(
                    "Invalid goal_reached distance provided: " +
                    params['distance'])
            self.attributes['distance'] = float(params['distance'])
            return

        if params['type'] == 'timer':
            if 'per_agent' not in params:
                raise ValueError(
                    "Missing 'per_agent' [bool] for 'timer' type condition.")
            self.attributes['per_agent'] = params['per_agent']
            if 'dist' not in params:
                raise ValueError(
                    "Missing 'dist' for 'timer' type condition." +
                    "'u' for uniform distribution, " +
                    "'c' for constant value.")
            self.attributes['dist'] = params['dist']
            if params['dist'] == 'u':
                if 'min' not in params:
                    raise ValueError("Missing 'min' for 'dist'='u'")
                if 'max' not in params:
                    raise ValueError("Missing 'max' for 'dist'='u'")
                if float(params['min']) < 0 or \
                   float(params['max']) < 0 or \
                   float(params['max']) <= float(params['min']):
                    raise ValueError(
                        "Invalid 'min' and 'max' value for 'timer' condition")
                else:
                    self.attributes['min'] = params['min']
                    self.attributes['max'] = params['max']
            elif params['dist'] == 'c':
                if 'value' not in params:
                    raise ValueError("Missing 'value' for 'dist'='c'")
                if float(params['value']) < 0.0:
                    raise ValueError("Invalid 'value' for 'timer' condition")
                self.attributes['value'] = params['value']
            else:
                raise ValueError(
                    "'dist' must be configured as either 'u' or 'c'")
            return

        if params['type'] == 'not':
            if 'condition1' not in params:
                raise ValueError(
                    "A subcondition 'condition1' must be included.")
            sub_condition = TransitionCondition()
            sub_condition.parse_condition(params['condition1'])
            self.sub_elements.append(sub_condition)

        if params['type'] == 'and' or params['type'] == 'or':
            if 'condition1' not in params or 'condition2' not in params:
                raise ValueError(
                    "2 conditions 'condition1' and 'condition2' " +
                    "must be included.")
            condition1 = TransitionCondition()
            condition1.parse_condition(params['condition1'])
            condition2 = TransitionCondition()
            condition2.parse_condition(params['condition2'])
            self.sub_elements.append(condition1)
            self.sub_elements.append(condition2)


class TransitionTarget (Element):
    def __init__(self):
        Element.__init__(self, 'Target')
        self.attributes['type'] = 'prob'

    def is_valid(self):
        if len(self.sub_elements) == 0:
            return False
        return True

    def parse_target(self, params):
        if not params:
            return

        for state_item in params:
            tmp = TargetState()
            tmp.parse_state(state_item)
            self.sub_elements.append(tmp)


class TargetState (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'State')

    def parse_state(self, params):
        if 'weight' in params and 'name' in params:
            self.attributes['weight'] = params['weight']
            self.attributes['name'] = params['name']
        else:
            raise ValueError(
                "'weight' and 'name' are required for Target State")


class GoalSet (Element):
    def __init__(self):
        Element.__init__(self, 'GoalSet')
        self.attributes['id'] = -1
        self.goal_list = []
        self.goal_area = set()
        self.capacity = 1

    def is_valid(self):
        if self.attributes['id'] < 0:
            return False
        if len(self.goal_list) == 0:
            return False
        return True

    def add_goal(self, goal):
        assert(isinstance(goal, Goal))
        goal.attributes['id'] = len(self.goal_list)
        goal.attributes['capacity'] = self.capacity
        self.goal_list.append(goal)
        self.sub_elements.append(goal)

    def load_from_yaml(self, yaml_node, human_goals):
        if 'set_id' not in yaml_node or\
           'set_area' not in yaml_node or\
           'capacity' not in yaml_node:
            raise ValueError("Invalid GoalSet Yaml!")
        self.attributes['id'] = yaml_node['set_id']
        for area in yaml_node['set_area']:
            self.goal_area.add(area)
        self.capacity = int(yaml_node['capacity'])

        for area in self.goal_area:
            if area not in human_goals:
                raise ValueError(
                    "Invalid goal area name: " +
                    area +
                    " please check the building.yaml")
            for goal in human_goals[area]:
                cur_goal = Goal()
                cur_goal.attributes['x'] = float(goal[0])
                cur_goal.attributes['y'] = float(goal[1])
                self.add_goal(cur_goal)


class Goal (LeafElement):
    def __init__(self):
        LeafElement.__init__(self, 'Goal')
        self.attributes['id'] = -1
        self.attributes['type'] = 'point'
        self.attributes['x'] = 0.0
        self.attributes['y'] = 0.0
        self.attributes['weight'] = 1.0
        self.attributes['capacity'] = 1

    def is_valid(self):
        return self.attributes['id'] >= 0
