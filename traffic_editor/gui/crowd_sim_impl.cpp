#include <traffic_editor/crowd_sim_impl.h>

namespace crowd_sim{

//=================================================
bool State::isValid() const{
    if (is_final_state && name.size() > 0) return true;
    // not final state
    if (name.size() > 0 && navmesh_file_name.size() > 0 && goal_set_id >= 0) return true;
    return false;
}

YAML::Node State::to_yaml() const {
    YAML::Node state_node(YAML::NodeType::Map);
    state_node.SetStyle(YAML::EmitterStyle::Flow);
    state_node["name"] = getName();
    state_node["goal_set"] = getGoalSetId();
    state_node["navmesh_file_name"] = getNavmeshFileName();
    state_node["final"] = getFinalState()? 1 : 0;
    return state_node;
}

//===================================================
void GoalSet::addGoalArea(std::string area_name){
    if (area_name.empty()){
        std::cout << "Invalid area_name provided." << std::endl;
    }
    this->goal_area_contained.insert(area_name);
}

YAML::Node GoalSet::getGoalAreasToYaml() const {
    YAML::Node goal_area = YAML::Node(YAML::NodeType::Sequence);
    goal_area.SetStyle(YAML::EmitterStyle::Flow);
    for (auto area : getGoalAreas()) {
        goal_area.push_back(area);
    }
    return goal_area;
}

YAML::Node GoalSet::to_yaml() const {
    YAML::Node goalset_node(YAML::NodeType::Map);
    goalset_node.SetStyle(YAML::EmitterStyle::Flow);
    goalset_node["set_id"] = getGoalSetId();
    goalset_node["capacity"] = getCapacity();
    goalset_node["set_area"] = getGoalAreasToYaml();
    return goalset_node;
}

//====================================================
YAML::Node AgentProfile::to_yaml() const {
    YAML::Node profile_node(YAML::NodeType::Map);
    profile_node.SetStyle(YAML::EmitterStyle::Flow);
    profile_node["name"] = profile_name;
    profile_node["class"] = profile_class;
    profile_node["max_accel"] = max_accel;
    profile_node["max_angle_vel"] = max_angle_vel;
    profile_node["max_neighbors"] = max_neighbors;
    profile_node["max_speed"] = max_speed;
    profile_node["neighbor_dist"] = neighbor_dist;
    profile_node["obstacle_set"] = obstacle_set;
    profile_node["pref_speed"] = pref_speed;
    profile_node["r"] = r;
    profile_node["ORCA_tau"] = ORCA_tau;
    profile_node["ORCA_tauObst"] = ORCA_tauObst;
    return profile_node;
}

//===========================================================
YAML::Node Transition::to_yaml() const{
    YAML::Node transition_node(YAML::NodeType::Map);
    transition_node.SetStyle(YAML::EmitterStyle::Flow);
    transition_node["from"] = from_state_name;
    transition_node["to"] = to_state_name.size() == 1 ? to_state_name.begin()->first : "";
    transition_node["Condition"] = condition->to_yaml();
    transition_node["Target"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto to_state : to_state_name) {
        YAML::Node target_node = YAML::Node(YAML::NodeType::Map);
        target_node["name"] = to_state.first;
        target_node["weight"] = to_state.second;
        transition_node["Target"].push_back(target_node);
    }
    return transition_node;
}

YAML::Node ConditionGOAL::to_yaml() const {
    YAML::Node goal_node = YAML::Node(YAML::NodeType::Map);
    goal_node.SetStyle(YAML::EmitterStyle::Flow);
    goal_node["type"] = getConditionName();
    goal_node["distance"] = getValue();
    return goal_node;
}

YAML::Node ConditionTIMER::to_yaml() const {
    YAML::Node timer_node = YAML::Node(YAML::NodeType::Map);
    timer_node.SetStyle(YAML::EmitterStyle::Flow);
    timer_node["type"] = getConditionName();
    timer_node["dist"] = distribution; //currently only support const distribution
    timer_node["value"] = getValue();
    timer_node["per_agent"] = "true";
    return timer_node;
}

YAML::Node ConditionAND::to_yaml() const {
    YAML::Node and_node = YAML::Node(YAML::NodeType::Map);
    and_node.SetStyle(YAML::EmitterStyle::Block);
    and_node["type"] = getConditionName();
    and_node["condition1"] = condition1->to_yaml();
    and_node["condition2"] = condition2->to_yaml();
    return and_node;
}

YAML::Node ConditionOR::to_yaml() const {
    YAML::Node or_node = YAML::Node(YAML::NodeType::Map);
    or_node.SetStyle(YAML::EmitterStyle::Block);
    or_node["type"] = getConditionName();
    or_node["condition1"] = condition1->to_yaml();
    or_node["condition2"] = condition2->to_yaml();
    return or_node;
}

YAML::Node ConditionNOT::to_yaml() const {
    YAML::Node not_node = YAML::Node(YAML::NodeType::Map);
    not_node.SetStyle(YAML::EmitterStyle::Block);
    not_node["type"] = getConditionName();
    not_node["condition1"] = condition1->to_yaml();
    return not_node;
}

//===========================================================
YAML::Node AgentGroup::to_yaml() const {
    YAML::Node group_node = YAML::Node(YAML::NodeType::Map);
    group_node.SetStyle(YAML::EmitterStyle::Flow);
    group_node["group_id"] = group_id;
    group_node["profile_selector"] = agent_profile;
    group_node["state_selector"] = initial_state;
    group_node["agents_number"] = spawn_number;
    group_node["agents_name"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto name : external_agent_name) {
        group_node["agents_name"].push_back(name);
    }
    group_node["x"] = spawn_point_x;
    group_node["y"] = spawn_point_y;
    return group_node;
}

//===========================================================
void CrowdSimImplementation::initializeState() {
    if(states.size() == 0)
        states.emplace_back("external_static");
    else
        states[0] = State("external_static");
}

void CrowdSimImplementation::initializeAgentProfile() {
    if (agent_profiles.size() == 0)
        agent_profiles.emplace_back("external_agent");
    else 
        agent_profiles[0] = AgentProfile("external_agent");
}

void CrowdSimImplementation::initializeAgentGroup() {
    if (agent_groups.size() == 0) 
        agent_groups.emplace_back(0, true);
    else
        agent_groups[0] = AgentGroup(0, true);
}

YAML::Node CrowdSimImplementation::output_obstacle_node() const {
    // Need to refactoring later. Currently with hard code configuration.
    YAML::Node obstacle_node = YAML::Node(YAML::NodeType::Map);
    obstacle_node.SetStyle(YAML::EmitterStyle::Flow);
    obstacle_node["class"] = 1;
    obstacle_node["type"] = "nav_mesh";
    obstacle_node["file_name"] = this->navmesh_filename_list[0];
    return obstacle_node;
}

YAML::Node CrowdSimImplementation::to_yaml() {
    YAML::Node top_node = YAML::Node(YAML::NodeType::Map);
    top_node["enable"] = enable_crowd_sim? 1 : 0;

    top_node["states"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto state : states) {
        if (!state.isValid()) continue;
        top_node["states"].push_back(state.to_yaml());
    }

    top_node["goal_sets"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto goal_set : goal_sets) {
        top_node["goal_sets"].push_back(goal_set.to_yaml());
    }

    top_node["agent_profiles"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto profile : agent_profiles) {
        top_node["agent_profiles"].push_back(profile.to_yaml());
    }

    top_node["transitions"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto transition : transitions) {
        top_node["transitions"].push_back(transition.to_yaml());
    }

    top_node["obstacle_set"] = output_obstacle_node();

    top_node["agent_group"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto group : agent_groups) {
        top_node["agent_group"].push_back(group.to_yaml());
    }
    

    return top_node;
}
    
} //namespace crowd_sim
