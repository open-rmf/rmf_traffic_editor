#include <traffic_editor/crowd_sim_impl.h>

namespace crowd_sim{

std::vector<std::string> CrowdSimImplementation::getGoalAreas() {
    return std::vector<std::string>(goal_areas.begin(), goal_areas.end());
}

std::vector<std::string> CrowdSimImplementation::getNavmeshFileName() {
    return navmesh_filename_list;
}

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

    return top_node;
}
    
} //namespace crowd_sim
