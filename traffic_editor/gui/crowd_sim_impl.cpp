#include <crowd_sim_impl.h>

namespace crowd_sim{

std::vector<std::string> CrowdSimImplementation::getGoalAreas() {
    return std::vector<std::string>(goal_areas.begin(), goal_areas.end());
}

std::vector<std::string> CrowdSimImplementation::getNavmeshFileName() {
    return navmesh_filename_list;
}

void CrowdSimImplementation::clearAgentProfile() {
    agent_profiles.clear();
    agent_profiles.emplace_back(*external_agent);
}

State::State(std::string state_name) : name(state_name) {}

State::~State() {}

void State::setFinalState(bool is_final) {
    this->is_final_state = is_final;
}

void State::setGoalSetId(size_t goal_set_id) {
    this->goal_set_id = goal_set_id;
}

void State::setNavmeshFile(std::string file_name) {
    this->navmesh_file_name = file_name;
}

void State::setName(std::string name) {
    this->name = name;
}

bool State::isValid() {
    return true;
}

std::string State::getName() {
    return this->name;
}

std::string State::getNavmeshFileName() {
    return this->navmesh_file_name;
}

bool State::getFinalState() {
    return this->is_final_state;
}

size_t State::getGoalSetId() {
    return this->goal_set_id;
}

GoalSet::GoalSet(size_t goal_id) : id(goal_id) {}

GoalSet::~GoalSet() {}

void GoalSet::addGoalArea(std::string area_name){
    this->goal_area_contained.insert(area_name);
}

size_t GoalSet::getGoalSetId() {
    return this->id;
}

std::set<std::string> GoalSet::getGoalAreas() {
    return this->goal_area_contained;
}
    
} //namespace crowd_sim
