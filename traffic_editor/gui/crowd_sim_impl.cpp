#include <crowd_sim_impl.h>

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

bool State::isValid(){
    return true;
}

GoalSet::GoalSet(size_t goal_id) : id(goal_id) {}

GoalSet::~GoalSet() {}

void GoalSet::addGoalArea(std::string area_name){
    this->goal_area_contained.insert(area_name);
}

