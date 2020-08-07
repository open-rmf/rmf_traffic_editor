#ifndef CROWD_SIM_IMPL__H
#define CROWD_SIM_IMPL__H

#include <string>
#include <vector>
#include <set>
#include <memory>

namespace crowd_sim {

class State
{
public:
    State(std::string state_name);
    ~State();

    bool isValid();
    void setNavmeshFile(std::string file_name);
    void setFinalState(bool is_final);
    void setGoalSetId(size_t goal_set_id);
    void setName(std::string name);

    std::string getName();
    std::string getNavmeshFileName();
    bool getFinalState();
    size_t getGoalSetId();

private:
    std::string name = "";
    std::string navmesh_file_name = "";
    bool is_final_state = true;
    size_t goal_set_id = 0;
};

class GoalSet
{
public:
    GoalSet(size_t goal_id);
    ~GoalSet();

    void addGoalArea(std::string goal_area_name);
    std::set<std::string> getGoalAreas();
    size_t getGoalSetId();

private:
    size_t id;
    std::set<std::string> goal_area_contained;
};

// class Condition
// {

// };

// class Transition
// {
// public:
//     Transition(State* from_state);
//     ~Transition();

// private:
//     std::shared_ptr<State> from_state = nullptr;
//     std::shared_ptr<State> to_state = nullptr;
//     std::shared_ptr<Condition> condition = nullptr;
// };

class CrowdSimImplementation
{
public:
    CrowdSimImplementation() {}
    ~CrowdSimImplementation() {}

    std::vector<std::string> getGoalAreas();
    std::vector<std::string> getNavmeshFileName();
    
    std::vector<State> states;
    std::vector<GoalSet> goal_sets;
    // std::vector<Transition> transitions;
    // std::vector<AgentProfile> agent_profiles;
    // std::vector<AgentGroup> agent_groups;
    // std::vector<ModelType> model_types;

    std::set<std::string> goal_areas;
    std::vector<std::string> navmesh_filename_list;
};

} //namespace crowd_sim
using CrowdSimImplPtr = std::shared_ptr<crowd_sim::CrowdSimImplementation>;

#endif
