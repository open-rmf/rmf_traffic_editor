#ifndef CROWD_SIM_IMPL__H
#define CROWD_SIM_IMPL__H

#include <string>
#include <vector>
#include <unordered_set>
#include <memory>

class State
{
public:
    State(std::string state_name);
    ~State();

    bool isValid();
    void setNavmeshFile(std::string file_name);
    void setFinalState(bool is_final);
    void setGoalSetId(size_t goal_set_id);

private:
    std::string name;
    std::string navmesh_file_name;
    bool is_final_state;
    size_t goal_set_id;
};

class GoalSet
{
public:
    GoalSet(size_t goal_id);
    ~GoalSet();

    void addGoalArea(std::string goal_area_name);
private:
    size_t id;
    std::unordered_set<std::string> goal_area_contained;
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

    std::vector<State> states;
    std::vector<GoalSet> goal_sets;
    // std::vector<Transition> transitions;
    // std::vector<AgentProfile> agent_profiles;
    // std::vector<AgentGroup> agent_groups;
    // std::vector<ModelType> model_types;
};

using CrowdSimImplPtr = std::shared_ptr<CrowdSimImplementation>;

#endif
