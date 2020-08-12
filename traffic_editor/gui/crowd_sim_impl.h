#ifndef CROWD_SIM_IMPL__H
#define CROWD_SIM_IMPL__H

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>

namespace crowd_sim {
class State;
class GoalSet;
class Condition;

using StatePtr = std::shared_ptr<State>;
using GoalSetPtr = std::shared_ptr<GoalSet>;
using ConditionPtr = std::shared_ptr<Condition>;

//=========================================================
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

//=========================================================
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

//=========================================================
class AgentProfile
{
public:
    AgentProfile(std::string profile_name) : profile_name(profile_name) {}
    ~AgentProfile() {}

    std::string profile_name = "new_profile";
    // default external_agent profile setup
    size_t profile_class = 1;
    double max_accel = 0.0;
    double max_angle_vel = 0.0;
    size_t max_neighbors = 10;
    double max_speed = 0.0;
    double neighbor_dist = 5.0;
    size_t obstacle_set = 1;
    double pref_speed = 0.0;
    double r = 0.25;
    double ORCA_tau = 1.0;
    double ORCA_tauObst = 0.4;
};

//=========================================================
class Condition
{
public:
    Condition() {}
    ~Condition() {}

    virtual std::string getConditionName() {
        return name;
    }
    virtual bool isValid() {
        return false;
    }

    enum TYPE {
        BASE,
        GOAL,
        TIMER,
        AND,
        OR,
        NOT
    } type;
    
    std::string name = "base_condition";
};

class ConditionGOAL : public Condition 
{
public:
    ConditionGOAL() {
        type = GOAL;
        name = "goal_reached";
    }
    ~ConditionGOAL() {}

    void setDistance(double distance) {
        this->distance = distance;
    }

    double getDistance() {
        return distance;
    }

    bool isValid() override {
        return true;
    }

private:
    double distance = 0.1;
};

class ConditionTimer : public Condition {
public:
    ConditionTimer() {
        type = TIMER;
        name = "timer";
    }
    ~ConditionTimer() {}

    void setDuration(double value) {
        this->duration = value;
    }

    double getDuration() {
        return this->duration;
    }

    std::string getTimerDistribution() {
        return this->distribution;
    }

    bool isValid() override {
        return true;
    }

private:
    //currently only provides const value distribution for timer
    std::string distribution = "c";
    double duration = 30.0;
};

class ConditionAND : public Condition 
{
public:
    ConditionAND() {
        type = AND;
        name = "and";
    }
    ~ConditionAND() {}

    void setConditions(ConditionPtr condition1, ConditionPtr condition2) {
        this->condition1 = condition1;
        this->condition2 = condition2;
    }

    bool isValid() override {
        if(condition1 && condition2 && condition1->isValid() && condition2->isValid()) {
            return true;
        }
        return false;
    }

private:
    ConditionPtr condition1;
    ConditionPtr condition2;
};

class ConditionOR : public Condition 
{
public:
    ConditionOR() {
        type = OR;
        name = "or";
    }
    ~ConditionOR() {}

    void setConditions(ConditionPtr condition1, ConditionPtr condition2) {
        this->condition1 = condition1;
        this->condition2 = condition2;
    }

    bool isValid() override {
        if(condition1 && condition2 && condition1->isValid() && condition2->isValid()) {
            return true;
        }
        return false;
    }

private:
    ConditionPtr condition1;
    ConditionPtr condition2;
};

class ConditionNot : public Condition
{
public:
    ConditionNot() {
        type = NOT;
        name = "not";
    }
    ~ConditionNot() {}

    void setConditions(ConditionPtr condition1) {
        this->condition1 = condition1;
    } 

    bool isValid() override {
        if(condition1 && condition1->isValid()) {
            return true;
        }
        return false;
    }

private:
    ConditionPtr condition1;
};

//=========================================================
class Transition
{
using StateName = std::string;
using ToStateType = std::map<StateName, double>;
public:
    Transition(StateName from_state_name) {
        this->from_state_name = from_state_name;
        this->condition = std::make_shared<Condition>();
    }
    ~Transition() {}

    void setFromState(StateName state_name) {
        this->from_state_name = state_name;
    }
    std::string getFromState() const {
        return this->from_state_name;
    }

    void addToState(StateName state_name, double weight = 1.0) {
        this->to_state_name.insert(std::make_pair(state_name, weight) );
    }
    void deleteToState(StateName state_name) {
        this->to_state_name.erase(state_name);
    }
    ToStateType getToState() const {
        return this->to_state_name;
    }
    void clearToState() {
        this->to_state_name.clear();
    }

    void setCondition(ConditionPtr condition) {
        this->condition = condition;
    }

    ConditionPtr getCondition() const {
        return condition;
    }

    bool isValid() {
        if (condition && condition->isValid() && to_state_name.size() > 0 && from_state_name.size() > 0) {
            return true;
        } 
        return false;
    }

private:
    StateName from_state_name = "";
    //output states with weights
    ToStateType to_state_name;
    ConditionPtr condition;
};

//=========================================================
class CrowdSimImplementation
{
public:
    CrowdSimImplementation() {
        external_agent = std::make_shared<AgentProfile>("external_agent");
        clearAgentProfile();
    }
    ~CrowdSimImplementation() {}

    std::vector<std::string> getGoalAreas();
    std::vector<std::string> getNavmeshFileName();
    void clearAgentProfile();
    
    std::vector<State> states;
    std::vector<GoalSet> goal_sets;
    std::vector<Transition> transitions;
    std::vector<AgentProfile> agent_profiles;
    // std::vector<AgentGroup> agent_groups;
    // std::vector<ModelType> model_types;

    std::set<std::string> goal_areas;
    std::vector<std::string> navmesh_filename_list;

private:
    std::shared_ptr<AgentProfile> external_agent;
};

} //namespace crowd_sim
using CrowdSimImplPtr = std::shared_ptr<crowd_sim::CrowdSimImplementation>;

#endif
