#ifndef CROWD_SIM_IMPL__H
#define CROWD_SIM_IMPL__H

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {
class State;
class GoalSet;
class Condition;
class ConditionAND;
class ConditionOR;
class ConditionNOT;
class ConditionGOAL;
class ConditionTIMER;

using StatePtr = std::shared_ptr<State>;
using GoalSetPtr = std::shared_ptr<GoalSet>;
using ConditionPtr = std::shared_ptr<Condition>;
using ConditionAndPtr = std::shared_ptr<ConditionAND>;
using ConditionOrPtr = std::shared_ptr<ConditionOR>;
using ConditionNotPtr = std::shared_ptr<ConditionNOT>;
using ConditionGoalPtr = std::shared_ptr<ConditionGOAL>;
using ConditionTimerPtr = std::shared_ptr<ConditionTIMER>;

//=========================================================
class State
{
public:
    State(std::string state_name_) 
        : name(state_name_),
        navmesh_file_name(""),
        is_final_state(true),
        goal_set_id(-1)
    {}
    State(const YAML::Node& input)
        : name("N.A."),
        navmesh_file_name(""),
        is_final_state(true),
        goal_set_id(-1)
    {
        from_yaml(input);
    }
    ~State() {}

    void setNavmeshFileName(std::string file_name_) {this->navmesh_file_name = file_name_;}
    void setFinalState(bool is_final_) { this->is_final_state = is_final_; }
    void setGoalSetId(size_t goal_set_id_) { this->goal_set_id = static_cast<int>(goal_set_id_); }
    void setName(std::string name_) { this->name = name_; }

    bool isValid() const;
    std::string getName() const {return this->name;}
    std::string getNavmeshFileName() const {return this->navmesh_file_name;}
    bool getFinalState() const {return this->is_final_state;}
    int getGoalSetId() const {return this->goal_set_id;}

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

private:
    std::string name;
    std::string navmesh_file_name;
    bool is_final_state;
    int goal_set_id;
};

//=========================================================
class GoalSet
{
public:
    GoalSet(size_t goal_id_) 
        : id(goal_id_),
        capacity(1),
        goal_area_contained({})
    {}
    GoalSet(const YAML::Node& input) 
        : id(65535), //initialize with invalid id
        capacity(1),
        goal_area_contained({})
    {
        from_yaml(input);
    }
    ~GoalSet() {}

    void addGoalArea(std::string goal_area_name);
    void setCapacity(size_t capacity_) { this->capacity = capacity_; }
    
    std::set<std::string> getGoalAreas() const { return this->goal_area_contained; }
    YAML::Node getGoalAreasToYaml() const;
    size_t getGoalSetId() const { return this->id; }
    size_t getCapacity() const {return this->capacity; }

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

private:
    size_t id;
    size_t capacity;
    std::set<std::string> goal_area_contained;

    void setGoalSetId(size_t id_) { this->id = id_; }
};

//=========================================================
class AgentProfile
{
public:
    AgentProfile(std::string profile_name_) 
        : profile_name(profile_name_),
        profile_class(1),
        max_neighbors(10),
        obstacle_set(1),
        max_accel(0.0),
        max_angle_vel(0.0),
        max_speed(0.0),
        neighbor_dist(5.0),
        pref_speed(0.0),
        r(0.25),
        ORCA_tau(1.0),
        ORCA_tauObst(0.4)
    {}
    AgentProfile(const YAML::Node& input) 
        : profile_name("N.A."),
        profile_class(1),
        max_neighbors(10),
        obstacle_set(1),
        max_accel(0.0),
        max_angle_vel(0.0),
        max_speed(0.0),
        neighbor_dist(5.0),
        pref_speed(0.0),
        r(0.25),
        ORCA_tau(1.0),
        ORCA_tauObst(0.4)
    {
        from_yaml(input);
    }
    ~AgentProfile() {}

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

    std::string profile_name;
    size_t profile_class, max_neighbors, obstacle_set;
    double max_accel, max_angle_vel, max_speed, neighbor_dist, pref_speed, r, ORCA_tau, ORCA_tauObst;
};

//=========================================================
class Condition
{
public:
    enum TYPE {
        BASE,
        GOAL,
        TIMER,
        AND,
        OR,
        NOT
    };

    Condition() : name("base_condition"), type(BASE) 
    {}
    Condition(std::string name_, TYPE type_) : name(name_), type(type_)
    {}
    virtual ~Condition() {}

    ConditionPtr init_from_yaml(const YAML::Node& input);

    virtual std::string getConditionName() const { return name; }
    virtual TYPE getType() const { return type; }
    virtual bool isValid() const { return false; }

    virtual YAML::Node to_yaml() const { return YAML::Node(YAML::NodeType::Map); }
    virtual void from_yaml(const YAML::Node& input) { //base class do nothing
        if (!input["type"]) printf("Invalid Condition yaml input.");
    }

private:
    std::string name;
    TYPE type;
};

class LeafCondition : public Condition
{
public:
    LeafCondition(const std::string& condition_name, Condition::TYPE condition_type)
        : Condition(condition_name, condition_type), value(0)
    {}
    LeafCondition(const std::string& condition_name, Condition::TYPE condition_type, double condition_value)
        : Condition(condition_name, condition_type), value(condition_value)
    {}
    virtual ~LeafCondition() {}

    double getValue() const {return value;}
    void setValue(double condition_value) {value = condition_value;}

private:
    double value;
};

class BoolCondition : public Condition
{
public:
    BoolCondition(const std::string& condition_name, Condition::TYPE condition_type)
        : Condition(condition_name, condition_type), condition1(nullptr), condition2(nullptr) 
    {}
    BoolCondition(const std::string& condition_name, Condition::TYPE condition_type, 
        ConditionPtr condition_ptr_1)
        : Condition(condition_name, condition_type), 
        condition1(condition_ptr_1), condition2(nullptr) 
    {}
    BoolCondition(const std::string& condition_name, Condition::TYPE condition_type, 
        ConditionPtr condition_ptr_1, ConditionPtr condition_ptr_2)
        : Condition(condition_name, condition_type), 
        condition1(condition_ptr_1), condition2(condition_ptr_2) 
    {}
    virtual ~BoolCondition() {}

    virtual void setCondition(ConditionPtr condition, int condition_index);
    virtual void setCondition(ConditionPtr condition); //default set condition1
    virtual ConditionPtr getCondition(int condition_index) const;
    virtual ConditionPtr getCondition() const; //default return condition1
    bool isValid() const override;

    virtual YAML::Node to_yaml() const override;
    virtual void from_yaml(const YAML::Node& input) override;

private:
    ConditionPtr condition1, condition2;
};

class ConditionGOAL : public LeafCondition 
{
public:
    ConditionGOAL() : LeafCondition("goal_reached", GOAL, 0.1)
    {}
    ~ConditionGOAL() {}

    bool isValid() const override { return true; }
    YAML::Node to_yaml() const override;
    void from_yaml(const YAML::Node& input) override;
};

class ConditionTIMER : public LeafCondition 
{
public:
    ConditionTIMER() : LeafCondition("timer", TIMER, 30.0), distribution("c")
    {}
    ~ConditionTIMER() {}

    std::string getTimerDistribution() const { return this->distribution;}
    bool isValid() const override { return true; }
    YAML::Node to_yaml() const override;
    void from_yaml(const YAML::Node& input) override;

private:
    //currently only provides const value distribution for timer
    std::string distribution;
};

class ConditionAND : public BoolCondition 
{
public:
    ConditionAND() 
        : BoolCondition("and", AND, std::make_shared<Condition>(), std::make_shared<Condition>())
    {}
    ~ConditionAND() {}
};

class ConditionOR : public BoolCondition 
{
public:
    ConditionOR() 
        : BoolCondition("or", OR, std::make_shared<Condition>(), std::make_shared<Condition>())  
    {}
    ~ConditionOR() {}
};

class ConditionNOT : public BoolCondition
{
public:
    ConditionNOT()
        : BoolCondition("not", NOT, std::make_shared<Condition>())
    {}
    ~ConditionNOT() {}
};

//=========================================================
class Transition
{
using StateName = std::string;
using ToStateType = std::map<StateName, double>;
public:
    Transition(StateName from_state_name_) 
        : from_state_name(from_state_name_),
        to_state_name({}),
        condition(std::make_shared<Condition>())
    {}
    Transition(const YAML::Node& input)
        : from_state_name("N.A."),
        to_state_name({}),
        condition(std::make_shared<Condition>())
    {
        from_yaml(input);
    }
    ~Transition() {}

    void setFromState(StateName state_name_) { this->from_state_name = state_name_; }
    std::string getFromState() const { return this->from_state_name; }

    void addToState(StateName state_name_, double weight_ = 1.0) {
        this->to_state_name.insert(std::make_pair(state_name_, weight_) );
    }
    void deleteToState(StateName state_name_) {
        this->to_state_name.erase(state_name_);
    }
    ToStateType getToState() const {
        return this->to_state_name;
    }
    void clearToState() {
        this->to_state_name.clear();
    }

    void setCondition(ConditionPtr condition_) { this->condition = condition_; }
    ConditionPtr getCondition() const { return condition; }

    bool isValid() {
        if (condition->isValid() && to_state_name.size() > 0 && from_state_name.size() > 0)
            return true; 
        std::cout << "Invalid transition" << std::endl;
        return false;
    }

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

private:
    StateName from_state_name;
    ToStateType to_state_name;
    ConditionPtr condition;
};

//=========================================================
class AgentGroup
{
public:
    AgentGroup(size_t group_id_, bool is_external_group_ = false) 
        : group_id(group_id_), 
        is_external_group(is_external_group_),
        spawn_point_x(0.0),
        spawn_point_y(0.0),
        spawn_number(0),
        external_agent_name({}),
        agent_profile(""),
        initial_state("")
    {}
    AgentGroup(const YAML::Node& input) 
        : group_id(65535), 
        is_external_group(false),
        spawn_point_x(0.0),
        spawn_point_y(0.0),
        spawn_number(0),
        external_agent_name({}),
        agent_profile(""),
        initial_state("")
    {
        from_yaml(input);
    }
    ~AgentGroup() {}

    bool isValid() const { return agent_profile.size() > 0 && initial_state.size() > 0; }
    bool isExternalGroup() const { return is_external_group; }
    size_t getGroupId() const { return group_id; }
    std::pair<double, double> getSpawnPoint() const { return std::pair<double, double>(spawn_point_x, spawn_point_y); }
    std::vector<std::string> getExternalAgentName() const { return external_agent_name; }
    int getSpawnNumber() const { return spawn_number; }
    std::string getAgentProfile() const { return agent_profile; }
    std::string getInitialState() const { return initial_state; }

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

    void setSpawnPoint(double x, double y) {
        spawn_point_x = x;
        spawn_point_y = y;
    }
    void setExternalAgentName(std::vector<std::string>& external_name) {
        external_agent_name.clear();
        for (auto name : external_name) {
            external_agent_name.emplace_back(name);
        }
        spawn_number = static_cast<int>(external_agent_name.size());
    }
    void setSpawnNumber(int number) {
        spawn_number = number;
    }
    void setAgentProfile(std::string profile) {
        agent_profile = profile;
    }
    void setInitialState(std::string state) {
        initial_state = state;
    }

private:
    size_t group_id;
    bool is_external_group = false;
    double spawn_point_x, spawn_point_y;
    int spawn_number;
    std::vector<std::string> external_agent_name;
    std::string agent_profile, initial_state;
};

//========================================================
class ModelType 
{
public:
    struct GazeboConf {
        std::string filename;
        std::vector<double> initial_pose;

        GazeboConf(std::string file = "", std::vector<double> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
            : filename(file), initial_pose(pose) {}
        YAML::Node to_yaml() const{
            YAML::Node result = YAML::Node(YAML::NodeType::Map);
            result.SetStyle(YAML::EmitterStyle::Flow);
            result["filename"] = filename;
            result["pose"] = YAML::Node(YAML::NodeType::Sequence);
            result["pose"] = initial_pose;
            return result;
        }
        void from_yaml(const YAML::Node& input) {
            filename = input["filename"].as<std::string>();
            const YAML::Node& pose_node = input["pose"];
            size_t i = 0;
            for (YAML::const_iterator it = pose_node.begin(); 
                it != pose_node.end() && i < initial_pose.size(); 
                it++) {
                initial_pose[i++] = (*it).as<double>();
            }
        }
    };

    struct IgnConf{
        std::string filename;
        std::vector<double> initial_pose;
        IgnConf(std::string file = "", std::vector<double> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
            : filename(file), initial_pose(pose) {}
        YAML::Node to_yaml() const{
            YAML::Node result = YAML::Node(YAML::NodeType::Map);
            result.SetStyle(YAML::EmitterStyle::Flow);
            result["model_file_path"] = filename;
            result["pose"] = YAML::Node(YAML::NodeType::Sequence);
            result["pose"] = initial_pose;
            return result;
        }
        void from_yaml(const YAML::Node& input) {
            filename = input["model_file_path"].as<std::string>();
            const YAML::Node& pose_node = input["pose"];
            size_t i = 0;
            for (YAML::const_iterator it = pose_node.begin(); 
                it != pose_node.end() && i < initial_pose.size(); 
                it++) {
                initial_pose[i++] = (*it).as<double>();
            }
        }
    };

public:
    ModelType(std::string type_name, std::string animation_name) 
        : name(type_name),
        animation(animation_name),
        animation_speed(0.2),
        gazebo_conf(),
        ign_conf()
     {}
     ModelType(const YAML::Node& input) 
        : name("N.A"),
        animation("N.A"),
        animation_speed(0.2),
        gazebo_conf(),
        ign_conf()
     {
        from_yaml(input);
     }
    ~ModelType() {}

    std::string getName() const {return name;}
    std::string getAnimation() const {return animation;}
    double getAnimationSpeed() const {return animation_speed;}
    GazeboConf getGazeboConf() const {return gazebo_conf;}
    IgnConf getIgnConf() const {return ign_conf;}

    void setName(std::string type_name_) { name = type_name_; }
    void setAnimation(std::string animation_name_) { animation = animation_name_; }
    void setAnimationSpeed(double speed_) { animation_speed = speed_; }
    void setGazeboConf(std::string file, std::vector<double> pose) {
        gazebo_conf = GazeboConf(file, pose);
    }
    void setIgnConf(std::string file, std::vector<double> pose) {
        ign_conf = IgnConf(file, pose);
    }

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);

private:
    std::string name, animation;
    double animation_speed;
    GazeboConf gazebo_conf;
    IgnConf ign_conf;
};

//=========================================================
class CrowdSimImplementation
{
public:
    CrowdSimImplementation() 
        : enable_crowd_sim(false),
        update_time_step(0.1)
    {
        init_default_configure();
    }
    ~CrowdSimImplementation() {}

    std::vector<std::string> getGoalAreas() const { 
        return std::vector<std::string>(goal_areas.begin(), goal_areas.end()); }
    std::vector<std::string> getNavmeshFileName() const {return navmesh_filename_list;}

    YAML::Node to_yaml();
    bool from_yaml(const YAML::Node& input);
    void clear();
    void init_default_configure();

    // update from project.building in crowd_sim_table
    std::set<std::string> goal_areas;
    std::vector<std::string> navmesh_filename_list;
    
    // real configurations
    bool enable_crowd_sim;
    double update_time_step;
    std::vector<State> states;
    std::vector<GoalSet> goal_sets;
    std::vector<Transition> transitions;
    std::vector<AgentProfile> agent_profiles;
    std::vector<AgentGroup> agent_groups;
    std::vector<ModelType> model_types;

private:
    void initializeState();
    void initializeAgentProfile();
    void initializeAgentGroup();
    void initializeModelType();

    //obstacle set is not allowed to change
    YAML::Node output_obstacle_node() const;
};

} //namespace crowd_sim
using CrowdSimImplPtr = std::shared_ptr<crowd_sim::CrowdSimImplementation>;

#endif
