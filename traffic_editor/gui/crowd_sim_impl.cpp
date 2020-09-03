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

void State::from_yaml(const YAML::Node& input) {
    setName(input["name"].as<std::string>());
    setNavmeshFileName(input["navmesh_file_name"].as<std::string>());
    setFinalState(input["final"].as<int>() == 0? false : true);
    setGoalSetId(input["goal_set"].as<int>());
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

void GoalSet::from_yaml(const YAML::Node& input) {
    setGoalSetId(input["set_id"].as<size_t>());
    setCapacity(input["capacity"].as<size_t>());
    goal_area_contained.clear();
    for (auto area : input["set_area"]) {
        addGoalArea(area.as<std::string>());
    }
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

void AgentProfile::from_yaml(const YAML::Node& input) {
    profile_name = input["name"].as<std::string>();
    profile_class = input["class"].as<size_t>();
    max_neighbors = input["max_neighbors"].as<size_t>();
    obstacle_set = input["obstacle_set"].as<size_t>();
    max_accel = input["max_accel"].as<double>();
    max_angle_vel = input["max_angle_vel"].as<double>();
    max_speed = input["max_speed"].as<double>();
    neighbor_dist = input["neighbor_dist"].as<double>();
    pref_speed = input["pref_speed"].as<double>();
    r = input["r"].as<double>();
    ORCA_tau = input["ORCA_tau"].as<double>();
    ORCA_tauObst = input["ORCA_tauObst"].as<double>();  
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

void Transition::from_yaml(const YAML::Node& input) {
    setFromState(input["from"].as<std::string>());
    if ( input["to"] && input["to"].as<std::string>() != "" ) {
        addToState(input["to"].as<std::string>());
    }
    const YAML::Node& targets = input["Target"];
    for (YAML::const_iterator it = targets.begin(); it != targets.end(); it++) {
        if (!it->IsMap()) {
            throw std::runtime_error("Single Target expects a map");
        }
        addToState((*it)["name"].as<std::string>(), (*it)["weight"].as<double>() );
    }

    //set condition from yaml
    ConditionPtr condition_ptr = std::make_shared<Condition>()->init_from_yaml(input["Condition"]);
    condition_ptr->from_yaml(input["Condition"]);
    setCondition(condition_ptr);
}

ConditionPtr Condition::init_from_yaml(const YAML::Node& input) {
    if (input["type"] && input["type"].as<std::string>() == "goal_reached") {
        return std::make_shared<ConditionGOAL>();
    }
    if (input["type"] && input["type"].as<std::string>() == "timer") {
        return std::make_shared<ConditionTIMER>();
    }
    if (input["type"] && input["type"].as<std::string>() == "and") {
        return std::make_shared<ConditionAND>();
    }
    if (input["type"] && input["type"].as<std::string>() == "or") {
        return std::make_shared<ConditionOR>();
    }
    if (input["type"] && input["type"].as<std::string>() == "not") {
        return std::make_shared<ConditionNOT>();
    }
    //default
    return std::make_shared<Condition>();
}

void BoolCondition::setCondition(ConditionPtr condition, int condition_index) {
    if (!condition) return;
    if (condition_index == 1)
        this->condition1 = condition;
    if (condition_index == 2)
        this->condition2 = condition;
}

void BoolCondition::setCondition(ConditionPtr condition) {
    setCondition(condition, 1);
 }

ConditionPtr BoolCondition::getCondition(int condition_index) const {
    if (condition_index == 1)
        return this->condition1;
    if (condition_index == 2) {
        if (this->getType() == Condition::TYPE::NOT) {
            return this->condition1; }
        return this->condition2;
    }
    return this->condition1;
}

ConditionPtr BoolCondition::getCondition() const {
    return getCondition(1);
}

bool BoolCondition::isValid() const {
    if (this->getType() == Condition::TYPE::NOT) {
        if (condition1->isValid()) return true;
    }
    else {
        if(condition1->isValid() && condition2->isValid()) { return true; }
    }
    std::cout << "Invalid <" << this->getConditionName() << "> condition" << std::endl;
    return false;
}

YAML::Node BoolCondition::to_yaml() const {
    YAML::Node bool_node = YAML::Node(YAML::NodeType::Map);
    bool_node.SetStyle(YAML::EmitterStyle::Block);
    bool_node["type"] = getConditionName();
    bool_node["condition1"] = getCondition(1)->to_yaml();
    if (getCondition(2)) {
        bool_node["condition2"] = getCondition(2)->to_yaml();
    }
    return bool_node;
}

void BoolCondition::from_yaml(const YAML::Node& input) {
    if (input["condition1"]) {
        auto condition1_ptr = init_from_yaml(input["condition1"]);
        condition1_ptr->from_yaml(input["condition1"]);
        setCondition(condition1_ptr, 1);
    }

    if (input["condition2"]) {
        auto condition2_ptr = init_from_yaml(input["condition2"]);
        condition2_ptr->from_yaml(input["condition2"]);
        setCondition(condition2_ptr, 2);
    }
}

YAML::Node ConditionGOAL::to_yaml() const {
    YAML::Node goal_node = YAML::Node(YAML::NodeType::Map);
    goal_node.SetStyle(YAML::EmitterStyle::Flow);
    goal_node["type"] = getConditionName();
    goal_node["distance"] = getValue();
    return goal_node;
}

void ConditionGOAL::from_yaml(const YAML::Node& input) {
    if (input["type"].as<std::string>() != "goal_reached" ) {
        throw std::runtime_error("Error in parsing goal_reached condition");
    }
    if (input["distance"] && input["distance"].as<double>() > 0) {
        setValue(input["distance"].as<double>() );
    } 
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

void ConditionTIMER::from_yaml(const YAML::Node& input) {
    if (input["type"].as<std::string>() != "timer" ) {
        throw std::runtime_error("Error in parsing timer condition");
    }
    if (input["value"] && input["value"].as<double>() > 0) {
        setValue(input["value"].as<double>() );
    } 
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

void AgentGroup::from_yaml(const YAML::Node& input) {
    group_id = input["group_id"].as<size_t>();
    spawn_point_x = input["x"].as<double>();
    spawn_point_y = input["y"].as<double>();
    spawn_number = input["agents_number"].as<int>();
    agent_profile = input["profile_selector"].as<std::string>();
    initial_state = input["state_selector"].as<std::string>();
    const YAML::Node& agent_name_node = input["agents_name"];
    for (YAML::const_iterator it = agent_name_node.begin(); it != agent_name_node.end(); it++) {
        external_agent_name.emplace_back( (*it).as<std::string>() );
    }
    if (external_agent_name.size() > 0) {
        is_external_group = true;
    }
}

//==========================================================
YAML::Node ModelType::to_yaml() const {
    YAML::Node model_node = YAML::Node(YAML::NodeType::Map);
    model_node.SetStyle(YAML::EmitterStyle::Flow);
    model_node["typename"] = getName();
    model_node["animation"] = getAnimation();
    model_node["animation_speed"] = getAnimationSpeed();
    model_node["gazebo"] = gazebo_conf.to_yaml();
    model_node["ign"] = ign_conf.to_yaml();
    return model_node;
}

void ModelType::from_yaml(const YAML::Node& input) {
    name = input["typename"].as<std::string>();
    animation = input["animation"].as<std::string>();
    animation_speed = input["animation_speed"].as<double>();
    gazebo_conf.from_yaml(input["gazebo"]);
    ign_conf.from_yaml(input["ign"]);
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
    agent_groups[0].setAgentProfile("external_agent");
    agent_groups[0].setInitialState("external_static");
}

void CrowdSimImplementation::initializeModelType() {
    if (model_types.size() == 0) 
        model_types.emplace_back("human", "walk");
    else
        model_types[0] = ModelType("human", "walk");
    auto& default_type = model_types.at(0);
    default_type.setAnimationSpeed(0.2);
    default_type.setGazeboConf(
        "walk.dae",
        {0, 0, 0, 0, 0, 0}
    );
    default_type.setIgnConf(
        "https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor",
        {0, 0, 0, 0, 0, 0}
    );
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
    top_node["update_time_step"] = update_time_step;

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

    top_node["agent_groups"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto group : agent_groups) {
        top_node["agent_groups"].push_back(group.to_yaml());
    }

    top_node["model_types"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto model_type : model_types) {
        top_node["model_types"].push_back(model_type.to_yaml());
    }
    
    return top_node;
}

bool CrowdSimImplementation::from_yaml(const YAML::Node& input) {
    
    // check
    if (!input["goal_sets"] || !input["goal_sets"].IsSequence()) {
        printf("Error in load goal_sets\n");
        return false;
    }
    if (!input["states"] || !input["states"].IsSequence()) {
        printf("Error in load states\n");
        return false;
    }
    if (!input["transitions"] || !input["transitions"].IsSequence()) {
        printf("Error in load transitions\n");
        return false;
    }
    if (!input["agent_profiles"] || !input["agent_profiles"].IsSequence()) {
        printf("Error in load agent_profiles\n");
        return false;
    }
    if (!input["agent_groups"] || !input["agent_groups"].IsSequence()) {
        printf("Error in load agent_groups\n");
        return false;
    }
    if (!input["model_types"] || !input["model_types"].IsSequence()) {
        printf("Error in load model_types\n");
        return false;
    }

    clear();
    // load part
    if (input["update_time_step"] && input["update_time_step"].as<double>() > 0) {
        this->update_time_step = input["update_time_step"].as<double>();
    }
    if (input["enable"] && input["enable"].as<int>() == 1) {
        this->enable_crowd_sim = true;
    } else {
        // default disable crowd_simulation
        this->enable_crowd_sim = false;
    }

    const YAML::Node& goal_set_node = input["goal_sets"];
    for (YAML::const_iterator it = goal_set_node.begin(); it != goal_set_node.end(); it++) {
        GoalSet goalset_temp(*it);
        this->goal_sets.emplace_back(goalset_temp);
    }
    printf("crowd_sim loaded %lu goal_sets\n", this->goal_sets.size());
    
    const YAML::Node& state_node = input["states"];
    for (YAML::const_iterator it = state_node.begin(); it != state_node.end(); it++) {
        State state_temp(*it);
        this->states.emplace_back(state_temp);
    }
    printf("crowd_sim loaded %lu states\n", this->states.size());

    const YAML::Node& transition_node = input["transitions"];
    for (YAML::const_iterator it = transition_node.begin(); it != transition_node.end(); it++) {
        Transition transition_temp(*it);
        this->transitions.emplace_back(transition_temp);
    }
    printf("crowd_sim loaded %lu transitions\n", this->transitions.size());

    const YAML::Node& agent_profile_node = input["agent_profiles"];
    for (YAML::const_iterator it = agent_profile_node.begin(); it != agent_profile_node.end(); it++) {
        AgentProfile agent_profile_temp(*it);
        this->agent_profiles.emplace_back(agent_profile_temp);
    }
    printf("crowd_sim loaded %lu agent_profiles\n", this->agent_profiles.size());
    
    const YAML::Node& agent_group_node = input["agent_groups"];
    for (YAML::const_iterator it = agent_group_node.begin(); it!= agent_group_node.end(); it++) {
        AgentGroup agent_group_temp(*it);
        this->agent_groups.emplace_back(agent_group_temp);
    }
    printf("crowd_sim loaded %lu agent_groups\n", this->agent_profiles.size());

    const YAML::Node& model_type_node = input["model_types"];
    for (YAML::const_iterator it = model_type_node.begin(); it != model_type_node.end(); it++) {
        ModelType model_type_temp(*it);
        this->model_types.emplace_back(model_type_temp);
    }
    printf("crowd_sim loaded %lu model_types\n", this->agent_profiles.size());

    init_default_configure();
    return true;
}

void CrowdSimImplementation::clear() {
    goal_areas.clear();
    navmesh_filename_list.clear();
    
    enable_crowd_sim = false;
    update_time_step = 0.1;
    states.clear();
    goal_sets.clear();
    transitions.clear();
    agent_profiles.clear();
    agent_groups.clear();
    model_types.clear();
}

void CrowdSimImplementation::init_default_configure() {
    initializeState();
    initializeAgentProfile();
    initializeAgentGroup();
    initializeModelType();
}
    
} //namespace crowd_sim
