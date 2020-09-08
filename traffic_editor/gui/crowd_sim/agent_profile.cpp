#include <traffic_editor/crowd_sim/agent_profile.h>

using namespace crowd_sim;

//==============================================
YAML::Node AgentProfile::to_yaml() const
{
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

//==============================================
void AgentProfile::from_yaml(const YAML::Node& input)
{
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
