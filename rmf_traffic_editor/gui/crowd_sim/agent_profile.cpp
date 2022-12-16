/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

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
