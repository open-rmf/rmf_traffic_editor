#ifndef CROWD_SIM_AGENT_PROFILE__H
#define CROWD_SIM_AGENT_PROFILE__H

#include <string>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

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

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

  std::string profile_name;
  size_t profile_class, max_neighbors, obstacle_set;
  double max_accel, max_angle_vel, max_speed, neighbor_dist, pref_speed, r,
    ORCA_tau, ORCA_tauObst;
};

} //namespace crowd_sim

#endif