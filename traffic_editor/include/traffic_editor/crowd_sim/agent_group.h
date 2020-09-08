#ifndef CROWD_SIM_AGENT_GROUP__H
#define CROWD_SIM_AGENT_GROUP__H

#include <string>
#include <vector>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

class AgentGroup
{
public:
  AgentGroup(size_t group_id, bool is_external_group = false)
  : _group_id(group_id),
    _is_external_group(is_external_group),
    _spawn_point_x(0.0),
    _spawn_point_y(0.0),
    _spawn_number(0),
    _external_agent_name({}),
    _agent_profile(""),
    _initial_state("")
  {}
  AgentGroup(const YAML::Node& input)
  : _group_id(65535),
    _is_external_group(false),
    _spawn_point_x(0.0),
    _spawn_point_y(0.0),
    _spawn_number(0),
    _external_agent_name({}),
    _agent_profile(""),
    _initial_state("")
  {
    from_yaml(input);
  }

  bool is_valid() const
  {
    return _agent_profile.size() > 0 && _initial_state.size() > 0;
  }
  bool is_external_group() const { return _is_external_group; }
  size_t get_group_id() const { return _group_id; }
  std::pair<double, double> get_spawn_point() const
  {
    return std::pair<double, double>(_spawn_point_x, _spawn_point_y);
  }
  std::vector<std::string> get_external_agent_name() const
  {
    return _external_agent_name;
  }
  int get_spawn_number() const { return _spawn_number; }
  std::string get_agent_profile() const { return _agent_profile; }
  std::string get_initial_state() const { return _initial_state; }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

  void set_spawn_point(double x, double y)
  {
    _spawn_point_x = x;
    _spawn_point_y = y;
  }
  void set_external_agent_name(std::vector<std::string>& external_name)
  {
    _external_agent_name.clear();
    _external_agent_name = external_name;
    _spawn_number = _external_agent_name.size();
  }
  void set_spawn_number(int number)
  {
    _spawn_number = static_cast<size_t>(number);
  }
  void set_agent_profile(std::string profile)
  {
    _agent_profile = profile;
  }
  void set_initial_state(std::string state)
  {
    _initial_state = state;
  }

private:
  size_t _group_id;
  bool _is_external_group;
  double _spawn_point_x, _spawn_point_y;
  size_t _spawn_number;
  std::vector<std::string> _external_agent_name;
  std::string _agent_profile, _initial_state;
};

} //namespace crowd_sim

#endif