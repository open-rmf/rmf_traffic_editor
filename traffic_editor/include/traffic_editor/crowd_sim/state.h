#ifndef CROWD_SIM_STATE__H
#define CROWD_SIM_STATE__H

#include <string>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {
class State
{
public:
  State(std::string state_name)
  : _name(state_name),
    _navmesh_file_name(""),
    _is_final_state(true),
    _goal_set_id(-1)
  {}

  State(const YAML::Node& input)
  : _name("N.A."),
    _navmesh_file_name(""),
    _is_final_state(true),
    _goal_set_id(-1)
  {
    from_yaml(input);
  }

  void set_navmesh_file_name(std::string file_name)
  {
    this->_navmesh_file_name = file_name;
  }
  void set_final_state(bool is_final) { this->_is_final_state = is_final; }
  void set_goal_set_id(size_t goal_set_id)
  {
    this->_goal_set_id = static_cast<int>(goal_set_id);
  }
  void set_name(std::string name) { this->_name = name; }

  bool is_valid() const;
  std::string get_name() const {return this->_name;}
  std::string get_navmesh_file_name() const {return this->_navmesh_file_name;}
  bool get_final_state() const {return this->_is_final_state;}
  int get_goal_set_id() const {return this->_goal_set_id;}

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  std::string _name;
  std::string _navmesh_file_name;
  bool _is_final_state;
  int _goal_set_id;
};

using StatePtr = std::shared_ptr<State>;

} //namespace crowd_sim

#endif