#ifndef CROWD_SIM_GOAL_SET__H
#define CROWD_SIM_GOAL_SET__H

#include <string>
#include <set>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

class GoalSet
{
public:
  GoalSet(size_t goal_id)
  : _id(goal_id),
    _capacity(1),
    _goal_area_contained({})
  {}

  GoalSet(const YAML::Node& input)
  : _id(65535), //initialize with invalid id
    _capacity(1),
    _goal_area_contained({})
  {
    from_yaml(input);
  }

  void add_goal_area(std::string goal_area_name);
  void set_capacity(size_t capacity) { this->_capacity = capacity; }

  std::set<std::string> get_goal_areas() const
  {
    return this->_goal_area_contained;
  }
  YAML::Node get_goal_areas_to_yaml() const;
  size_t get_goal_set_id() const { return this->_id; }
  size_t get_capacity() const {return this->_capacity; }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  size_t _id;
  size_t _capacity;
  std::set<std::string> _goal_area_contained;

  void _set_goal_set_id(size_t id_) { this->_id = id_; }
};

using GoalSetPtr = std::shared_ptr<GoalSet>;
} //namespace crowd_sim

#endif