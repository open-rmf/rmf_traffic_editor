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

#include <traffic_editor/crowd_sim/condition.h>

#include <iostream>

using namespace crowd_sim;

//===========================================================
ConditionPtr Condition::init_from_yaml(const YAML::Node& input)
{
  if (input["type"] && input["type"].as<std::string>() == "goal_reached")
  {
    return std::make_shared<ConditionGOAL>();
  }
  if (input["type"] && input["type"].as<std::string>() == "timer")
  {
    return std::make_shared<ConditionTIMER>();
  }
  if (input["type"] && input["type"].as<std::string>() == "and")
  {
    return std::make_shared<ConditionAND>();
  }
  if (input["type"] && input["type"].as<std::string>() == "or")
  {
    return std::make_shared<ConditionOR>();
  }
  if (input["type"] && input["type"].as<std::string>() == "not")
  {
    return std::make_shared<ConditionNOT>();
  }
  //default
  return std::make_shared<Condition>();
}

//===========================================================
void BoolCondition::set_condition(ConditionPtr condition, int condition_index)
{
  if (!condition)
    return;
  if (condition_index == 1)
    this->_condition1 = condition;
  if (condition_index == 2)
    this->_condition2 = condition;
}

//===========================================================
void BoolCondition::set_condition(ConditionPtr condition)
{
  set_condition(condition, 1);
}

//===========================================================
ConditionPtr BoolCondition::get_condition(int condition_index) const
{
  if (condition_index == 1)
    return this->_condition1;
  if (condition_index == 2)
  {
    if (this->get_type() == Condition::TYPE::NOT)
    {
      return this->_condition1;
    }
    return this->_condition2;
  }
  return this->_condition1;
}

//===========================================================
ConditionPtr BoolCondition::get_condition() const
{
  return get_condition(1);
}

//===========================================================
bool BoolCondition::is_valid() const
{
  if (this->get_type() == Condition::TYPE::NOT)
  {
    if (_condition1->is_valid())
      return true;
  }
  else
  {
    if (_condition1->is_valid() && _condition2->is_valid())
    {
      return true;
    }
  }
  std::cout << "Invalid <" << this->get_condition_name() << "> condition" <<
    std::endl;
  return false;
}

//===========================================================
YAML::Node BoolCondition::to_yaml() const
{
  YAML::Node bool_node = YAML::Node(YAML::NodeType::Map);
  bool_node.SetStyle(YAML::EmitterStyle::Block);
  bool_node["type"] = get_condition_name();
  bool_node["condition1"] = get_condition(1)->to_yaml();
  if (get_condition(2))
  {
    bool_node["condition2"] = get_condition(2)->to_yaml();
  }
  return bool_node;
}

//===========================================================
void BoolCondition::from_yaml(const YAML::Node& input)
{
  if (input["condition1"])
  {
    auto condition1_ptr = init_from_yaml(input["condition1"]);
    condition1_ptr->from_yaml(input["condition1"]);
    set_condition(condition1_ptr, 1);
  }

  if (input["condition2"])
  {
    auto condition2_ptr = init_from_yaml(input["condition2"]);
    condition2_ptr->from_yaml(input["condition2"]);
    set_condition(condition2_ptr, 2);
  }
}

//===========================================================
YAML::Node ConditionGOAL::to_yaml() const
{
  YAML::Node goal_node = YAML::Node(YAML::NodeType::Map);
  goal_node.SetStyle(YAML::EmitterStyle::Flow);
  goal_node["type"] = get_condition_name();
  goal_node["distance"] = get_value();
  return goal_node;
}

//===========================================================
void ConditionGOAL::from_yaml(const YAML::Node& input)
{
  if (input["type"].as<std::string>() != "goal_reached")
  {
    throw std::runtime_error("Error in parsing goal_reached condition");
  }
  if (input["distance"] && input["distance"].as<double>() > 0)
  {
    set_value(input["distance"].as<double>() );
  }
}

//===========================================================
YAML::Node ConditionTIMER::to_yaml() const
{
  YAML::Node timer_node = YAML::Node(YAML::NodeType::Map);
  timer_node.SetStyle(YAML::EmitterStyle::Flow);
  timer_node["type"] = get_condition_name();
  timer_node["dist"] = _distribution; //currently only support const distribution
  timer_node["value"] = get_value();
  timer_node["per_agent"] = "true";
  return timer_node;
}

//===========================================================
void ConditionTIMER::from_yaml(const YAML::Node& input)
{
  if (input["type"].as<std::string>() != "timer")
  {
    throw std::runtime_error("Error in parsing timer condition");
  }
  if (input["value"] && input["value"].as<double>() > 0)
  {
    set_value(input["value"].as<double>() );
  }
}