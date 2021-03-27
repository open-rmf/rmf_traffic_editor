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

#ifndef CROWD_SIM_CONDITION__H
#define CROWD_SIM_CONDITION__H

#include <string>
#include <set>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

/*
 * Condition base class. This base class controls the name and type of all the other kind conditions
 * and provides the basic interface functions for all conditions
 */
class Condition;
using ConditionPtr = std::shared_ptr<Condition>;

class Condition
{
public:
  enum TYPE
  {
    BASE,
    GOAL,
    TIMER,
    AND,
    OR,
    NOT
  };

  Condition()
  : _name("base_condition"), _type(BASE)
  {}
  Condition(std::string name, TYPE type)
  : _name(name), _type(type)
  {}
  virtual ~Condition() {}

  ConditionPtr init_from_yaml(const YAML::Node& input);

  std::string get_condition_name() const { return _name; }
  TYPE get_type() const { return _type; }

  virtual bool is_valid() const { return false; }
  virtual YAML::Node to_yaml() const { return YAML::Node(YAML::NodeType::Map); }
  virtual void from_yaml(const YAML::Node& input)   // default do nothing
  {
    if (!input["type"]) printf("Invalid Condition yaml input. \n");
  }

private:
  std::string _name;
  TYPE _type;
};

/*
 * LeafCondition class. This class provides the interface function for "goal_reached" condition and "timer" condition
 */
class LeafCondition : public Condition
{
public:
  LeafCondition(
    const std::string& condition_name,
    Condition::TYPE condition_type)
  : Condition(condition_name, condition_type),
    _value(0)
  {}
  LeafCondition(
    const std::string& condition_name,
    Condition::TYPE condition_type, double condition_value)
  : Condition(condition_name, condition_type),
    _value(condition_value)
  {}
  virtual ~LeafCondition() {}

  double get_value() const {return _value;}
  void set_value(double condition_value) { _value = condition_value; }

private:
  double _value;
};

/*
 * BoolCondition class. This class provides the interface function for "and", "or", "not" condition
 */
class BoolCondition : public Condition
{
public:
  BoolCondition(
    const std::string& condition_name,
    Condition::TYPE condition_type)
  : Condition(condition_name, condition_type),
    _condition1(nullptr),
    _condition2(nullptr)
  {}
  BoolCondition(
    const std::string& condition_name,
    Condition::TYPE condition_type,
    ConditionPtr condition_ptr_1)
  : Condition(condition_name, condition_type),
    _condition1(condition_ptr_1),
    _condition2(nullptr)
  {}
  BoolCondition(
    const std::string& condition_name,
    Condition::TYPE condition_type,
    ConditionPtr condition_ptr_1,
    ConditionPtr condition_ptr_2)
  : Condition(condition_name, condition_type),
    _condition1(condition_ptr_1),
    _condition2(condition_ptr_2)
  {}
  virtual ~BoolCondition() {}

  void set_condition(ConditionPtr condition, int condition_index);
  void set_condition(ConditionPtr condition); //default set condition1
  ConditionPtr get_condition(int condition_index) const;
  ConditionPtr get_condition() const; //default return condition1
  bool is_valid() const override;

  virtual YAML::Node to_yaml() const override;
  virtual void from_yaml(const YAML::Node& input) override;

private:
  ConditionPtr _condition1, _condition2;
};

/*
 * Real Conditions
 */
//==============================================================
class ConditionGOAL final : public LeafCondition
{
public:
  ConditionGOAL()
  : LeafCondition("goal_reached", GOAL, 0.1)
  {}
  ~ConditionGOAL() {}

  bool is_valid() const override { return true; }
  YAML::Node to_yaml() const override;
  void from_yaml(const YAML::Node& input) override;
};

using ConditionGoalPtr = std::shared_ptr<ConditionGOAL>;

//==============================================================
class ConditionTIMER final : public LeafCondition
{
public:
  ConditionTIMER()
  : LeafCondition("timer", TIMER, 30.0),
    _distribution("c")
  {}
  ~ConditionTIMER() {}

  std::string get_timer_distribution() const { return this->_distribution;}

  bool is_valid() const override { return true; }
  YAML::Node to_yaml() const override;
  void from_yaml(const YAML::Node& input) override;

private:
  // currently only provides const value distribution for timer
  std::string _distribution;
};

using ConditionTimerPtr = std::shared_ptr<ConditionTIMER>;

//==============================================================
class ConditionAND final : public BoolCondition
{
public:
  ConditionAND()
  : BoolCondition(
      "and",
      AND,
      std::make_shared<Condition>(),
      std::make_shared<Condition>())
  {}
  ~ConditionAND() {}
};

using ConditionAndPtr = std::shared_ptr<ConditionAND>;

//==============================================================
class ConditionOR final : public BoolCondition
{
public:
  ConditionOR()
  : BoolCondition(
      "or",
      OR,
      std::make_shared<Condition>(),
      std::make_shared<Condition>())
  {}
  ~ConditionOR() {}
};

using ConditionOrPtr = std::shared_ptr<ConditionOR>;

//==============================================================
class ConditionNOT final : public BoolCondition
{
public:
  ConditionNOT()
  : BoolCondition(
      "not",
      NOT,
      std::make_shared<Condition>())
  {}
  ~ConditionNOT() {}
};

using ConditionNotPtr = std::shared_ptr<ConditionNOT>;

} //namespace crowd_sim

#endif