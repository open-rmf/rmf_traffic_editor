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

#ifndef CROWD_SIM_MODEL_TYPE__H
#define CROWD_SIM_MODEL_TYPE__H

#include <string>
#include <vector>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

class ModelType
{
public:
  ModelType(std::string type_name, std::string animation_name)
  : _name(type_name),
    _animation(animation_name),
    _animation_speed(0.2)
  {}

  ModelType(const YAML::Node& input)
  : _name("N.A"),
    _animation("N.A"),
    _animation_speed(0.2)
  {
    from_yaml(input);
  }

  std::string get_name() const { return _name; }
  std::string get_animation() const { return _animation; }
  double get_animation_speed() const { return _animation_speed; }

  void set_name(const std::string& type_name) { _name = type_name; }
  void set_animation(const std::string& animation_name)
  {
    _animation = animation_name;
  }
  void set_animation_speed(double speed) { _animation_speed = speed; }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  std::string _name, _animation;
  double _animation_speed;
};

} //namespace crowd_sim

#endif