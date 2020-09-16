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
  struct GazeboConf
  {
    std::string filename;
    std::string idle_filename;
    std::vector<double> initial_pose;

    GazeboConf(
      std::string file = "",
      std::string idle_file = "",
      std::vector<double> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    : filename(file),
      idle_filename(idle_file),
      initial_pose(pose)
    {}

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);
  };

  struct IgnConf
  {
    std::string filename;
    std::vector<double> initial_pose;
    IgnConf(
      std::string file = "",
      std::vector<double> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    : filename(file),
      initial_pose(pose)
    {}

    YAML::Node to_yaml() const;
    void from_yaml(const YAML::Node& input);
  };

public:
  ModelType(std::string type_name, std::string animation_name)
  : _name(type_name),
    _animation(animation_name),
    _animation_speed(0.2),
    _gazebo_conf(),
    _ign_conf()
  {}

  ModelType(const YAML::Node& input)
  : _name("N.A"),
    _animation("N.A"),
    _animation_speed(0.2),
    _gazebo_conf(),
    _ign_conf()
  {
    from_yaml(input);
  }

  std::string get_name() const {return _name;}
  std::string get_animation() const {return _animation;}
  double get_animation_speed() const {return _animation_speed;}
  GazeboConf get_gazebo_conf() const {return _gazebo_conf;}
  IgnConf get_ign_conf() const {return _ign_conf;}

  void set_name(std::string type_name) { _name = type_name; }
  void set_animation(std::string animation_name)
  {
    _animation = animation_name;
  }
  void set_animation_speed(double speed) { _animation_speed = speed; }
  void set_gazebo_conf(std::string file, std::string idle_file, std::vector<double> pose)
  {
    _gazebo_conf = GazeboConf(file, idle_file, pose);
  }
  void set_ign_conf(std::string file, std::vector<double> pose)
  {
    _ign_conf = IgnConf(file, pose);
  }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  std::string _name, _animation;
  double _animation_speed;
  GazeboConf _gazebo_conf;
  IgnConf _ign_conf;
};

} //namespace crowd_sim

#endif