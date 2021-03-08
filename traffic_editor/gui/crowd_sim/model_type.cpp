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

#include <traffic_editor/crowd_sim/model_type.h>

using namespace crowd_sim;

//==============================================
YAML::Node ModelType::to_yaml() const
{
  YAML::Node model_node = YAML::Node(YAML::NodeType::Map);
  model_node.SetStyle(YAML::EmitterStyle::Flow);
  model_node["typename"] = get_name();
  model_node["animation"] = get_animation();
  model_node["animation_speed"] = get_animation_speed();
  return model_node;
}

//==============================================
void ModelType::from_yaml(const YAML::Node& input)
{
  _name = input["typename"].as<std::string>();
  _animation = input["animation"].as<std::string>();
  _animation_speed = input["animation_speed"].as<double>();
}