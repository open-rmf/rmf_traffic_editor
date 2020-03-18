/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef MODEL_H
#define MODEL_H

/*
 * This class represents an instance of a model on a navigation map
 * The pixmap for visualization is stored in the EditorModel class; this
 * class instead represents the _placement_ of a Model reference on
 * a map.
 */

#include <string>
#include <yaml-cpp/yaml.h>

class Model
{
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  std::string model_name;
  std::string instance_name;
  bool selected = false;  // only for visualization, not saved to YAML
  bool is_static = true;

  Model();

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node &data);

  void set_param(const std::string &name, const std::string &value);
};

#endif
