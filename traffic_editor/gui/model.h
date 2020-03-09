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

#include "param.h"

class Model
{
public:
  double x;
  double y;
  Param z;
  double yaw;
  std::string model_name;
  std::string instance_name;
  bool selected;  // only for visualization, not saved to YAML

  Model();
  Model(
      const double _x,
      const double _y,
      const double _z,
      const double _yaw,
      const std::string &_model_name,
      const std::string &_instance_name);

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node &data);

  void set_param(const std::string &name, const std::string &value);
};

#endif
