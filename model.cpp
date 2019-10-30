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

#include "model.h"
using std::string;


Model::Model()
: x(0), y(0), yaw(0), selected(false)
{
}

Model::Model(
    const double _x,
    const double _y,
    const double _yaw,
    const std::string &_model_name,
    const std::string &_instance_name)
: x(_x), y(_y), yaw(_yaw),
  model_name(_model_name),
  instance_name(_instance_name),
  selected(false)
{
}

void Model::from_yaml(const YAML::Node &data)
{
  if (!data.IsMap())
    throw std::runtime_error("Model::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  yaw = data["yaw"].as<double>();
  model_name = data["model_name"].as<string>();
  instance_name = data["name"].as<string>();
}

YAML::Node Model::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n["x"] = round(x * 1000.0) / 1000.0;
  n["y"] = round(y * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = round(yaw * 10000.0) / 10000.0;
  n["name"] = instance_name;
  n["model_name"] = model_name;
  return n;
}
