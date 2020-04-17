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

#include <QtGlobal>

#include "model.h"
using std::string;


Model::Model()
{
}

void Model::from_yaml(const YAML::Node &data)
{
  if (!data.IsMap())
    throw std::runtime_error("Model::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  if (data["z"])
  {
    z = data["z"].as<double>();
  }
  else
  {
    qWarning(
        "parsed a deprecated .building.yaml, models should have z defined.");
    z = 0.0;
  }
  yaw = data["yaw"].as<double>();
  model_name = data["model_name"].as<string>();
  instance_name = data["name"].as<string>();
  if (data["static"])
    is_static = data["static"].as<bool>();
  else
    is_static = true;
}

YAML::Node Model::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n.SetStyle(YAML::EmitterStyle::Flow);
  n["x"] = round(x * 1000.0) / 1000.0;
  n["y"] = round(y * 1000.0) / 1000.0;
  n["z"] = round(z * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = round(yaw * 10000.0) / 10000.0;
  n["name"] = instance_name;
  n["model_name"] = model_name;
  n["static"] = is_static;
  return n;
}

void Model::set_param(const std::string &name, const std::string &value)
{
  if (name == "elevation")
  {
    try
    {
      z = std::stod(value);
    }
    catch(const std::exception& e)
    {
      qWarning("[elevation] field can only be a double/float.");
    }
  }
  else if (name == "static")
  {
    // not sure if there is a super elite way to parse 'true' in STL
    string lowercase(value);
    std::transform(
        lowercase.begin(),
        lowercase.end(),
        lowercase.begin(),
        [](char c) { return std::tolower(c); });

    if (value == "true")
      is_static = true;
    else
      is_static = false;
  }
  else if (name == "name")
  {
    instance_name = value;
  }
}
