/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
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

#include <fstream>

#include "scenario.h"
#include "yaml_utils.h"

using std::string;


Scenario::Scenario()
{
}

Scenario::~Scenario()
{
}

bool Scenario::load()
{
  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(filename.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", filename.c_str(), e.what());
    return false;
  }

  if (yaml["name"])
    name = yaml["name"].as<string>();

  return true;
}

bool Scenario::save() const
{
  YAML::Node y;
  y["version"] = 1;
  y["name"] = name;

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  return true;
}
