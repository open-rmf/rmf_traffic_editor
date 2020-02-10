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

#ifndef PROJECT_H
#define PROJECT_H

#include "building.h"
#include "scenario.h"

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>


class Project
{
public:
  std::string name;
  std::string filename;

  Building building;
  std::vector<Scenario> scenarios;

  /////////////////////////////////
  Project();
  ~Project();

  bool save();
  bool load(const std::string& _filename);

private:
  bool load_yaml_file(const std::string& _filename);
  bool save_yaml_file() const;
};


#endif
