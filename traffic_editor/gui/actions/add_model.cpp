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

#include "add_model.h"
#include <math.h>

AddModelCommand::AddModelCommand(
  Project* project,
  int level_idx,
  double x,
  double y,
  std::string name)
{
  _project = project;
  _level_idx = level_idx;
  _x = x;
  _y = y;
  _name = name;
}

AddModelCommand::~AddModelCommand()
{

}

void AddModelCommand::undo()
{
  for (size_t i = 0; i < _project->building.levels[_level_idx].models.size();
    i++)
  {
    if (_project->building.levels[_level_idx].models[i].uuid == _uuid)
    {
      _project->building.levels[_level_idx].models.erase(
        _project->building.levels[_level_idx].models.begin() + i
      );
      return;
    }
  }
}


void AddModelCommand::redo()
{
  _uuid = _project->building.add_model(
    _level_idx,
    _x,
    _y,
    0.0,
    M_PI / 2.0,
    _name);
}