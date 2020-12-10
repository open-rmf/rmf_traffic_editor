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

#include "add_fiducial.h"

AddFiducialCommand::AddFiducialCommand(
  Project* project,
  int level_idx,
  double x,
  double y)
{
  _project = project;
  _x = x;
  _y = y;
  _level_idx = level_idx;
}

AddFiducialCommand::~AddFiducialCommand()
{

}

void AddFiducialCommand::undo()
{
  int index_to_remove = -1;

  for (size_t i = 0; i < _project->building.levels[_level_idx].fiducials.size();
    i++)
  {
    if (_uuid == _project->building.levels[_level_idx].fiducials[i].uuid)
    {
      index_to_remove = i;
    }
  }
  if (index_to_remove < 0)
  {
    //something wrong
    return;
  }

  _project->building.levels[_level_idx].fiducials.erase(
    _project->building.levels[_level_idx].fiducials.begin() + index_to_remove
  );
}

void AddFiducialCommand::redo()
{
  _uuid = _project->building.add_fiducial(_level_idx, _x, _y);
}