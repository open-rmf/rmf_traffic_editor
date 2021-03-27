/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#include "add_vertex.h"

AddVertexCommand::AddVertexCommand(
  Building* building,
  int level_idx,
  double x,
  double y)
{
  _x = x;
  _y = y;
  _level_idx = level_idx; //TODO: Dependency on level_idx is dangerous.
  _building = building;
}

AddVertexCommand::~AddVertexCommand()
{
  //Empty to make linker happy
}

void AddVertexCommand::undo()
{
  size_t length = _building->levels[_level_idx].vertices.size();
  //TODO: SLOW O(n) method... Need to rework datastructures.
  for (size_t i = 0; i < length; i++)
  {
    if (_building->levels[_level_idx].vertices[i].uuid == _vert_id)
    {
      _building->levels[_level_idx].vertices.erase(
        _building->levels[_level_idx].vertices.begin() + i);
      break;
    }
  }
}

void AddVertexCommand::redo()
{
  _building->add_vertex(_level_idx, _x, _y);
  size_t sz = _building->levels[_level_idx].vertices.size();
  _vert_id = _building->levels[_level_idx].vertices[sz - 1].uuid;
}
