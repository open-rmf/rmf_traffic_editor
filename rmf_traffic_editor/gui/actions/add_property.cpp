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
#include "add_property.h"

AddPropertyCommand::AddPropertyCommand(
  Building* building,
  std::string property,
  Param value,
  int level_idx)
{
  _building = building;
  _prop = property;
  _val = value;
  _level_idx = level_idx;
  _vert_id = -1;

  for (size_t i = 0; i < building->levels[level_idx].vertices.size(); i++)
  {
    if (building->levels[level_idx].vertices[i].selected)
      _vert_id = i;
  }
}

AddPropertyCommand::~AddPropertyCommand()
{

}

int AddPropertyCommand::get_vertex_updated()
{
  return _vert_id;
}

void AddPropertyCommand::redo()
{
  if (_vert_id < 0)
    return;

  _building->levels[_level_idx].vertices[_vert_id].params[_prop] = _val;
}

void AddPropertyCommand::undo()
{
  auto v = _building->levels[_level_idx].vertices[_vert_id];
  if (v.params.count(_prop) == 0)
    return;

  _building->levels[_level_idx].vertices[_vert_id].params.erase(_prop);
}
