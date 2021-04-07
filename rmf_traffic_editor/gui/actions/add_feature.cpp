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

#include "add_feature.h"

AddFeatureCommand::AddFeatureCommand(
  Building* building,
  int level,
  int layer,
  double x,
  double y)
: _building(building),
  _level(level),
  _layer(layer),
  _x(x),
  _y(y)
{
}

void AddFeatureCommand::undo()
{
  int index_to_remove = -1;
  auto& s = _building->levels[_level].feature_sets()[_layer];

  for (size_t ii = 0; ii < s.size(); ++ii)
  {
    if (_uuid == s[ii].uuid())
      index_to_remove = ii;
  }

  if (index_to_remove < 0)
    return;

  s.erase(s.begin() + index_to_remove);
}

void AddFeatureCommand::redo()
{
  _uuid = _building->add_feature(_level, _layer, _x, _y);
}

