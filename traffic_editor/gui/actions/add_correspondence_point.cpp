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

#include "add_correspondence_point.hpp"

AddCorrespondencePointCommand::AddCorrespondencePointCommand(
  Project* project,
  int level,
  int layer,
  double x,
  double y)
: project_(project),
  level_(level),
  layer_(layer),
  x_(x),
  y_(y)
{
}

void AddCorrespondencePointCommand::undo()
{
  int index_to_remove = -1;

  for (
    size_t ii = 0;
    ii < project_->building.levels[level_].correspondence_point_sets()[layer_].size();
    ++ii)
  {
    if (uuid_ ==
        project_->building.levels[level_].correspondence_point_sets()[layer_][ii].uuid())
    {
      index_to_remove = ii;
    }
  }
  if (index_to_remove < 0) {
    return;
  }

  project_->building.levels[level_].correspondence_point_sets()[layer_].erase(
    project_->building.levels[level_].correspondence_point_sets()[layer_].begin() +
      index_to_remove);
}

void AddCorrespondencePointCommand::redo()
{
  uuid_ = project_->building.add_correspondence_point(level_, layer_, x_, y_);
}

