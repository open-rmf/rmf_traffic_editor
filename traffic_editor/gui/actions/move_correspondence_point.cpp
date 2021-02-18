/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "move_correspondence_point.hpp"

MoveCorrespondencePointCommand::MoveCorrespondencePointCommand(
  Project* project,
  int level,
  int layer,
  int point_id)
: has_moved(false),
  project_(project),
  level_(level),
  layer_(layer),
  point_id_(point_id)
{
  CorrespondencePoint correspondence_point =
    project_->building.levels[level_].
    correspondence_point_sets()[layer_][point_id_];
  final_x_ = original_x_ = correspondence_point.x();
  final_y_ = original_y_ = correspondence_point.y();
}

void MoveCorrespondencePointCommand::undo()
{
  CorrespondencePoint& correspondence_point =
    project_->building.levels[level_].
    correspondence_point_sets()[layer_][point_id_];
  correspondence_point.set_x(original_x_);
  correspondence_point.set_y(original_y_);
}

void MoveCorrespondencePointCommand::redo()
{
  CorrespondencePoint& correspondence_point =
    project_->building.levels[level_].
    correspondence_point_sets()[layer_][point_id_];
  correspondence_point.set_x(final_x_);
  correspondence_point.set_y(final_y_);
}

void MoveCorrespondencePointCommand::set_final_destination(double x, double y)
{
  final_x_ = x;
  final_y_ = y;
  has_moved = true;
}
