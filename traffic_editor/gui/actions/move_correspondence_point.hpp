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

#ifndef ACTIONS__MOVE_CORRESPONDENCE_POING_HPP_
#define ACTIONS__MOVE_CORRESPONDENCE_POING_HPP_

#include <QUndoCommand>
#include <QUuid>

#include "project.h"

class MoveCorrespondencePointCommand : public QUndoCommand
{
public:
  MoveCorrespondencePointCommand(Project* project, int id, int level, int layer);

  void undo() override;
  void redo() override;

  void set_final_destination(double x, double y);

  bool has_moved;

private:
  Project* project_;
  int level_, layer_, point_id_;
  double original_x_, original_y_;
  double final_x_, final_y_;
};

#endif  // ACTIONS__MOVE_CORRESPONDENCE_POING_HPP_
