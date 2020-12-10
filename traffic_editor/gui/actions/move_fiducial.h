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

#ifndef _MOVE_FIDUCIAL_H_
#define _MOVE_FIDUCIAL_H_

#include <QUndoCommand>
#include "project.h"

class MoveFiducialCommand : public QUndoCommand
{
public:
  MoveFiducialCommand(
    Project* project,
    int level,
    int fiducial_id
  );
  virtual ~MoveFiducialCommand();

  void undo() override;
  void redo() override;

  void set_final_destination(double x, double y);

  bool has_moved;
private:
  double _original_x, _original_y;
  double _final_x, _final_y;
  int _level_id, _fiducial_id;
  Project* _project;
};

#endif