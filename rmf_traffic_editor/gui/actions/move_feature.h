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

#ifndef ACTIONS__MOVE_FEATURE_H_
#define ACTIONS__MOVE_FEATURE_H_

#include <QUndoCommand>
#include <QUuid>

#include "building.h"

class MoveFeatureCommand : public QUndoCommand
{
public:
  MoveFeatureCommand(
    Building* building,
    int level_idx,
    int layer_idx,
    int feature_idx);

  void undo() override;
  void redo() override;

  void set_final_destination(double x, double y);

  bool has_moved;

private:
  Building* _building;
  int _level_idx, _layer_idx, _feature_idx;
  double _original_x, _original_y;
  double _final_x, _final_y;
};

#endif  // ACTIONS__MOVE_FEATURE_H_
