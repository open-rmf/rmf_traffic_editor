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

#include "move_model.h"

MoveModelCommand::MoveModelCommand(
  Building* building,
  int level,
  int model_id
)
{
  _building = building;
  Model model = _building->levels[level].models[model_id];
  _original_x = model.state.x;
  _original_y = model.state.y;
  _level_id = level;
  _model_id = model_id;
  has_moved = false;
}

MoveModelCommand::~MoveModelCommand()
{

}

void MoveModelCommand::undo()
{
  Model& model = _building->levels[_level_id].models[_model_id];
  model.state.x = _original_x;
  model.state.y = _original_y;
}

void MoveModelCommand::redo()
{
  Model& model = _building->levels[_level_id].models[_model_id];
  model.state.x = _final_x;
  model.state.y = _final_y;
}

void MoveModelCommand::set_final_destination(double x, double y)
{
  _final_x = x;
  _final_y = y;
  has_moved = true;
}
