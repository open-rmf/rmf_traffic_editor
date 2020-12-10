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

#include "rotate_model.h"

RotateModelCommand::RotateModelCommand(
  Project* project,
  int level,
  int model_id)
{
  has_moved = false;
  _project = project;
  _level_id = level;
  _model_id = model_id;
  _original_yaw =
    _project->building.levels[_level_id].models[_model_id].state.yaw;
}

RotateModelCommand::~RotateModelCommand()
{
}

void RotateModelCommand::undo()
{
  _project->building.set_model_yaw(_level_id, _model_id, _original_yaw);
}

void RotateModelCommand::redo()
{
  _project->building.set_model_yaw(_level_id, _model_id, _final_yaw);
}

void RotateModelCommand::set_final_destination(double yaw)
{
  has_moved = true;
  _final_yaw = yaw;
}