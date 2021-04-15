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

#include "add_constraint.hpp"

AddConstraintCommand::AddConstraintCommand(
  Building* building,
  const int level_idx,
  const QUuid& a,
  const QUuid& b)
: _building(building),
  _level_idx(level_idx),
  _id_a(a),
  _id_b(b)
{
}

AddConstraintCommand::~AddConstraintCommand()
{
}

void AddConstraintCommand::undo()
{
  _building->remove_constraint(_level_idx, _id_a, _id_b);
}

void AddConstraintCommand::redo()
{
  _building->add_constraint(_level_idx, _id_a, _id_b);
}

