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

#include "add_constraint.h"

AddConstraintCommand::AddConstraintCommand(
  Building* building,
  int level,
  const QUuid& a,
  const QUuid& b)
: _building(building),
  _layer(layer),
  _id_a(a),
  _id_b(b)
{
}

void AddConstraintCommand::undo()
{
  // TODO
  //_building->remove_feature(_level, _layer, _uuid);
}

void AddConstraintCommand::redo()
{
  // TODO
  //_uuid = _building->add_feature(_level, _layer, _x, _y);
}

