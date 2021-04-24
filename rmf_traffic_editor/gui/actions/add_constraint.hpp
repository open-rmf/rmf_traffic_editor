/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#ifndef _ADD_CONSTRAINT_H_
#define _ADD_CONSTRAINT_H_

#include <QUndoCommand>
#include <QUuid>

#include "building.h"

class AddConstraintCommand : public QUndoCommand
{

public:
  AddConstraintCommand(
    Building* building,
    const int level_idx,
    const QUuid& a,
    const QUuid& b);
  virtual ~AddConstraintCommand();
  void undo() override;
  void redo() override;

private:
  Building* _building;
  int _level_idx;
  QUuid _id_a;
  QUuid _id_b;
  std::vector<Constraint> _constraints_snapshot;
};


#endif
