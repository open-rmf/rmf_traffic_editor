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
#ifndef _ADD_PROPERTY_H_
#define _ADD_PROPERTY_H_

#include <QUndoCommand>
#include "building.h"

class AddPropertyCommand : public QUndoCommand
{
public:
  AddPropertyCommand(Building* building,
    std::string property,
    Param value,
    int level_idx);
  virtual ~AddPropertyCommand();
  int get_vertex_updated();
  void undo() override;
  void redo() override;

private:
  Building* _building;
  std::string _prop;
  Param _val;
  int _vert_id, _level_idx;
};
#endif
