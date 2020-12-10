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
#ifndef _ADD_MODEL_H_
#define _ADD_MODEL_H_

#include <QUndoCommand>
#include "project.h"

class AddModelCommand : public QUndoCommand
{

public:
  AddModelCommand(
    Project* project,
    int level_idx,
    double x,
    double y,
    std::string name);
  virtual ~AddModelCommand();
  void undo() override;
  void redo() override;
private:
  Project* _project;
  double _x, _y;
  int _level_idx;
  QUuid _uuid;
  std::string _name;
};

#endif