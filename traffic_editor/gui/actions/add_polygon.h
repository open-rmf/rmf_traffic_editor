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
#ifndef _ADD_POLYGON_H_
#define _ADD_POLYGON_H_

#include <QUndoCommand>
#include "editor_mode_id.h"
#include "project.h"

class AddPolygonCommand : public QUndoCommand
{

public:
  AddPolygonCommand(
    Project* project,
    EditorModeId mode,
    Polygon polygon,
    int level_idx);
  virtual ~AddPolygonCommand();
  void undo() override;
  void redo() override;
private:
  Project* _project;
  EditorModeId _mode;
  Polygon _to_add;
  int _level_idx;
  std::vector<Polygon> _previous_polygons;
};

#endif