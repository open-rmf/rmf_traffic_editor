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

#ifndef MOVE_VERTEX_H
#define MOVE_VERTEX_H

#include <QUndoCommand>
#include "project.h"
#include "traffic_editor/vertex.h"

class MoveVertexCommand : public QUndoCommand
{
public:
  bool has_moved;
  MoveVertexCommand(Project* project, int level_idx, int mouse_vertex_idx);
  virtual ~MoveVertexCommand();

  void set_final_destination(double x, double y);
  void undo() override;
  void redo() override;

private:
  Project* _project;
  int _level_idx;
  Vertex _to_move;
  double _x, _y;
};

#endif