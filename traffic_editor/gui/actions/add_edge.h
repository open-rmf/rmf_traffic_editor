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

#ifndef _ADD_EDGE_H_
#define _ADD_EDGE_H_

#include <QUndoCommand>
#include "project.h"

class AddEdgeCommand : public QUndoCommand
{

public:
  AddEdgeCommand(Project* project, int level_idx);
  virtual ~AddEdgeCommand();
  void undo() override;
  void redo() override;
  int set_first_point(double x, double y);
  int set_second_point(double x, double y);
  void set_edge_type(Edge::Type type);
private:
  Project* _project;
  double _first_x, _first_y;
  double _second_x, _second_y;
  bool _first_point_not_exist, _first_point_drawn;
  bool _second_point_not_exist, _second_point_drawn;
  int _level_idx;
  std::vector<Edge> _edge_snapshot;
  std::vector<Vertex> _vert_snapshot, _final_snapshot;
  int _vert_id_first, _vert_id_second;
  Edge::Type _type;
};


#endif