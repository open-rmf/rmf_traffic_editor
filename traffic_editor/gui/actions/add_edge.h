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
  QUuid _vert_id_first, _vert_id_second;
  Edge::Type _type;
};


#endif