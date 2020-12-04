#ifndef MOVE_VERTEX_H
#define MOVE_VERTEX_H

#include <QUndoCommand>
#include "editor_mode_id.h"
#include "../project.h"
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