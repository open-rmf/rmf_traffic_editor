#ifndef _ADD_VERTEX_H_
#define _ADD_VERTEX_H_

#include <QUndoCommand>
#include "editor_mode_id.h"
#include "project.h"

class AddVertexCommand : public QUndoCommand 
{

public:
  AddVertexCommand(Project* project, EditorModeId mode, int level_idx, double x,
   double y);
  virtual ~AddVertexCommand() {}
  void undo() override;
  void redo() override;
private:
  Project* _project;
  EditorModeId _mode;
  double _x, _y;
  int _level_idx; 
  QUuid _vert_id;
};

#endif