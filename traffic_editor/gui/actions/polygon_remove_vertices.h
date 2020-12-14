#ifndef _POLYGON_REMOVE_H_
#define _POLYGON_REMOVE_H_

#include <QUndoCommand>
#include "editor_mode_id.h"
#include "project.h"

class PolygonRemoveVertCommand : public QUndoCommand
{

public:
  PolygonRemoveVertCommand(
    Polygon* polygon,
    int vert_id);
  virtual ~PolygonRemoveVertCommand();
  void undo() override;
  void redo() override;
private:
  Polygon* _polygon;
  int _vert_id;
  std::vector<int> _old_vertices;
};

#endif