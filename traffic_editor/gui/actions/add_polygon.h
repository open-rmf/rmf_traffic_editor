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