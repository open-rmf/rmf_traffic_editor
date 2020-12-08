#ifndef _ADD_FIDUCIAL_H_
#define _ADD_FIDUCIAL_H_

#include <QUndoCommand>
#include "project.h"

class AddFiducialCommand : public QUndoCommand
{

public:
  AddFiducialCommand(
    Project* project,
    int level_idx,
    double x,
    double y);
  virtual ~AddFiducialCommand();
  void undo() override;
  void redo() override;
private:
  Project* _project;
  double _x, _y;
  int _level_idx;
  QUuid _uuid;
};

#endif