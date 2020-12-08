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