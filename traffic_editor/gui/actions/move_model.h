#ifndef _MOVE_MODEL_H_
#define _MOVE_MODEL_H_

#include <QUndoCommand>
#include "project.h"

class MoveModelCommand : public QUndoCommand
{
public:
  MoveModelCommand(
    Project* project,
    int level,
    int model_id
  );
  virtual ~MoveModelCommand();

  void undo() override;
  void redo() override;

  void set_final_destination(double x, double y);

  bool has_moved;
private:
  double _original_x, _original_y;
  double _final_x, _final_y;
  int _level_id, _model_id;
  Project* _project;
};

#endif