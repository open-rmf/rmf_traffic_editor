#ifndef _ROTATE_MODEL_H_
#define _ROTATE_MODEL_H_

#include <QUndoCommand>
#include "project.h"

class RotateModelCommand : public QUndoCommand
{
public:
  RotateModelCommand(
    Project* project,
    int level,
    int model_id
  );
  virtual ~RotateModelCommand();

  void undo() override;
  void redo() override;

  void set_final_destination(double yaw);

  bool has_moved;
private:
  double _original_yaw;
  double _final_yaw;
  int _level_id, _model_id;
  Project* _project;
};


#endif