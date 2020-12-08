#ifndef _MOVE_FIDUCIAL_H_
#define _MOVE_FIDUCIAL_H_

#include <QUndoCommand>
#include "project.h"

class MoveFiducialCommand : public QUndoCommand
{
public:
  MoveFiducialCommand(
    Project* project,
    int level,
    int fiducial_id
  );
  virtual ~MoveFiducialCommand();

  void undo() override;
  void redo() override;

  void set_final_destination(double x, double y);

  bool has_moved;
private:
  double _original_x, _original_y;
  double _final_x, _final_y;
  int _level_id, _fiducial_id;
  Project* _project;
};

#endif