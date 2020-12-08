#include "move_fiducial.h"

MoveFiducialCommand::MoveFiducialCommand(
  Project* project,
  int level,
  int fiducial_id)
{
  _project = project;
  Fiducial fiducial = project->building.levels[level].fiducials[fiducial_id];
  _original_x = fiducial.x;
  _original_y = fiducial.y;
  _level_id = level;
  _fiducial_id = fiducial_id;
  has_moved = false;
}

MoveFiducialCommand::~MoveFiducialCommand()
{
}

void MoveFiducialCommand::undo()
{
  Fiducial& fiducial =
    _project->building.levels[_level_id].fiducials[_fiducial_id];
  fiducial.x = _original_x;
  fiducial.y = _original_y;
}

void MoveFiducialCommand::redo()
{
  Fiducial& fiducial =
    _project->building.levels[_level_id].fiducials[_fiducial_id];
  fiducial.x = _final_x;
  fiducial.y = _final_y;
}

void MoveFiducialCommand::set_final_destination(double x, double y)
{
  _final_x = x;
  _final_y = y;
  has_moved = true;
}