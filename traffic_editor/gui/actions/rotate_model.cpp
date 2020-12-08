#include "rotate_model.h"

RotateModelCommand::RotateModelCommand(
  Project* project,
  int level,
  int model_id)
{
  has_moved = false;
  _project = project;
  _level_id = level;
  _model_id = model_id;
  _original_yaw =
    _project->building.levels[_level_id].models[_model_id].state.yaw;
}

RotateModelCommand::~RotateModelCommand()
{
}

void RotateModelCommand::undo()
{
  _project->building.set_model_yaw(_level_id, _model_id, _original_yaw);
}

void RotateModelCommand::redo()
{
  _project->building.set_model_yaw(_level_id, _model_id, _final_yaw);
}

void RotateModelCommand::set_final_destination(double yaw)
{
  has_moved = true;
  _final_yaw = yaw;
}