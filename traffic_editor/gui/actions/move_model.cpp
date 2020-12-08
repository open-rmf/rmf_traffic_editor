#include "move_model.h"

MoveModelCommand::MoveModelCommand(
    Project* project,
    int level,
    int model_id
  )
{
  _project = project;
  Model model = project->building.levels[level].models[model_id];
  _original_x = model.state.x;
  _original_y = model.state.y;
  _level_id = level;
  _model_id = model_id;
  has_moved = false;
}

MoveModelCommand::~MoveModelCommand()
{

}

void MoveModelCommand::undo()
{
  Model& model = _project->building.levels[_level_id].models[_model_id];
  model.state.x = _original_x;
  model.state.y = _original_y;
}

void MoveModelCommand::redo()
{
  Model& model = _project->building.levels[_level_id].models[_model_id];
  model.state.x = _final_x;
  model.state.y = _final_y;
}

void MoveModelCommand::set_final_destination(double x, double y)
{
  _final_x = x;
  _final_y = y;
  has_moved = true;
}
