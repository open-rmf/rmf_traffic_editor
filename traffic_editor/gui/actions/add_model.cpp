#include "add_model.h"
#include <math.h>

AddModelCommand::AddModelCommand(
  Project* project,
  int level_idx,
  double x,
  double y,
  std::string name)
{
  _project = project;
  _level_idx = level_idx;
  _x = x;
  _y = y;
  _name = name;
}

AddModelCommand::~AddModelCommand()
{

}

void AddModelCommand::undo()
{
  for (size_t i = 0; i < _project->building.levels[_level_idx].models.size();
    i++)
  {
    if (_project->building.levels[_level_idx].models[i].uuid == _uuid)
    {
      _project->building.levels[_level_idx].models.erase(
        _project->building.levels[_level_idx].models.begin() + i
      );
      return;
    }
  }
}


void AddModelCommand::redo()
{
  _uuid = _project->building.add_model(
    _level_idx,
    _x,
    _y,
    0.0,
    M_PI / 2.0,
    _name);
}