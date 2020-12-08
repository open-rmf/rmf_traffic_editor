#include "add_fiducial.h"

AddFiducialCommand::AddFiducialCommand(
  Project* project,
  int level_idx,
  double x,
  double y)
{
  _project = project;
  _x = x;
  _y = y;
  _level_idx = level_idx;
}

AddFiducialCommand::~AddFiducialCommand()
{

}

void AddFiducialCommand::undo()
{
  int index_to_remove = -1;

  for (int i = 0; i < _project->building.levels[_level_idx].fiducials.size();
    i++)
  {
    if (_uuid == _project->building.levels[_level_idx].fiducials[i].uuid)
    {
      index_to_remove = i;
    }
  }
  if (index_to_remove < 0)
  {
    //something wrong
    return;
  }

  _project->building.levels[_level_idx].fiducials.erase(
    _project->building.levels[_level_idx].fiducials.begin() + index_to_remove
  );
}

void AddFiducialCommand::redo()
{
  _uuid = _project->building.add_fiducial(_level_idx, _x, _y);
}