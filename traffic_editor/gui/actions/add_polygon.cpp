#include "add_polygon.h"

AddPolygonCommand::AddPolygonCommand(
  Project* project,
  EditorModeId mode,
  Polygon polygon,
  int level_idx)
{
  //In this case to keep polygon undoing simple we use simple snapshots;
  _project = project;
  _mode = mode;
  _to_add = polygon;
  _level_idx = level_idx;
  if(_mode == MODE_BUILDING)
    _previous_polygons = _project->building.levels[level_idx].polygons;
  else if (mode == MODE_SCENARIO)
    _previous_polygons = _project->scenario_level(level_idx)->polygons;
}

AddPolygonCommand::~AddPolygonCommand()
{
}

void AddPolygonCommand::undo()
{
  if(_mode == MODE_BUILDING)
    _project->building.levels[_level_idx].polygons = _previous_polygons;
  else if (_mode == MODE_SCENARIO)
    _project->scenario_level(_level_idx)->polygons = _previous_polygons;
}

void AddPolygonCommand::redo()
{
  if (_mode == MODE_BUILDING)
    _project->building.levels[_level_idx].polygons.push_back(_to_add);
  else if (_mode == MODE_SCENARIO)
    _project->scenario_level(_level_idx)->polygons.push_back(_to_add);
}