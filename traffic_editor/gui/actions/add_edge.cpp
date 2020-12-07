#include "add_edge.h"

AddEdgeCommand::AddEdgeCommand(Project* project, int level_idx)
{
  _project = project;
  _level_idx = level_idx;
  _first_point_not_exist = false;
  _first_point_drawn = false;
  _second_point_not_exist = false;
  _second_point_drawn = false;
}

AddEdgeCommand::~AddEdgeCommand()
{

}

void AddEdgeCommand::redo()
{
  if (!_first_point_drawn)
  {
    _project->building.add_vertex(_level_idx, _first_x, _first_y);
    _vert_id_first =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
  }

  if (!_second_point_drawn)
  {
    _project->building.add_vertex(_level_idx, _second_x, _second_y);
    _vert_id_second =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
  }

  size_t prev_idx = _project->building.levels[_level_idx].get_vertex_by_id(
    _vert_id_first);
  size_t curr_idx = _project->building.levels[_level_idx].get_vertex_by_id(
    _vert_id_second);

  //Hackjob to fix wierd redo behaviour
  if (prev_idx > _project->building.levels[_level_idx].vertices.size())
  {
    _project->building.add_vertex(_level_idx, _first_x, _first_y);
    _vert_id_first =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
    prev_idx = _project->building.levels[_level_idx].vertices.size()-1;
  }

  if (curr_idx > _project->building.levels[_level_idx].vertices.size())
  {
    _project->building.add_vertex(_level_idx, _first_x, _first_y);
    _vert_id_second =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
    curr_idx = _project->building.levels[_level_idx].vertices.size()-1;
  }

  if (_type != Edge::LANE)
  {
    _project->building.add_edge(
      _level_idx,
      prev_idx,
      curr_idx,
      _type);
  }
  else
  {
    _project->add_lane(
      _level_idx,
      prev_idx,
      curr_idx);
  }
}

void AddEdgeCommand::undo()
{
  int first_idx = _project->building.levels[_level_idx].get_vertex_by_id(
    _vert_id_first);
  int second_idx = _project->building.levels[_level_idx].get_vertex_by_id(
    _vert_id_second);

  size_t to_be_removed = _project->building.levels[_level_idx].edges.size()+1;


  for (size_t i = 0; i < _project->building.levels[_level_idx].edges.size();
    i++)
  {
    if (
      _project->building.levels[_level_idx].edges[i].start_idx == first_idx
      && _project->building.levels[_level_idx].edges[i].end_idx == second_idx
      && _project->building.levels[_level_idx].edges[i].type == _type)
    {
      to_be_removed = i;
    }
  }
  if (to_be_removed > _project->building.levels[_level_idx].edges.size())
  {
    //Something must have gone wrong

    return;
  }
  _project->building.levels[_level_idx].edges.erase(_project->building.levels[
      _level_idx].edges.begin() + to_be_removed);


  if (_first_point_not_exist)
  {
    //delete the vertex as it was added while drawing
    _project->building.levels[_level_idx].vertices.erase(
      _project->building.levels[_level_idx].vertices.begin() + first_idx);
    _first_point_drawn = false;
  }
  if (_second_point_not_exist)
  {
    //delete the vertex as it was added while drawing
    //If the previous vertex has been deleted then the index of the second vertex would change
    second_idx = _project->building.levels[_level_idx].get_vertex_by_id(
      _vert_id_second);
    _project->building.levels[_level_idx].vertices.erase(
      _project->building.levels[_level_idx].vertices.begin() + second_idx);
    _second_point_drawn = false;
  }
}

int AddEdgeCommand::set_first_point(double x, double y)
{
  _first_x = x;
  _first_y = y;

  int clicked_idx = _project->building.nearest_item_index_if_within_distance(
    _level_idx, x, y, 10.0, Building::VERTEX);

  _first_point_drawn = true;
  if (clicked_idx < 0)
  {
    _first_point_not_exist = true;
    _project->building.add_vertex(_level_idx, x, y);
    _vert_id_first =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
    clicked_idx = _project->building.levels[_level_idx].vertices.size()-1;
  }
  else
  {
    _vert_id_first =
      _project->building.levels[_level_idx].vertices[clicked_idx].uuid;
  }
  return clicked_idx;
}

int AddEdgeCommand::set_second_point(double x, double y)
{
  _second_x = x;
  _second_y = y;

  int clicked_idx = _project->building.nearest_item_index_if_within_distance(
    _level_idx, x, y, 10.0, Building::VERTEX);

  _second_point_drawn = true;

  if (clicked_idx < 0)
  {
    _second_point_not_exist = true;
    _second_point_drawn = true;
    _project->building.add_vertex(_level_idx, x, y);
    _vert_id_second =
      _project->building.levels[_level_idx].vertices.rbegin()->uuid;
    clicked_idx = _project->building.levels[_level_idx].vertices.size()-1;
  }
  else
  {
    _vert_id_second =
      _project->building.levels[_level_idx].vertices[clicked_idx].uuid;
  }
  return clicked_idx;
}

void AddEdgeCommand::set_edge_type(Edge::Type type)
{
  _type = type;
}