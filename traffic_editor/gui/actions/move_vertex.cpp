#include "move_vertex.h"

MoveVertexCommand::MoveVertexCommand(Project* project, int level_idx, int mouse_vertex_idx)
{
  _to_move = project->building.levels[level_idx].vertices[mouse_vertex_idx];
  _level_idx = level_idx;
  _project = project;
  has_moved = false;
}

MoveVertexCommand::~MoveVertexCommand()
{

}

void MoveVertexCommand::set_final_destination(double x, double y)
{
  _x = x;
  _y = y;
  has_moved = true;
}

void MoveVertexCommand::undo()
{
  //Use ID because in future if we want to support photoshop style selective 
  //undo-redos it will be consistent even after deletion of intermediate vertices.
  for(Vertex& vert: _project->building.levels[_level_idx].vertices) {
    if(vert.uuid == _to_move.uuid) {
      vert.x = _to_move.x;
      vert.y = _to_move.y;
    }
  }
}

void MoveVertexCommand::redo()
{
  //Use ID because in future if we want to support photoshop style selective 
  //undo-redos it will be consistent even after deletion of intermediate vertices.
  for(Vertex& vert: _project->building.levels[_level_idx].vertices) {
    if(vert.uuid == _to_move.uuid) {
      vert.x = _x;
      vert.y = _y;
    }
  }
}