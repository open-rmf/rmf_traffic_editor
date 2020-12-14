#include "polygon_remove_vertices.h"
PolygonRemoveVertCommand::PolygonRemoveVertCommand(
  Polygon* polygon,
  int vert_id)
{
  _polygon = polygon;
  _vert_id = vert_id;
  _old_vertices = polygon->vertices;
}

PolygonRemoveVertCommand::~PolygonRemoveVertCommand()
{
}

void PolygonRemoveVertCommand::undo()
{
  _polygon->vertices = _old_vertices;
}

void PolygonRemoveVertCommand::redo()
{
  _polygon->remove_vertex(_vert_id);
}
