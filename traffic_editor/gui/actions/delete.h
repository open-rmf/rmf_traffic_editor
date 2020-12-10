#ifndef _DELETE_H_
#define _DELETE_H_

#include <QUndoCommand>
#include "project.h"

class DeleteCommand : public QUndoCommand
{
public:
  DeleteCommand(Project* project, int level_idx);
  virtual ~DeleteCommand();
  void undo() override;
  void redo() override;

private:
  std::vector<Vertex> _vertices;
  std::vector<int> _vertex_idx;
  std::vector<Edge> _edges;
  std::vector<int> _edge_idx;
  std::vector<Model> _models;
  std::vector<int> _model_idx;
  std::vector<Fiducial> _fiducials;
  std::vector<int> _fiducial_idx;
  std::vector<Polygon> _polygons;
  std::vector<int> _polygon_idx;

  Project* _project;
  int _level_idx;
};

#endif
