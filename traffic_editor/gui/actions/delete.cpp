/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "delete.h"

DeleteCommand::DeleteCommand(Project* project, int level_idx)
{
  _project = project;
  _level_idx = level_idx;
}

DeleteCommand::~DeleteCommand()
{

}

void DeleteCommand::undo()
{
  for (size_t i = 0; i < _vertices.size(); i++)
  {
    _project->building.levels[_level_idx].vertices.insert(
      _project->building.levels[_level_idx].vertices.begin() + _vertex_idx[i],
      _vertices[i]);
  }

  for (size_t i = 0; i < _edges.size(); i++)
  {
    _project->building.levels[_level_idx].edges.insert(
      _project->building.levels[_level_idx].edges.begin() + _edge_idx[i],
      _edges[i]);
  }

  for (size_t i = 0; i < _models.size(); i++)
  {
    _project->building.levels[_level_idx].models.insert(
      _project->building.levels[_level_idx].models.begin() + _model_idx[i],
      _models[i]);
  }

  for (size_t i = 0; i < _fiducials.size(); i++)
  {
    _project->building.levels[_level_idx].fiducials.insert(
      _project->building.levels[_level_idx].fiducials.begin() + _fiducial_idx[i],
      _fiducials[i]);
  }

  for (size_t i = 0; i < _polygons.size(); i++)
  {
    _project->building.levels[_level_idx].polygons.insert(
      _project->building.levels[_level_idx].polygons.begin() + _polygon_idx[i],
      _polygons[i]);
  }

  _vertices.clear();
  _vertex_idx.clear();
  _edges.clear();
  _edge_idx.clear();
  _models.clear();
  _model_idx.clear();
  _fiducials.clear();
  _fiducial_idx.clear();
  _polygons.clear();
  _polygon_idx.clear();
}

void DeleteCommand::redo()
{
  std::vector<BuildingLevel::SelectedItem> selected_items;
  _project->get_selected_items(_level_idx, selected_items);

  for (auto& item: selected_items)
  {
    if (item.model_idx >= 0)
    {
      _models.push_back(
        _project->building.levels[_level_idx].models[item.model_idx]
      );
      _model_idx.push_back(item.model_idx);
    }

    if (item.vertex_idx >= 0)
    {
      _vertices.push_back(
        _project->building.levels[_level_idx].vertices[item.vertex_idx]
      );
      _vertex_idx.push_back(item.vertex_idx);
    }

    if (item.fiducial_idx >= 0)
    {
      _fiducials.push_back(
        _project->building.levels[_level_idx].fiducials[item.fiducial_idx]
      );
      _fiducial_idx.push_back(item.fiducial_idx);
    }

    if (item.edge_idx >= 0)
    {
      _edges.push_back(
        _project->building.levels[_level_idx].edges[item.edge_idx]
      );
      _edge_idx.push_back(item.edge_idx);
    }

    if (item.polygon_idx >= 0)
    {
      _polygons.push_back(
        _project->building.levels[_level_idx].polygons[item.polygon_idx]
      );
      _polygon_idx.push_back(item.edge_idx);
    }
  }
  _project->delete_selected(_level_idx);
}