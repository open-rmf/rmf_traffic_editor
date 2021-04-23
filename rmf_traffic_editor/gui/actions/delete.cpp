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

DeleteCommand::DeleteCommand(Building* building, int level_idx)
{
  _building = building;
  _level_idx = level_idx;
}

DeleteCommand::~DeleteCommand()
{

}

void DeleteCommand::undo()
{
  for (size_t i = 0; i < _vertices.size(); i++)
  {
    _building->levels[_level_idx].vertices.insert(
      _building->levels[_level_idx].vertices.begin() + _vertex_idx[i],
      _vertices[i]);
  }

  for (size_t i = 0; i < _edges.size(); i++)
  {
    _building->levels[_level_idx].edges.insert(
      _building->levels[_level_idx].edges.begin() + _edge_idx[i],
      _edges[i]);
  }

  for (size_t i = 0; i < _models.size(); i++)
  {
    _building->levels[_level_idx].models.insert(
      _building->levels[_level_idx].models.begin() + _model_idx[i],
      _models[i]);
  }

  for (size_t i = 0; i < _fiducials.size(); i++)
  {
    _building->levels[_level_idx].fiducials.insert(
      _building->levels[_level_idx].fiducials.begin() + _fiducial_idx[i],
      _fiducials[i]);
  }

  for (size_t i = 0; i < _polygons.size(); i++)
  {
    _building->levels[_level_idx].polygons.insert(
      _building->levels[_level_idx].polygons.begin() + _polygon_idx[i],
      _polygons[i]);
  }

  for (size_t i = 0; i < _features.size(); i++)
  {
    Level& level = _building->levels[_level_idx];
    if (_feature_layer_idx[i] == 0)
    {
      level.floorplan_features.insert(
        level.floorplan_features.begin() + _feature_idx[i],
        _features[i]);
    }
    else
    {
      Layer& layer = level.layers[_feature_layer_idx[i] - 1];
      layer.features.insert(
        layer.features.begin() + _feature_idx[i],
        _features[i]);
    }
  }

  for (size_t i = 0; i < _constraints.size(); i++)
  {
    Level& level = _building->levels[_level_idx];
    level.constraints.insert(
      level.constraints.begin() + _constraint_idx[i],
      _constraints[i]);
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

  _features.clear();
  _feature_layer_idx.clear();
  _feature_idx.clear();

  _constraints.clear();
  _constraint_idx.clear();
}

void DeleteCommand::redo()
{
  std::vector<Level::SelectedItem> selected_items;
  _building->get_selected_items(_level_idx, selected_items);

  for (auto& item: selected_items)
  {
    if (item.model_idx >= 0)
    {
      _models.push_back(_building->levels[_level_idx].models[item.model_idx]);
      _model_idx.push_back(item.model_idx);
    }

    if (item.vertex_idx >= 0)
    {
      _vertices.push_back(
        _building->levels[_level_idx].vertices[item.vertex_idx]
      );
      _vertex_idx.push_back(item.vertex_idx);
    }

    if (item.fiducial_idx >= 0)
    {
      _fiducials.push_back(
        _building->levels[_level_idx].fiducials[item.fiducial_idx]
      );
      _fiducial_idx.push_back(item.fiducial_idx);
    }

    if (item.edge_idx >= 0)
    {
      _edges.push_back(_building->levels[_level_idx].edges[item.edge_idx]);
      _edge_idx.push_back(item.edge_idx);
    }

    if (item.polygon_idx >= 0)
    {
      _polygons.push_back(
        _building->levels[_level_idx].polygons[item.polygon_idx]
      );
      _polygon_idx.push_back(item.edge_idx); // (MQ) should be polygon_idx ?
    }

    if (item.feature_idx >= 0)
    {
      _feature_layer_idx.push_back(item.feature_layer_idx);
      _feature_idx.push_back(item.feature_idx);
      if (item.feature_layer_idx == 0)
      {
        _features.push_back(
          _building->levels[_level_idx].floorplan_features[item.feature_idx]);
      }
      else
      {
        _features.push_back(
          _building->levels[_level_idx].layers[item.feature_layer_idx-1].
          features[item.feature_idx]);
      }
    }

    if (item.constraint_idx >= 0)
    {
      _constraints.push_back(
        _building->levels[_level_idx].constraints[item.constraint_idx]);
      _constraint_idx.push_back(item.constraint_idx);
    }
  }
  _building->delete_selected(_level_idx);
}
