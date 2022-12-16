/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "move_feature.h"

MoveFeatureCommand::MoveFeatureCommand(
  Building* building,
  int level_idx,
  int layer_idx,
  int feature_idx)
: has_moved(false),
  _building(building),
  _level_idx(level_idx),
  _layer_idx(layer_idx),
  _feature_idx(feature_idx)
{
  Feature* f = nullptr;
  if (layer_idx == 0)
  {
    f = &_building->levels[_level_idx].floorplan_features[_feature_idx];
  }
  else
  {
    f = &_building->levels[_level_idx].
      layers[_layer_idx-1].features[_feature_idx];
  }
  _final_x = _original_x = f->x();
  _final_y = _original_y = f->y();
}

void MoveFeatureCommand::undo()
{
  Feature* f = nullptr;
  if (_layer_idx == 0)
  {
    f = &_building->levels[_level_idx].floorplan_features[_feature_idx];
  }
  else
  {
    f = &_building->levels[_level_idx].
      layers[_layer_idx-1].features[_feature_idx];
  }
  f->set_x(_original_x);
  f->set_y(_original_y);
}

void MoveFeatureCommand::redo()
{
  Feature* f = nullptr;
  if (_layer_idx == 0)
  {
    f = &_building->levels[_level_idx].floorplan_features[_feature_idx];
  }
  else
  {
    f = &_building->levels[_level_idx].
      layers[_layer_idx-1].features[_feature_idx];
  }
  f->set_x(_final_x);
  f->set_y(_final_y);
}

void MoveFeatureCommand::set_final_destination(double x, double y)
{
  _final_x = x;
  _final_y = y;
  has_moved = true;
}
