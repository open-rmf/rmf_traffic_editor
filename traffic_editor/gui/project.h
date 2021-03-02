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

#ifndef PROJECT_H
#define PROJECT_H

#include "building.h"
#include "editor_model.h"
#include "traffic_map.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class QGraphicsScene;
class QGraphicsItem;
class QGraphicsLineItem;


class Project
{
public:
  std::string name;

  Building building;
  std::vector<TrafficMap> traffic_maps;

  /////////////////////////////////
  Project();
  ~Project();

  void clear();

  void clear_scene();

/*
  void set_selected_containing_polygon(
    const EditorModeId mode,
    const int level_idx,
    const double x,
    const double y);

  Polygon::EdgeDragPolygon polygon_edge_drag_press(
    const EditorModeId mode,
    const int level_idx,
    const Polygon* polygon,
    const double x,
    const double y);
*/

  void add_lane(
    const int level_idx,
    const int start_idx,
    const int end_idx);

  RenderingOptions rendering_options;

  std::string get_filename() { return filename; }

private:
  bool load_yaml_file(const std::string& _filename);
  bool save_yaml_file() const;

  std::string filename;
};

#endif
