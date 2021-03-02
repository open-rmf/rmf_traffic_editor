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
#include "editor_mode_id.h"
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

  bool export_correspondence_points(
    int level_index,
    const std::string& dest_filename) const;

  void clear();

  void clear_scene();

  void draw(
    QGraphicsScene* scene,
    const int level_idx,
    std::vector<EditorModel>& editor_models);

  void clear_selection(const int level_idx);
  bool can_delete_current_selection(const int level_idx);
  bool delete_selected(const int level_idx);

  void get_selected_items(const int level_idx,
    std::vector<BuildingLevel::SelectedItem>& selected);

  void set_selected_containing_polygon(
    const EditorModeId mode,
    const int level_idx,
    const double x,
    const double y);

  void mouse_select_press(
    const EditorModeId mode,
    const int level_idx,
    const int layer_index,
    const double x,
    const double y,
    QGraphicsItem* graphics_item);

  Polygon::EdgeDragPolygon polygon_edge_drag_press(
    const EditorModeId mode,
    const int level_idx,
    const Polygon* polygon,
    const double x,
    const double y);

  Polygon* get_selected_polygon(const EditorModeId mode, const int level_idx);

  void add_lane(
    const int level_idx,
    const int start_idx,
    const int end_idx);

  RenderingOptions rendering_options;

  std::string get_filename() { return filename; }

private:
  bool load_yaml_file(const std::string& _filename);
  bool save_yaml_file() const;

  void set_selected_line_item(
    const int level_idx,
    QGraphicsLineItem* line_item,
    const EditorModeId mode);

  std::string filename;
};

#endif
