/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#ifndef LAYER_H
#define LAYER_H

#include <string>
#include <vector>

#include <QPixmap>
#include <yaml-cpp/yaml.h>

#include "coordinate_system.h"
#include "feature.hpp"
#include "transform.hpp"

class QGraphicsScene;
class QGraphicsPixmapItem;
class QTableWidget;


class Layer
{
public:
  Layer();
  ~Layer();

  std::string name;
  std::string filename;
  bool visible = true;

  /*
  double meters_per_pixel = 0.05;  // relative to the parent floorplan scale
  double translation_x = 0.0;
  double translation_y = 0.0;
  double rotation = 0.0;
  */

  Transform transform;

  QImage image, colorized_image;
  QPixmap pixmap;
  QGraphicsPixmapItem* scene_item = nullptr;  // Borrowed pointer, not owned, don't delete

  std::vector<Feature> features;

  bool from_yaml(
    const std::string& name,
    const YAML::Node& data,
    const CoordinateSystem& coordinate_system);

  YAML::Node to_yaml(const CoordinateSystem& coordinate_system) const;

  bool load_image();
  void colorize_image();

  void draw(
    QGraphicsScene* scene,
    const double level_meters_per_pixel,
    const CoordinateSystem& coordinate_system);

  QColor color;

  static QColor default_color(const int layer_idx);

  QUuid add_feature(
    const double x,
    const double y,
    const double level_meters_per_pixel);

  void remove_feature(QUuid feature_uuid);

  const Feature* find_feature(
    const double x,
    const double y,
    const double drawing_meters_per_pixel) const;

  QPointF transform_global_to_layer(const QPointF& global_point);
  QPointF transform_layer_to_global(const QPointF& layer_point);

  void clear_selection();

  void populate_property_editor(QTableWidget* property_editor) const;

  std::vector<std::pair<std::string, std::string>> transform_strings;
};

#endif
