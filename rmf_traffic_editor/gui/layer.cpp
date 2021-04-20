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

#include <cmath>

#include <QImageReader>
#include <QGraphicsPixmapItem>
#include <QGraphicsColorizeEffect>
#include <QGraphicsScene>
#include "layer.h"
using std::string;
using std::vector;

Layer::Layer()
{
}

Layer::~Layer()
{
}

bool Layer::from_yaml(const std::string& _name, const YAML::Node& y)
{
  if (!y.IsMap())
    throw std::runtime_error("Layer::from_yaml() expected a map");
  name = _name;
  filename = y["filename"].as<string>();

  /*
  meters_per_pixel = y["meters_per_pixel"].as<double>();
  translation_x = y["translation_x"].as<double>();
  translation_y = y["translation_y"].as<double>();
  rotation = y["rotation"].as<double>();
  */

  if (y["visible"])
    visible = y["visible"].as<bool>();

  if (y["transform"] && y["transform"].IsMap())
    transform.from_yaml(y["transform"]);

  if (y["color"] && y["color"].IsSequence() && y["color"].size() == 4)
    color = QColor::fromRgbF(
      y["color"][0].as<double>(),
      y["color"][1].as<double>(),
      y["color"][2].as<double>(),
      y["color"][3].as<double>());

  if (y["features"] && y["features"].IsSequence())
  {
    const YAML::Node& fy = y["features"];
    for (YAML::const_iterator it = fy.begin(); it != fy.end(); ++it)
    {
      Feature f;
      f.from_yaml(*it);
      features.push_back(f);
    }
  }

  return load_image();
}

bool Layer::load_image()
{
  QImageReader image_reader(QString::fromStdString(filename));
  image_reader.setAutoTransform(true);
  QImage image = image_reader.read();
  if (image.isNull())
  {
    qWarning("unable to read %s: %s",
      qUtf8Printable(QString::fromStdString(filename)),
      qUtf8Printable(image_reader.errorString()));
    return false;
  }
  image = image.convertToFormat(QImage::Format_Grayscale8);
  pixmap = QPixmap::fromImage(image);
  printf("successfully opened %s\n", filename.c_str());

  return true;
}

YAML::Node Layer::to_yaml() const
{
  YAML::Node y;
  y["filename"] = filename;
  y["visible"] = visible;

  y["color"].push_back(std::round(color.redF() * 1000.0) / 1000.0);
  y["color"].SetStyle(YAML::EmitterStyle::Flow);
  y["color"].push_back(std::round(color.greenF() * 1000.0) / 1000.0);
  y["color"].push_back(std::round(color.blueF() * 1000.0) / 1000.0);
  y["color"].push_back(std::round(color.alphaF() * 1000.0) / 1000.0);

  for (const auto& feature : features)
    y["features"].push_back(feature.to_yaml());

  y["transform"] = transform.to_yaml();

  return y;
}

void Layer::draw(
  QGraphicsScene* scene,
  const double level_meters_per_pixel)
{
  if (!visible)
    return;

  printf("Layer::draw(transform.scale=%.3f, level_meters_per_pixel=%.3f)\n",
    transform.scale(), level_meters_per_pixel);

  QGraphicsPixmapItem* item = scene->addPixmap(pixmap);

  // Store for later use in getting coordinates back out
  scene_item = item;

  printf("layer xtrans=%.3f scale=%.3f level_mpp=%.3f\n", transform.translation().x(), transform.scale(), level_meters_per_pixel);

  item->setPos(
    transform.translation().x() / level_meters_per_pixel,
    transform.translation().y() / level_meters_per_pixel);

  item->setScale(transform.scale() / level_meters_per_pixel);

  item->setRotation(-1.0 * transform.yaw() * 180.0 / M_PI);

  QGraphicsColorizeEffect* colorize_effect = new QGraphicsColorizeEffect;
  colorize_effect->setColor(color);
  item->setGraphicsEffect(colorize_effect);

  for (Feature& feature : features)
    feature.draw(scene, color, transform, level_meters_per_pixel);
}

QColor Layer::default_color(const int layer_idx)
{
  switch (layer_idx)
  {
    case 0:
    default: return QColor::fromRgbF(1, 0, 0, 1.0);
    case 1:  return QColor::fromRgbF(0, 1, 0, 1.0);
    case 2:  return QColor::fromRgbF(0, 0, 1, 1.0);
    case 3:  return QColor::fromRgbF(1, 1, 0, 1.0);
    case 4:  return QColor::fromRgbF(0, 1, 1, 1.0);
    case 5:  return QColor::fromRgbF(1, 0, 1, 1.0);
  }
}

void Layer::remove_feature(QUuid feature_id)
{
  int index_to_remove = -1;

  for (size_t i = 0; i < features.size(); i++)
  {
    if (feature_id == features[i].id())
      index_to_remove = i;
  }

  if (index_to_remove < 0)
    return;

  features.erase(features.begin() + index_to_remove);
}

QUuid Layer::add_feature(
  const double x,
  const double y,
  const double level_meters_per_pixel)
{
  printf("Layer::add_feature(%s, %.3f, %.3f, %.3f)\n",
    name.c_str(),
    x,
    y,
    level_meters_per_pixel);

  // convert clicks in the working area to meters
  QPointF layer_pixel =
    transform.backwards(
      QPointF(x * level_meters_per_pixel, y * level_meters_per_pixel));

  printf("  transformed: (%.3f, %.3f)\n", layer_pixel.x(), layer_pixel.y());
  features.push_back(Feature(layer_pixel));

  return features.rbegin()->id();
}

const Feature* Layer::find_feature(
  const double x,
  const double y,
  const double level_meters_per_pixel) const
{
  QPointF layer_pixel =
    transform.backwards(
      QPointF(x * level_meters_per_pixel, y * level_meters_per_pixel));

  double min_dist = 1e9;
  const Feature* min_feature = nullptr;

  for (size_t i = 0; i < features.size(); i++)
  {
    const Feature& f = features[i];
    const double dx = layer_pixel.x() - f.x();
    const double dy = layer_pixel.y() - f.y();
    const double dist = sqrt(dx * dx + dy * dy);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_feature = &features[i];
    }
  }

  printf("min_dist = %.3f   layer scale = %.3f\n", min_dist, transform.scale());
  // scale calculation? probably wrong...
  if (min_dist * level_meters_per_pixel < Feature::radius_meters)
    return min_feature;

  return nullptr;
}

QPointF Layer::transform_global_to_layer(const QPointF& global_point)
{
  return transform.backwards(global_point);
}

QPointF Layer::transform_layer_to_global(const QPointF& layer_point)
{
  return transform.forwards(layer_point);
}
