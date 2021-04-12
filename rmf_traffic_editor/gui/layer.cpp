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
  meters_per_pixel = y["meters_per_pixel"].as<double>();
  translation_x = y["translation_x"].as<double>();
  translation_y = y["translation_y"].as<double>();
  rotation = y["rotation"].as<double>();
  if (y["visible"])
    visible = y["visible"].as<bool>();

  if (y["color"] && y["color"].IsSequence() && y["color"].size() == 4)
    color = QColor::fromRgbF(
      y["color"][0].as<double>(),
      y["color"][1].as<double>(),
      y["color"][2].as<double>(),
      y["color"][3].as<double>());

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
  y.SetStyle(YAML::EmitterStyle::Flow);
  y["filename"] = filename;
  y["meters_per_pixel"] = meters_per_pixel;
  y["translation_x"] = translation_x;
  y["translation_y"] = translation_y;
  y["rotation"] = rotation;
  y["visible"] = visible;

  y["color"].push_back(color.redF());
  y["color"].push_back(color.greenF());
  y["color"].push_back(color.blueF());
  y["color"].push_back(color.alphaF());

  return y;
}

void Layer::draw(
  QGraphicsScene* scene,
  const double level_meters_per_pixel)
{
  if (!visible)
    return;

  QGraphicsPixmapItem* item = scene->addPixmap(pixmap);

  // Store for later use in getting coordinates back out
  scene_item = item;

  // set the origin of the pixmap frame to the lower-left corner
  item->setOffset(0, -pixmap.height());

  item->setPos(
    -translation_x / level_meters_per_pixel,
    translation_y / level_meters_per_pixel);

  item->setScale(meters_per_pixel / level_meters_per_pixel);

  item->setRotation(-1.0 * rotation * 180.0 / M_PI);

  QGraphicsColorizeEffect* colorize_effect = new QGraphicsColorizeEffect;
  colorize_effect->setColor(color);
  item->setGraphicsEffect(colorize_effect);
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
