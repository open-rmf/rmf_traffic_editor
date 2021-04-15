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

#include <cmath>

#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QString>

#include "constraint.hpp"
#include "level.h"

using std::string;


Constraint::Constraint()
{
}

Constraint::Constraint(const QUuid& a, const QUuid& b)
{
  _ids.push_back(a);
  _ids.push_back(b);
}

void Constraint::add_id(const QUuid& id)
{
  _ids.push_back(id);
}

void Constraint::from_yaml(const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("Constraint::from_yaml() expected a map");

  if (data["ids"] && data["ids"].IsSequence())
  {
    const YAML::Node& y_ids = data["ids"];
    for (YAML::const_iterator it = y_ids.begin(); it != y_ids.end(); ++it)
      _ids.push_back(QUuid(QString::fromStdString(it->as<string>())));
  }
}

YAML::Node Constraint::to_yaml() const
{
  YAML::Node node;
  node.SetStyle(YAML::EmitterStyle::Flow);
  for (const auto& id : _ids)
    node["ids"].push_back(id.toString().toStdString());
  return node;
}

void Constraint::draw(
  QGraphicsScene* scene,
  const Level& level) const
{
  if (_ids.size() != 2)
  {
    printf("WOAH! tried to draw a constraint with only %d ID's!\n",
      static_cast<int>(_ids.size()));
    return;
  }

  const QColor color = QColor::fromRgbF(0.7, 0.7, 0.2, 1.0);
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, 0.5);

  const double pen_width = 0.1 / level.drawing_meters_per_pixel;
  QPen pen(
    QBrush(_selected ? selected_color : color),
    pen_width,
    Qt::SolidLine,
    Qt::RoundCap);

  const Feature *f1 = level.find_feature(_ids[0]);
  if (!f1)
  {
    printf("woah! couldn't find constraint ID %s\n",
      _ids[0].toString().toStdString().c_str());
    return;
  }

  const Feature *f2 = level.find_feature(_ids[1]);
  if (!f1)
  {
    printf("woah! couldn't find constraint ID %s\n",
      _ids[1].toString().toStdString().c_str());
    return;
  }

  QGraphicsLineItem* line = scene->addLine(
    f1->x(),
    f1->y(),
    f2->x(),
    f2->y(),
    pen);
  line->setZValue(199.0);
}

bool Constraint::operator==(const Constraint& other)
{
  if (_ids.size() != other._ids.size())
    return false;

  bool all_equal_forwards = true;
  bool all_equal_backwards = true;

  for (size_t i = 0; i < _ids.size(); i++)
  {
    if (_ids[i] != other._ids[i])
      all_equal_forwards = false;

    if (_ids[i] != other._ids[other._ids.size() - 1 - i])
      all_equal_backwards = false;
  }
  return all_equal_forwards || all_equal_backwards;
}
