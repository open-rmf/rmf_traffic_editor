/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "traffic_editor/vertex.h"
using std::string;
using std::vector;
using std::pair;

const vector<pair<string, Param::Type>> Vertex::allowed_params
{
  { "is_parking_spot", Param::Type::BOOL },
  { "is_charger", Param::Type::BOOL},
  { "dock_name", Param::Type::STRING},
  { "workcell_name", Param::Type::STRING },
  { "spawn_robot_type", Param::Type::STRING },
  { "spawn_robot_name", Param::Type::STRING },
  { "is_holding_point", Param::Type::BOOL },
  { "is_passthrough_point", Param::Type::BOOL },
  { "human_goal_set_name", Param::Type::STRING },
};


Vertex::Vertex()
: x(0), y(0), selected(false)
{
  uuid = QUuid::createUuid();
}

Vertex::Vertex(double _x, double _y, const string& _name)
: x(_x), y(_y), name(_name), selected(false)
{
  uuid = QUuid::createUuid();
}

void Vertex::from_yaml(const YAML::Node& data)
{
  if (!data.IsSequence())
    throw std::runtime_error("Vertex::from_yaml expected a sequence");
  x = data[0].as<double>();
  y = data[1].as<double>();
  if (data.size() < 4)
    return;// todo: remove... intended only during format transition
  // skip the z-offset in data[2] for now
  name = data[3].as<string>();

  // load the parameters, all of which (including the params map) are
  // optional at the moment.
  if (data.size() >= 4)
  {
    for (YAML::const_iterator it = data[4].begin(); it != data[4].end(); ++it)
    {
      Param p;
      p.from_yaml(it->second);
      params[it->first.as<string>()] = p;
    }
  }
}

YAML::Node Vertex::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node vertex_node;
  vertex_node.SetStyle(YAML::EmitterStyle::Flow);
  vertex_node.push_back(std::round(x * 1000.0) / 1000.0);
  vertex_node.push_back(std::round(y * 1000.0) / 1000.0);
  vertex_node.push_back(0.0);  // placeholder for Z offsets in the future
  vertex_node.push_back(name);

  if (!params.empty())
  {
    YAML::Node params_node(YAML::NodeType::Map);
    for (const auto& param : params)
      params_node[param.first] = param.second.to_yaml();
    vertex_node.push_back(params_node);
  }
  return vertex_node;
}

void Vertex::draw(
  QGraphicsScene* scene,
  const double radius,
  const QColor& color) const
{
  QPen vertex_pen(Qt::black);
  vertex_pen.setWidthF(radius / 2.0);

  const double a = 0.5;

  QColor nonselected_color(color);
  nonselected_color.setAlphaF(a);

  QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, a);

  scene->addEllipse(
    x - radius,
    y - radius,
    2 * radius,
    2 * radius,
    vertex_pen,
    selected ? QBrush(selected_color) : QBrush(nonselected_color));

  if (!name.empty())
  {
    QGraphicsSimpleTextItem* item = scene->addSimpleText(
      QString::fromStdString(name),
      QFont("Helvetica", 6));
    item->setBrush(selected ? selected_color : color);
    item->setPos(x, y + radius);
  }
}

void Vertex::set_param(const std::string& param_name, const std::string& value)
{
  auto it = params.find(param_name);
  if (it == params.end())
  {
    printf("tried to set unknown parameter [%s]\n", param_name.c_str());
    return;  // unknown parameter
  }
  it->second.set(value);
}
