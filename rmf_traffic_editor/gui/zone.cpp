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

#include <algorithm>
#include <cmath>
#include <iostream>

#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>

#include "zone.h"
using std::string;

YAML::Node InternalVertex::to_yaml() const
{
  YAML::Node n;
  n["x"] = std::round(x * 1000.0) / 1000.0;
  n["y"] = std::round(y * 1000.0) / 1000.0;
  n["priority"] = priority;
  n["group"] = group;

  return n;
}

void InternalVertex::from_yaml(const std::string& _name, const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("InternalVertex::from_yaml() expected a map");
  name = _name;
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  priority = data["priority"].as<uint>();
  group = data["group"].as<std::string>();
}

YAML::Node ExternalVertex::to_yaml() const
{
  YAML::Node n;
  n["is_entry_point"] = is_entry_point;
  n["is_exit_point"] = is_exit_point;

  return n;
}

void ExternalVertex::from_yaml(const std::string& _name, const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("ExternalVertex::from_yaml() expected a map");
  name = _name;
  is_entry_point = data["is_entry_point"].as<bool>();
  is_exit_point = data["is_exit_point"].as<bool>();
}

Zone::Zone()
{
}

void Zone::from_yaml(
  const std::string& _name,
  const YAML::Node& data,
  const std::vector<Level>& levels)
{
  if (!data.IsMap())
    throw std::runtime_error("Zone::from_yaml() expected a map");
  name = _name;
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  yaw = data["yaw"].as<double>();
  level = data["level"].as<string>();

  for (const auto& map_level : levels)
  {
    if (map_level.name == level)
    {
      elevation = map_level.elevation;
      break;
    }
  }

  width = data["width"].as<double>();
  depth = data["depth"].as<double>();

  if (data["internal_vertices"] && data["internal_vertices"].IsMap())
  {
    const YAML::Node& v = data["internal_vertices"];
    for (YAML::const_iterator it = v.begin(); it != v.end(); ++it)
    {
      InternalVertex iv;
      iv.from_yaml(it->first.as<string>(), it->second);
      internal_vertices.push_back(iv);
    }
  }

  if (data["external_vertices"] && data["external_vertices"].IsMap())
  {
    const YAML::Node& v = data["external_vertices"];
    for (YAML::const_iterator it = v.begin(); it != v.end(); ++it)
    {
      ExternalVertex ev;
      ev.from_yaml(it->first.as<string>(), it->second);
      external_vertices.push_back(ev);
    }
  }
}

YAML::Node Zone::to_yaml() const
{
  YAML::Node n;
  n["x"] = std::round(x * 1000.0) / 1000.0;
  n["y"] = std::round(y * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = std::round(yaw * 10000.0) / 10000.0;
  n["level"] = level;
  n["width"] = std::round(width * 1000.0) / 1000.0;
  n["depth"] = std::round(depth * 1000.0) / 1000.0;

  n["internal_vertices"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& iv : internal_vertices)
    n["internal_vertices"][iv.name] = iv.to_yaml();

  n["external_vertices"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& ev : external_vertices)
    n["external_vertices"][ev.name] = ev.to_yaml();
  return n;
}

void Zone::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel,
  const string& level_name,
  const bool apply_transformation,
  const std::vector<Vertex>& level_vertices) const
{
  if (!show_zone && apply_transformation)
    return;

  if (level_name != level && apply_transformation)
    return;

  const double zone_w = width / meters_per_pixel;
  const double zone_d = depth / meters_per_pixel;
  QPen zone_pen(Qt::black);
  zone_pen.setWidth(0.05 / meters_per_pixel);

  QGraphicsRectItem* zone_rect = new QGraphicsRectItem(
    -zone_w / 2.0,
    -zone_d / 2.0,
    zone_w,
    zone_d);
  zone_rect->setPen(zone_pen);
  zone_rect->setBrush(QBrush(QColor::fromRgbF(0.3, 0.3, 1.0, 0.2)));
  scene->addItem(zone_rect);

  QList<QGraphicsItem*> items;
  items.append(zone_rect);

  if (!name.empty())
  {
    QFont font("Helvetica");
    font.setPointSize(0.2 / meters_per_pixel);
    QGraphicsSimpleTextItem* text_item = scene->addSimpleText(
      QString::fromStdString(name), font);
    text_item->setBrush(QColor(255, 0, 0, 255));
    text_item->setPos(-zone_w / 3.0, 0.0);
    items.append(text_item);
  }

  const double radius = 0.1 / meters_per_pixel;

  QFont name_font("Helvetica");
  name_font.setPointSize(0.1 / meters_per_pixel);
  QFont annotate_font("Helvetica");
  annotate_font.setPointSize(0.15 / meters_per_pixel);
  QPen vertex_pen(Qt::black);
  vertex_pen.setWidthF(radius / 2.0);
  const QBrush vertex_brush = QBrush(
    QColor::fromRgbF(0.0, 0.0, 0.0, 0.5));

  if (apply_transformation)
  {
    if (show_vertices)
    {
      for (const auto& iv : internal_vertices)
      {
        QGraphicsEllipseItem* ellipse_item = scene->addEllipse(
          iv.x /meters_per_pixel - radius,
          iv.y /meters_per_pixel - radius,
          2 * radius,
          2 * radius,
          vertex_pen,
          vertex_brush);
        ellipse_item->setZValue(20.0);
        items.append(ellipse_item);

        QGraphicsSimpleTextItem* annotate_text_item = scene->addSimpleText(
          QString::fromStdString(iv.group + "/p" + std::to_string(iv.priority)),
          annotate_font);
        annotate_text_item->setBrush(QColor(255, 0, 0, 255));
        annotate_text_item->setPos(
          iv.x /meters_per_pixel + radius,
          iv.y /meters_per_pixel - 3.0 * radius);
        annotate_text_item->setZValue(95.0);
        items.append(annotate_text_item);

        QGraphicsSimpleTextItem* name_text_item = scene->addSimpleText(
          QString::fromStdString(iv.name),
          name_font);
        name_text_item->setBrush(QColor(255, 0, 0, 255));
        name_text_item->setPos(
          iv.x /meters_per_pixel + radius,
          iv.y /meters_per_pixel + radius);
        name_text_item->setZValue(95.0);
        items.append(name_text_item);
      }
    }

    if (show_lanes)
    {
      const double lane_pen_width = 0.5 / meters_per_pixel;
      QPen lane_pen(QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.3)),
        lane_pen_width);
      lane_pen.setCapStyle(Qt::RoundCap);
      const QPen arrow_pen(
        QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.3)), lane_pen_width / 8.0);

      for (const auto& ev : external_vertices)
      {
        if (!ev.is_entry_point && !ev.is_exit_point)
          continue;

        const Vertex* v = nullptr;
        for (const auto& lv : level_vertices)
        {
          if (lv.name == ev.name)
          {
            v = &lv;
            break;
          }
        }
        if (v == nullptr)
          continue;

        const double dx = v->x - x;
        const double dy = v->y - y;
        const double ex = std::cos(yaw) * dx - std::sin(yaw) * dy;
        const double ey = std::sin(yaw) * dx + std::cos(yaw) * dy;

        for (const auto& iv : internal_vertices)
        {
          const double ix = iv.x / meters_per_pixel;
          const double iy = iv.y / meters_per_pixel;

          QGraphicsLineItem* lane_item = scene->addLine(ex, ey, ix, iy,
              lane_pen);
          items.append(lane_item);

          // only draw arrows if it's a unidirectional lane
          if (ev.is_entry_point == ev.is_exit_point)
            continue;

          const double start_x = ev.is_entry_point ? ex : ix;
          const double start_y = ev.is_entry_point ? ey : iy;
          const double end_x = ev.is_entry_point ? ix : ex;
          const double end_y = ev.is_entry_point ? iy : ey;
          const double len = std::sqrt(
            (end_x - start_x) * (end_x - start_x)
            + (end_y - start_y) * (end_y - start_y));
          if (len < 1e-6)
            continue;

          const double norm_x = (end_x - start_x) / len;
          const double norm_y = (end_y - start_y) / len;
          const double arrow_w = lane_pen_width / 2.5;
          const double arrow_l = lane_pen_width / 2.5;
          const double arrow_spacing = lane_pen_width * 4.0;

          for (double d = 0.0; d < len; d += arrow_spacing)
          {
            // first calculate the center vertex of this arrowhead
            const double cx = start_x + d * norm_x;
            const double cy = start_y + d * norm_y;
            // one edge vertex of arrowhead
            const double e1x = cx - arrow_w * norm_y;
            const double e1y = cy + arrow_w * norm_x;
            // another edge vertex of arrowhead
            const double e2x = cx + arrow_w * norm_y;
            const double e2y = cy - arrow_w * norm_x;
            // tip of arrowhead
            const double tx = cx + arrow_l * norm_x;
            const double ty = cy + arrow_l * norm_y;
            // now add arrowhead lines
            items.append(scene->addLine(e1x, e1y, tx, ty, arrow_pen));
            items.append(scene->addLine(e2x, e2y, tx, ty, arrow_pen));
          }
        }
      }
    }

    QGraphicsItemGroup* group = scene->createItemGroup(items);
    group->setZValue(95.0);
    group->setPos(x, y);
    group->setRotation(-180.0 / 3.1415926 * yaw);
  }
  else if (show_vertices)
  {
    for (std::size_t i = 0; i < internal_vertices.size(); i++)
    {
      const InternalVertex& vertex = internal_vertices[i];
      const double px = vertex.x / meters_per_pixel;
      const double py = vertex.y / meters_per_pixel;

      QGraphicsEllipseItem* ellipse_item = scene->addEllipse(
        px - radius,
        py - radius,
        2 * radius,
        2 * radius,
        vertex_pen,
        vertex_brush);
      ellipse_item->setZValue(20.0);

      QGraphicsSimpleTextItem* name_item = scene->addSimpleText(
        QString::fromStdString(vertex.name),
        name_font);
      name_item->setBrush(QColor(255, 0, 0, 255));
      name_item->setPos(px + radius, py + radius);
      name_item->setZValue(30.0);

      if (!vertex.group.empty())
      {
        QGraphicsSimpleTextItem* annotate_text_item = scene->addSimpleText(
          QString::fromStdString(
            vertex.group + "/p" + std::to_string(vertex.priority)),
          annotate_font);
        annotate_text_item->setBrush(QColor(255, 0, 0, 255));
        annotate_text_item->setPos(px + radius, py - 3.0 * radius);
        annotate_text_item->setZValue(30.0);
      }
    }
  }
}
