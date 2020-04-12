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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

#include <QFileInfo>
#include <QDir>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <QElapsedTimer>

#include "building.h"
#include "yaml_utils.h"

using std::string;
using std::vector;
using std::make_pair;
using std::unique_ptr;
using std::shared_ptr;


Building::Building()
: name("building")
{
}

Building::~Building()
{
}

/// Load a YAML file description of a building map
///
/// This function replaces the contents of this object with what is
/// in the YAML file.
bool Building::load_yaml_file()
{
  printf("Building::load_yaml_file(%s)\n", filename.c_str());
  YAML::Node y;
  try
  {
    y = YAML::LoadFile(filename.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", filename.c_str(), e.what());
    return false;
  }

  // change directory to the path of the file, so that we can correctly open
  // relative paths recorded in the file

  // TODO: save previous directory and restore it when leaving this function
  // in case the building file is in a different path from the project file
  QString dir(QFileInfo(QString::fromStdString(filename)).absolutePath());
  qDebug("changing directory to [%s]", qUtf8Printable(dir));
  if (!QDir::setCurrent(dir))
  {
    printf("couldn't change directory\n");
    return false;
  }

  if (y["name"])
    name = y["name"].as<string>();

  if (y["reference_level_name"])
    reference_level_name = y["reference_level_name"].as<string>();

  if (!y["levels"] || !y["levels"].IsMap())
  {
    printf("expected top-level dictionary named 'levels'");
    return false;
  }

  levels.clear();
  const YAML::Node yl = y["levels"];
  for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
  {
    unique_ptr<BuildingLevel> l = std::make_unique<BuildingLevel>();
    l->from_yaml(it->first.as<string>(), it->second);
    levels.push_back(std::move(l));
  }

  QtConcurrent::blockingMap(
    levels,
    [&](auto& level) { level->load_drawing(); });

  // now that all images are loaded, we can calculate scale for annotated
  // measurement lanes
  for (auto& level : levels)
    level->calculate_scale();

  if (y["lifts"] && y["lifts"].IsMap())
  {
    const YAML::Node& y_lifts = y["lifts"];
    for (YAML::const_iterator it = y_lifts.begin(); it != y_lifts.end(); ++it)
    {
      Lift lift;
      lift.from_yaml(it->first.as<string>(), it->second);
      lifts.push_back(lift);
    }
  }

  calculate_all_transforms();
  return true;
}

bool Building::save_yaml_file()
{
  printf("Building::save_yaml(%s)\n", filename.c_str());

  YAML::Node y;
  y["name"] = name;

  if (!reference_level_name.empty())
    y["reference_level_name"] = reference_level_name;

  y["levels"] = YAML::Node(YAML::NodeType::Map);
  for (const auto &level : levels)
    y["levels"][level->name] = level->to_yaml();

  y["lifts"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& lift : lifts)
    y["lifts"][lift.name] = lift.to_yaml();

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  return true;
}

void Building::add_vertex(int level_index, double x, double y)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;
  levels[level_index]->add_vertex(x, y);
}

void Building::add_fiducial(int level_index, double x, double y)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;
  levels[level_index]->fiducials.push_back(Fiducial(x, y));
}

int Building::find_nearest_vertex_index(
    int level_index, double x, double y, double &distance)
{
  double min_dist = 1e100;
  int min_index = -1;
  for (size_t i = 0; i < levels[level_index]->vertices.size(); i++) {
    const Vertex &v = levels[level_index]->vertices[i];
    const double dx = x - v.x;
    const double dy = y - v.y;
    const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
    if (dist2 < min_dist) {
      min_dist = dist2;
      min_index = i;
    }
  }
  distance = sqrt(min_dist);
  return min_index;  // will be -1 if vertices vector is empty
}

Building::NearestItem Building::nearest_items(
      const int level_index,
      const double x,
      const double y)
{
  NearestItem ni;
  if (level_index >= static_cast<int>(levels.size()))
    return ni;
  const BuildingLevel& level = *levels[level_index];

  for (size_t i = 0; i < level.vertices.size(); i++)
  {
    const Vertex& p = level.vertices[i];
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double dist = sqrt(dx*dx + dy*dy);
    if (dist < ni.vertex_dist)
    {
      ni.vertex_dist = dist;
      ni.vertex_idx = i;
    }
  }

  for (size_t i = 0; i < level.fiducials.size(); i++)
  {
    const Fiducial& f = level.fiducials[i];
    const double dx = x - f.x;
    const double dy = y - f.y;
    const double dist = sqrt(dx*dx + dy*dy);
    if (dist < ni.fiducial_dist)
    {
      ni.fiducial_dist = dist;
      ni.fiducial_idx = i;
    }
  }

  for (size_t i = 0; i < level.models.size(); i++)
  {
    const Model& m = *level.models[i];
    const double dx = x - m.state.x;
    const double dy = y - m.state.y;
    const double dist = sqrt(dx*dx + dy*dy);  // no need for sqrt each time
    if (dist < ni.model_dist)
    {
      ni.model_dist = dist;
      ni.model_idx = i;
    }
  }

  return ni;
}

int Building::nearest_item_index_if_within_distance(
      const int level_index,
      const double x,
      const double y,
      const double distance_threshold,
      const ItemType item_type)
{
  if (level_index >= static_cast<int>(levels.size()))
    return -1;

  double min_dist = 1e100;
  int min_index = -1;
  if (item_type == VERTEX)
  {
    for (size_t i = 0; i < levels[level_index]->vertices.size(); i++)
    {
      const Vertex& p = levels[level_index]->vertices[i];
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  else if (item_type == FIDUCIAL)
  {
    for (size_t i = 0; i < levels[level_index]->fiducials.size(); i++)
    {
      const Fiducial& f = levels[level_index]->fiducials[i];
      const double dx = x - f.x;
      const double dy = y - f.y;
      const double dist2 = dx*dx + dy*dy;
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  else if (item_type == MODEL)
  {
    for (size_t i = 0; i < levels[level_index]->models.size(); i++)
    {
      const Model& m = *levels[level_index]->models[i];
      const double dx = x - m.state.x;
      const double dy = y - m.state.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  if (sqrt(min_dist) < distance_threshold)
    return min_index;
  return -1;
}

void Building::add_edge(
      const int level_index,
      const int start_vertex_index,
      const int end_vertex_index,
      const Edge::Type edge_type)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;

  printf("Building::add_edge(%d, %d, %d, %d)\n",
      level_index, start_vertex_index, end_vertex_index,
      static_cast<int>(edge_type));
  levels[level_index]->edges.push_back(
      Edge(start_vertex_index, end_vertex_index, edge_type));
}

bool Building::delete_selected(const int level_index)
{
  if (level_index >= static_cast<int>(levels.size()))
    return false;

  printf("Building::delete_keypress()\n");
  if (!levels[level_index]->delete_selected())
    return false;

  return true;
}

void Building::add_model(
    const int level_idx,
    const double x,
    const double y,
    const double z,
    const double yaw,
    const std::string &model_name)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  printf("Building::add_model(%d, %.1f, %.1f, %.1f, %.2f, %s)\n",
      level_idx, x, y, z, yaw, model_name.c_str());
  unique_ptr<Model> m = std::make_unique<Model>();
  m->state.x = x;
  m->state.y = y;
  m->state.z = z;
  m->state.yaw = yaw;
  m->model_name = model_name;
  m->instance_name = model_name;  // todo: add unique numeric suffix?
  m->is_static = true;
  levels[level_idx]->models.push_back(std::move(m));
}

void Building::set_model_yaw(
    const int level_idx,
    const int model_idx,
    const double yaw)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  levels[level_idx]->models[model_idx]->state.yaw = yaw;
}

void Building::clear()
{
  name = "";
  reference_level_name = "";
  levels.clear();
  lifts.clear();
  clear_transform_cache();
}

void Building::add_level(unique_ptr<BuildingLevel> new_level)
{
  // make sure we don't have this level already
  for (const auto &level : levels)
    if (level->name == new_level->name)
      return;
  levels.push_back(std::move(new_level));
}

void Building::draw_lifts(QGraphicsScene *scene, const int level_idx)
{
  const BuildingLevel& level = *levels[level_idx];
  for (const auto &lift : lifts)
  {
    // find the level index referenced by the lift
    int reference_floor_idx = -1;
    for (size_t i = 0; i < levels.size(); i++)
      if (levels[i]->name == lift.reference_floor_name)
      {
        reference_floor_idx = static_cast<int>(i);
        break;
      }

    Transform t;
    if (reference_floor_idx >= 0)
      t = get_transform(reference_floor_idx, level_idx);

    lift.draw(
        scene,
        level.drawing_meters_per_pixel,
        level.name,
        true,
        t.scale,
        t.dx,
        t.dy);
  }
}

bool Building::transform_between_levels(
    const std::string& from_level_name,
    const QPointF& from_point,
    const std::string& to_level_name,
    QPointF& to_point)
{
  int from_level_idx = -1;
  int to_level_idx = -1;
  for (size_t i = 0; i < levels.size(); i++)
  {
    if (levels[i]->name == from_level_name)
      from_level_idx = i;
    if (levels[i]->name == to_level_name)
      to_level_idx = i;
  }
  if (from_level_idx < 0 || to_level_idx < 0)
  {
    to_point = from_point;
    return false;
  }
  return transform_between_levels(
      from_level_idx,
      from_point,
      to_level_idx,
      to_point);
}

bool Building::transform_between_levels(
    const int from_level_idx,
    const QPointF& from_point,
    const int to_level_idx,
    QPointF& to_point)
{

  if (from_level_idx < 0 ||
      from_level_idx >= static_cast<int>(levels.size()) ||
      to_level_idx < 0 ||
      to_level_idx >= static_cast<int>(levels.size()))
  {
    to_point = from_point;
    return false;
  }

  const Transform t = get_transform(from_level_idx, to_level_idx);

  to_point.rx() = t.scale * from_point.x() + t.dx;
  to_point.ry() = t.scale * from_point.y() + t.dy;
  return true;
}

void Building::clear_transform_cache()
{
  transforms.clear();
}

Building::Transform Building::compute_transform(
    const int from_level_idx,
    const int to_level_idx)
{
  // short-circuit if it's the same level
  if (from_level_idx == to_level_idx)
  {
    Building::Transform t;
    t.scale = 1.0;
    t.dx = 0.0;
    t.dy = 0.0;
    return t;
  }

  // this internal function assumes that bounds checking has already happened
  const BuildingLevel& from_level = *levels[from_level_idx];
  const BuildingLevel& to_level = *levels[to_level_idx];

  // assemble a vector of fudicials in common to these levels
  vector< std::pair<Fiducial, Fiducial> > fiducials;
  for (const Fiducial& f0 : from_level.fiducials)
    for (const Fiducial& f1 : to_level.fiducials)
      if (f0.name == f1.name)
      {
        fiducials.push_back(make_pair(f0, f1));
        break;
      }

  // calculate the distances between each fiducial on their levels
  vector< std::pair <double, double > > distances;
  for (size_t f0_idx = 0; f0_idx < fiducials.size(); f0_idx++)
    for (size_t f1_idx = f0_idx + 1; f1_idx < fiducials.size(); f1_idx++)
      distances.push_back(
          make_pair(
              fiducials[f0_idx].first.distance(fiducials[f1_idx].first),
              fiducials[f0_idx].second.distance(fiducials[f1_idx].second)));

  // for now, we'll just compute the mean of the relative scale estimates.
  // we can do fancier statistics later, if needed.
  double relative_scale_sum = 0;
  for (size_t i = 0; i < distances.size(); i++)
    relative_scale_sum += distances[i].second / distances[i].first;
  const double scale = relative_scale_sum / distances.size();

  // scale the fiducials and estimate the "optimal" translation.
  // for now, we'll just use the mean of the translation estimates.
  double trans_x_sum = 0;
  double trans_y_sum = 0;
  for (const auto& fiducial : fiducials)
  {
    trans_x_sum += fiducial.second.x - fiducial.first.x * scale;
    trans_y_sum += fiducial.second.y - fiducial.first.y * scale;
  }
  const double trans_x = trans_x_sum / fiducials.size();
  const double trans_y = trans_y_sum / fiducials.size();

  Building::Transform t;
  t.scale = scale;
  t.dx = trans_x;
  t.dy = trans_y;

  printf("transform %d->%d: scale = %.5f translation = (%.2f, %.2f)\n",
      from_level_idx,
      to_level_idx,
      t.scale,
      t.dx,
      t.dy);

  return t;
}

Building::Transform Building::get_transform(
      const int from_level_idx,
      const int to_level_idx)
{
  // this operation is a bit "heavy" so we'll cache the transformations
  // as they are computed
  LevelPair level_pair;
  level_pair.from_idx = from_level_idx;
  level_pair.to_idx = to_level_idx;

  TransformMap::iterator transform_it = transforms.find(level_pair);
  Transform t;

  if (transform_it == transforms.end())
  {
    // the transform wasn't in the cache, so we need to compute it
    t = compute_transform(from_level_idx, to_level_idx);
    transforms[level_pair] = t;
  }
  else
    t = transform_it->second;

  return t;
}

void Building::calculate_all_transforms()
{
  if (levels.empty())
    return;  // let's not crash

  clear_transform_cache();
  for (size_t i = 0; i < levels.size(); i++)
    for (size_t j = 0; j < levels.size(); j++)
      get_transform(i, j);

  // set drawing scale using this data
  const int ref_idx = get_reference_level_idx();
  const double ref_scale = levels[ref_idx]->drawing_meters_per_pixel;
  for (int i = 0; i < static_cast<int>(levels.size()); i++)
  {
    if (i != get_reference_level_idx())
    {
      Transform t = get_transform(ref_idx, i);
      if (levels[i]->fiducials.size() >= 2)
        levels[i]->drawing_meters_per_pixel = ref_scale / t.scale;
    }
  }
}

int Building::get_reference_level_idx()
{
  if (reference_level_name.empty())
    return 0;
  for (size_t i = 0; i < levels.size(); i++)
    if (levels[i]->name == reference_level_name)
      return static_cast<int>(i);
  return 0;
}

void Building::clear_scene()
{
  for (auto& level : levels)
    level->clear_scene();

  {
    std::lock_guard<std::mutex> guard(active_edges_mutex);
    for (auto& active_edge : active_edges)
      active_edge.graphics_line = nullptr;
  }
}

shared_ptr<planner::Graph> Building::planner_graph(
    const int graph_idx,
    const string& level_name)
{
  shared_ptr<planner::Graph> graph = std::make_shared<planner::Graph>();

  // spin through all edges
  for (const auto& level : levels)
  {
    if (level->name != level_name)
      continue;

    for (const auto& edge : level->edges)
    {
      if (edge.type != Edge::LANE)
        continue;

      if (edge.get_graph_idx() == graph_idx)
      {
        const Vertex& v_start = level->vertices[edge.start_idx];
        const Vertex& v_end = level->vertices[edge.end_idx];
        graph->add_edge(v_start, v_end);
        if (edge.is_bidirectional())
          graph->add_edge(v_end, v_start);
      }
    }

    graph->scale_all_nodes(level->drawing_meters_per_pixel);
    break;  // there will only be one level of this name.
  }

  return graph;
}

double Building::level_meters_per_pixel(const string& level_name) const
{
  for (const auto& level : levels)
    if (level->name == level_name)
      return level->drawing_meters_per_pixel;
  return 0.05;  // just a somewhat sane default
}

bool Building::request_lane_edge(
    const std::string& level_name,
    const planner::Edge &edge,
    const std::string& requester_name,
    const bool remove_all_other_reservations)
{
  std::lock_guard<std::mutex> guard(active_edges_mutex);

  /*
  static int req_count = 0;
  if (++req_count > 20)
    exit(1);
  */

  /*
  printf(
      "REQUEST [%s]: (%.2f, %.2f)->(%.2f, %.2f)\n",
      requester_name.c_str(),
      edge.start->x,
      edge.start->y,
      edge.end->x,
      edge.end->y);

  for (const auto& e : active_edges)
  {
    printf("    %s: (%.2f, %.2f)->(%.2f, %.2f)\n",
        e.model_name.c_str(),
        e.edge.start->x,
        e.edge.start->y,
        e.edge.end->x,
        e.edge.end->y);
  }
  */

  for (auto it = active_edges.begin(); it != active_edges.end(); ++it)
  {
    if (it->level_name != level_name)
      continue;

    if (it->model_name == requester_name)
      continue;  // don't be afraid of our own reservations...

    if (it->edge == edge)
      return false;

    if (*(it->edge.start) == *(edge.start) ||
        *(it->edge.start) == *(edge.end) ||
        *(it->edge.end) == *(edge.start) ||
        *(it->edge.end) == *(edge.end))
      return false;
  }

  if (remove_all_other_reservations)
  {
    /*
    active_edges.erase(
        std::remove_if(
            active_edges.begin(),
            active_edges.end(),
            [requester_name](const EdgeReservation& res)
            { 
              return res.model_name == requester_name;
            }),
        active_edges.end());
    */
    for (auto it = active_edges.begin(); it != active_edges.end(); )
    {
      if (it->model_name == requester_name)
      {
        if (it->graphics_line)
          lines_to_remove.push_back(it->graphics_line);
        it = active_edges.erase(it);
      }
      else
        ++it;
    }
  }

  EdgeReservation reservation;
  reservation.level_name = level_name;
  reservation.edge = edge;
  reservation.model_name = requester_name;

  active_edges.push_back(reservation);

  // printf("  %zu active edges\n", active_edges.size());
  // printf("after request: %zu active edges\n", active_edges.size());

  return true;
}

void Building::release_lane_edge(
    const std::string& level_name,
    const planner::Edge &edge,
    const std::string& requester_name)
{
  std::lock_guard<std::mutex> guard(active_edges_mutex);

  if (!edge.start || !edge.end)
    return;

#if 0
  printf(
      "RELEASE [%s]: (%.2f, %.2f)->(%.2f, %.2f)\n",
      requester_name.c_str(),
      edge.start->x,
      edge.start->y,
      edge.end->x,
      edge.end->y);

  for (const auto& e : active_edges)
  {
    printf("    %s: (%.2f, %.2f)->(%.2f, %.2f)\n",
        e.model_name.c_str(),
        e.edge.start->x,
        e.edge.start->y,
        e.edge.end->x,
        e.edge.end->y);
  }
#endif

  for (auto it = active_edges.begin(); it != active_edges.end(); ++it)
  {
    if (it->model_name != requester_name)
      continue;

    if (it->level_name != level_name)
      continue;

    if (it->edge == edge)
    {
      if (it->graphics_line)
        lines_to_remove.push_back(it->graphics_line);
      active_edges.erase(it);
      break;
    }
  }

#if 0
  if (found)
    printf("  found edge! released it.\n");
  else
    printf("  couldn't find edge to release\n");

  printf("after release: %zu active edges\n", active_edges.size());
#endif
}

void Building::draw_active_edges(QGraphicsScene *scene, const int level_idx)
{
  const string& level_name = levels[level_idx]->name;

  std::lock_guard<std::mutex> guard(active_edges_mutex);

  for (QGraphicsLineItem* line_to_remove : lines_to_remove)
    scene->removeItem(line_to_remove);
  lines_to_remove.clear();

  for (auto& active_edge : active_edges)
  {
    if (active_edge.graphics_line)
      continue;

    if (active_edge.level_name != level_name)
      continue;

    // todo: draw the actual line
    const double scale = levels[level_idx]->drawing_meters_per_pixel;
    active_edge.graphics_line = scene->addLine(
        active_edge.edge.start->x / scale,
        active_edge.edge.start->y / scale,
        active_edge.edge.end->x / scale,
        active_edge.edge.end->y / scale,
        QPen(
          QBrush(QColor::fromRgbF(1.0, 1.0, 0.0, 0.5)),
          1.0 / scale,
          Qt::SolidLine, Qt::RoundCap));
      
  }
}

void Building::release_all_lane_edges_for_model(
    const std::string& requester_name)
{
  // DO NOT USE...
  std::lock_guard<std::mutex> guard(active_edges_mutex);
  active_edges.erase(
      std::remove_if(
          active_edges.begin(),
          active_edges.end(),
          [requester_name](const EdgeReservation& res)
          { 
            return res.model_name == requester_name;
          }));
}
