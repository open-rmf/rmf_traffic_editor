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
: name("building"),
  coordinate_system(CoordinateSystem::ReferenceImage)
{
}

Building::~Building()
{
}

/// Load a YAML file description of a building map
///
/// This function replaces the contents of this object with what is
/// in the YAML file.
bool Building::load(const string& _filename)
{
  printf("Building::load(%s)\n", _filename.c_str());
  filename = _filename;

  if (filename.find(".project.yaml") != string::npos)
  {
    printf(
      "\nIt looks like this is a previous traffic-editor project file. "
      "This file is no longer used. Please load the .building.yaml "
      "file instead.\n\n");
    return false;
  }

  YAML::Node y;
  try
  {
    y = YAML::LoadFile(filename.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s\n", filename.c_str(), e.what());
    return false;
  }

  // change directory to the path of the file, so that we can correctly open
  // relative paths recorded in the file

  QString dir(QFileInfo(QString::fromStdString(filename)).absolutePath());
  qDebug("changing directory to [%s]", qUtf8Printable(dir));
  if (!QDir::setCurrent(dir))
  {
    printf("couldn't change directory\n");
    return false;
  }

  if (y["name"])
    name = y["name"].as<string>();

  if (y["coordinate_system"])
    coordinate_system.value =
      CoordinateSystem::value_from_string(y["coordinate_system"].as<string>());

  if (y["reference_level_name"])
    reference_level_name = y["reference_level_name"].as<string>();
  else
    reference_level_name = "";

  // crowd_sim_impl is initialized when creating crowd_sim_table in editor.cpp
  // just in case the pointer is not initialized
  if (crowd_sim_impl == nullptr)
    crowd_sim_impl = std::make_shared<crowd_sim::CrowdSimImplementation>();
  if (y["crowd_sim"] && y["crowd_sim"].IsMap())
  {
    if (!crowd_sim_impl->from_yaml(y["crowd_sim"]))
    {
      printf(
        "Error in loading crowd_sim configuration from yaml, re-initialize crowd_sim");
      crowd_sim_impl->clear();
      crowd_sim_impl->init_default_configure();
    }
  }

  if (!y["levels"] || !y["levels"].IsMap())
  {
    printf("expected top-level dictionary named 'levels'");
    return false;
  }

  levels.clear();
  const YAML::Node yl = y["levels"];
  for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
  {
    Level level;
    level.from_yaml(it->first.as<string>(), it->second, coordinate_system);
    levels.push_back(level);
  }

  QtConcurrent::blockingMap(
    levels,
    [&](auto& level) { level.load_drawing(); });

  // now that all images are loaded, we can calculate scale for annotated
  // measurement lanes
  for (auto& level : levels)
    level.calculate_scale(coordinate_system);

  lifts.clear();
  if (y["lifts"] && y["lifts"].IsMap())
  {
    const YAML::Node& y_lifts = y["lifts"];
    for (YAML::const_iterator it = y_lifts.begin(); it != y_lifts.end(); ++it)
    {
      Lift lift;
      lift.from_yaml(it->first.as<string>(), it->second, levels);
      lifts.push_back(lift);
    }
  }

  if (y["graphs"] && y["graphs"].IsMap())
  {
    const YAML::Node& g_map = y["graphs"];
    for (YAML::const_iterator it = g_map.begin(); it != g_map.end(); ++it)
    {
      Graph graph;
      graph.from_yaml(it->first.as<int>(), it->second);
      graphs.push_back(graph);
    }
  }

  if (y["parameters"] && y["parameters"].IsMap())
  {
    const YAML::Node& gp = y["parameters"];
    for (YAML::const_iterator it = gp.begin(); it != gp.end(); ++it)
    {
      Param p;
      p.from_yaml(it->second);
      params[it->first.as<string>()] = p;
    }
  }

  calculate_all_transforms();
  return true;
}

bool Building::save()
{
  printf("Building::save_yaml(%s)\n", filename.c_str());

  YAML::Node y;
  y["name"] = name;

  y["coordinate_system"] = coordinate_system.to_string();

  if (!reference_level_name.empty())
    y["reference_level_name"] = reference_level_name;

  y["levels"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& level : levels)
    y["levels"][level.name] = level.to_yaml(coordinate_system);

  y["lifts"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& lift : lifts)
    y["lifts"][lift.name] = lift.to_yaml();
  if (lifts.empty())
    y["lifts"].SetStyle(YAML::EmitterStyle::Flow);

  if (crowd_sim_impl)
    y["crowd_sim"] = crowd_sim_impl->to_yaml();

  y["graphs"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& graph : graphs)
    y["graphs"][graph.idx] = graph.to_yaml();

  if (!params.empty())
  {
    YAML::Node params_node(YAML::NodeType::Map);
    for (const auto& param : params)
      params_node[param.first] = param.second.to_yaml();
    y["parameters"] = params_node;
  }

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  if (!fout)
  {
    printf("unable to open %s\n", filename.c_str());
    return false;
  }
  fout << emitter.c_str() << std::endl;

  return true;
}

bool Building::export_features(
  int level_index,
  const std::string& dest_filename) const
{
  return levels[level_index].export_features(dest_filename);
}

void Building::add_vertex(int level_index, double x, double y)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;
  levels[level_index].add_vertex(x, y);
}

QUuid Building::add_fiducial(int level_index, double x, double y)
{
  if (level_index >= static_cast<int>(levels.size()))
    return NULL;
  levels[level_index].fiducials.push_back(Fiducial(x, y));
  return levels[level_index].fiducials.rbegin()->uuid;
}

QUuid Building::add_feature(
  int level_idx,
  int layer_idx,
  double x,
  double y)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return NULL;
  return levels[level_idx].add_feature(layer_idx, x, y);
}

void Building::remove_feature(
  const int level_idx,
  const int layer_idx,
  QUuid feature_uuid)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  levels[level_idx].remove_feature(layer_idx, feature_uuid);
}

int Building::find_nearest_vertex_index(
  int level_index,
  double x,
  double y,
  double& distance)
{
  double min_dist = 1e100;
  int min_index = -1;
  for (std::size_t i = 0; i < levels[level_index].vertices.size(); i++)
  {
    const Vertex& v = levels[level_index].vertices[i];
    const double dx = x - v.x;
    const double dy = y - v.y;
    const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
    if (dist2 < min_dist)
    {
      min_dist = dist2;
      min_index = i;
    }
  }
  distance = sqrt(min_dist);
  return min_index;  // will be -1 if vertices vector is empty
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
    level_index,
    start_vertex_index,
    end_vertex_index,
    static_cast<int>(edge_type));

  levels[level_index].edges.push_back(
    Edge(start_vertex_index, end_vertex_index, edge_type));
}

void Building::add_lane(
  const int level_index,
  const int start_vertex_index,
  const int end_vertex_index,
  const int graph_idx)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;

  printf("Building::add_lane(%d, %d, %d, graph=%d)\n",
    level_index,
    start_vertex_index,
    end_vertex_index,
    graph_idx);
  Edge e(start_vertex_index, end_vertex_index, Edge::LANE);
  e.set_graph_idx(graph_idx);
  levels[level_index].edges.push_back(e);
}

bool Building::delete_selected(const int level_index)
{
  if (level_index >= static_cast<int>(levels.size()))
    return false;

  if (!levels[level_index].delete_selected())
    return false;

  return true;
}

QUuid Building::add_model(
  const int level_idx,
  const double x,
  const double y,
  const double z,
  const double yaw,
  const std::string& model_name)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return NULL;

  printf("Building::add_model(%d, %.1f, %.1f, %.1f, %.2f, %s)\n",
    level_idx, x, y, z, yaw, model_name.c_str());
  Model m;
  m.state.x = x;
  m.state.y = y;
  m.state.z = z;
  m.state.yaw = yaw;
  m.model_name = model_name;
  m.instance_name = model_name;  // todo: add unique numeric suffix?
  m.is_static = true;
  m.is_dispensable = false;
  levels[level_idx].models.push_back(m);
  return levels[level_idx].models.rbegin()->uuid;
}

void Building::set_model_yaw(
  const int level_idx,
  const int model_idx,
  const double yaw)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  levels[level_idx].models[model_idx].state.yaw = yaw;
}

void Building::clear()
{
  name.clear();
  filename.clear();
  reference_level_name.clear();
  levels.clear();
  lifts.clear();
  clear_transform_cache();
}

void Building::add_level(const Level& new_level)
{
  // make sure we don't have this level already
  for (const auto& level : levels)
  {
    if (level.name == new_level.name)
      return;
  }
  levels.push_back(new_level);
}

void Building::draw_lifts(QGraphicsScene* scene, const int level_idx)
{
  const Level& level = levels[level_idx];
  for (const auto& lift : lifts)
  {
    // find the level index referenced by the lift
    int reference_floor_idx = -1;
    for (std::size_t i = 0; i < levels.size(); i++)
    {
      if (levels[i].name == lift.reference_floor_name)
      {
        reference_floor_idx = static_cast<int>(i);
        break;
      }
    }

    Transform t;
    if (reference_floor_idx >= 0)
      t = get_transform(reference_floor_idx, level_idx);

    lift.draw(
      scene,
      level.drawing_meters_per_pixel,
      level.name,
      level.elevation,
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
  for (std::size_t i = 0; i < levels.size(); i++)
  {
    if (levels[i].name == from_level_name)
      from_level_idx = i;
    if (levels[i].name == to_level_name)
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
  const Level& from_level = levels[from_level_idx];
  const Level& to_level = levels[to_level_idx];

  // assemble a vector of fudicials in common to these levels
  vector<std::pair<Fiducial, Fiducial>> fiducials;
  for (const Fiducial& f0 : from_level.fiducials)
  {
    for (const Fiducial& f1 : to_level.fiducials)
    {
      if (f0.name == f1.name)
      {
        fiducials.push_back(make_pair(f0, f1));
        break;
      }
    }
  }

  // calculate the distances between each fiducial on their levels
  vector<std::pair<double, double>> distances;
  for (std::size_t f0_idx = 0; f0_idx < fiducials.size(); f0_idx++)
  {
    for (std::size_t f1_idx = f0_idx + 1; f1_idx < fiducials.size(); f1_idx++)
    {
      distances.push_back(
        make_pair(
          fiducials[f0_idx].first.distance(fiducials[f1_idx].first),
          fiducials[f0_idx].second.distance(fiducials[f1_idx].second)));
    }
  }

  if (distances.empty())
    return Building::Transform();

  // for now, we'll just compute the mean of the relative scale estimates.
  // we can do fancier statistics later, if needed.
  double relative_scale_sum = 0;
  for (std::size_t i = 0; i < distances.size(); i++)
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
    return;// let's not crash

  clear_transform_cache();
  for (std::size_t i = 0; i < levels.size(); i++)
  {
    for (std::size_t j = 0; j < levels.size(); j++)
    {
      get_transform(i, j);
    }
  }

  // set drawing scale using this data
  const int ref_idx = get_reference_level_idx();
  const double ref_scale = levels[ref_idx].drawing_meters_per_pixel;
  for (int i = 0; i < static_cast<int>(levels.size()); i++)
  {
    if (i != get_reference_level_idx())
    {
      Transform t = get_transform(ref_idx, i);
      if (levels[i].fiducials.size() >= 2)
        levels[i].drawing_meters_per_pixel = ref_scale / t.scale;
    }
  }
}

int Building::get_reference_level_idx()
{
  if (reference_level_name.empty())
    return 0;
  for (std::size_t i = 0; i < levels.size(); i++)
  {
    if (levels[i].name == reference_level_name)
      return static_cast<int>(i);
  }
  return 0;
}

void Building::clear_scene()
{
  for (auto& level : levels)
    level.clear_scene();
}

double Building::level_meters_per_pixel(const string& level_name) const
{
  for (const auto& level : levels)
  {
    if (level.name == level_name)
      return level.drawing_meters_per_pixel;
  }
  return 0.05;  // just a somewhat sane default
}

void Building::rotate_all_models(const double rotation)
{
  for (auto& level : levels)
  {
    for (auto& model : level.models)
    {
      model.state.yaw += rotation;
    }
  }
}

bool Building::set_filename(const std::string& _fn)
{
  const string suffix(".building.yaml");

  // ensure there is at least one character in addition to the suffix length
  if (_fn.size() <= suffix.size())
  {
    printf("Building::set_filename() too short: [%s]\n", _fn.c_str());
    return false;
  }

  // ensure the filename ends in .building.yaml
  // it should, because the "save as" dialog appends it, but...
  if (_fn.compare(_fn.size() - suffix.size(), suffix.size(), suffix))
  {
    printf(
      "Building::set_filename() filename had unexpected suffix: [%s]\n",
      _fn.c_str());
    return false;
  }

  const string no_suffix(_fn.substr(0, _fn.size() - suffix.size()));

  const std::size_t last_slash_pos = no_suffix.rfind('/', no_suffix.size());

  const string stem(
    (last_slash_pos == string::npos) ?
    no_suffix :
    string(no_suffix, last_slash_pos + 1));

  filename = _fn;

  if (name.empty())
    name = stem;

  printf(
    "set building filename to [%s]\n",
    filename.c_str());
  return true;
}

void Building::clear_selection(const int level_idx)
{
  if (levels.empty())
    return;
  levels[level_idx].clear_selection();
}

bool Building::can_delete_current_selection(const int level_idx)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return false;
  return levels[level_idx].can_delete_current_selection();
}

void Building::get_selected_items(
  const int level_idx,
  std::vector<Level::SelectedItem>& selected)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;
  levels[level_idx].get_selected_items(selected);
}

void Building::draw(
  QGraphicsScene* scene,
  const int level_idx,
  std::vector<EditorModel>& editor_models,
  const RenderingOptions& rendering_options)
{
  if (levels.empty())
  {
    printf("nothing to draw!\n");
    return;
  }

  levels[level_idx].draw(
    scene,
    editor_models,
    rendering_options,
    graphs,
    coordinate_system);

  draw_lifts(scene, level_idx);
}

Polygon* Building::get_selected_polygon(const int level_idx)
{
  for (std::size_t i = 0; i < levels[level_idx].polygons.size(); i++)
  {
    if (levels[level_idx].polygons[i].selected)
      return &levels[level_idx].polygons[i];// abomination
  }
  return nullptr;
}

Polygon::EdgeDragPolygon Building::polygon_edge_drag_press(
  const int level_idx,
  const Polygon* polygon,
  const double x,
  const double y)
{
  Polygon::EdgeDragPolygon edp;

  if (level_idx < 0 || level_idx >= static_cast<int>(levels.size()))
    return edp;

  return levels[level_idx].polygon_edge_drag_press(polygon, x, y);
}

void Building::add_constraint(
  const int level_idx,
  const QUuid& a,
  const QUuid& b)
{
  if (level_idx < 0 || level_idx >= static_cast<int>(levels.size()))
    return;
  levels[level_idx].add_constraint(a, b);
}

void Building::remove_constraint(
  const int level_idx,
  const QUuid& a,
  const QUuid& b)
{
  if (level_idx < 0 || level_idx >= static_cast<int>(levels.size()))
    return;
  levels[level_idx].remove_constraint(a, b);
}

int Building::nearest_item_index_if_within_distance(
  const double level_idx,
  const double x,
  const double y,
  const double distance_threshold,
  const Level::ItemType item_type)
{
  if (level_idx >= levels.size())
    return -1;

  return levels[level_idx].nearest_item_index_if_within_distance(
    x,
    y,
    distance_threshold,
    item_type);
}
