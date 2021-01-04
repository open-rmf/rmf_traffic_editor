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

#include <cmath>
#include <fstream>

#include "project.h"
#include "yaml_utils.h"

#include <QFileInfo>
#include <QDir>
#include <QGraphicsItem>

using std::string;
using std::vector;


Project::Project()
{
  for (size_t i = 0; i < rendering_options.show_building_lanes.size(); i++)
    rendering_options.show_building_lanes[i] = true;
}

Project::~Project()
{
}

bool Project::load_yaml_file(const std::string& _filename)
{
  filename = _filename;

  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(filename.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", filename.c_str(), e.what());
    return false;
  }

  // change directory to the path of the file, so that we can correctly open
  // relative paths recorded in the file

  // TODO: save previous directory and restore it, in case other files
  // we load are in different directories (!)

  QString dir(QFileInfo(QString::fromStdString(filename)).absolutePath());
  printf("changing directory to [%s]", qUtf8Printable(dir));
  if (!QDir::setCurrent(dir))
  {
    printf("couldn't change directory\n");
    return false;
  }

  if (yaml["name"])
    name = yaml["name"].as<string>();

  if (yaml["building"] && yaml["building"].IsMap())
  {
    if (!yaml["building"]["filename"])
    {
      printf("expected a 'filename' key within the 'building' map\n");
      return false;
    }
    building.filename = yaml["building"]["filename"].as<string>();
    if (!building.load_yaml_file())
      return false;
  }

  if (yaml["scenarios"] && yaml["scenarios"].IsSequence())
  {
    for (YAML::const_iterator scenario_node = yaml["scenarios"].begin();
      scenario_node != yaml["scenarios"].end();
      ++scenario_node)
    {
      std::unique_ptr<Scenario> scenario(new Scenario);
      scenario->filename = (*scenario_node)["filename"].as<string>();
      if (!scenario->load())
      {
        printf("couldn't load [%s]\n", scenario->filename.c_str());
        //return false;
      }
      scenarios.push_back(std::move(scenario));
    }
    if (!scenarios.empty())
      scenario_idx = 0;
  }

  if (yaml["traffic_maps"] && yaml["traffic_maps"].IsMap())
  {
    const YAML::Node& ytm = yaml["traffic_maps"];
    for (YAML::const_iterator it = ytm.begin(); it != ytm.end(); ++it)
    {
      TrafficMap tm;
      tm.from_project_yaml(it->first.as<string>(), it->second);
      traffic_maps.push_back(tm);
    }
  }

  return true;
}

bool Project::save_yaml_file() const
{
  printf("Project::save_yaml_file()\n");

  YAML::Node y;
  y["version"] = 1;
  y["name"] = name;

  y["building"] = YAML::Node(YAML::NodeType::Map);
  y["building"]["filename"] = building.filename;

  for (const auto& scenario : scenarios)
  {
    printf("saving scenario\n");
    YAML::Node scenario_node;
    scenario_node["filename"] = scenario->filename;
    y["scenarios"].push_back(scenario_node);
  }

  y["traffic_maps"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& traffic_map : traffic_maps)
    y["traffic_maps"][traffic_map.name] = traffic_map.to_project_yaml();

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  return true;
}

bool Project::save()
{
  if (!save_yaml_file())
    return false;

  building.save_yaml_file();

  /*
  // TODO: currently the scenarios are not fully parsed, so we
  // can't save them back to disk without data loss
  for (const auto& scenario : scenarios)
    if (!scenario->save())
      return false;
  */

  return true;
}

bool Project::load(const std::string& _filename)
{
  // future extension point: dispatch based on file type (json/yaml/...)
  return load_yaml_file(_filename);
}

void Project::add_scenario_vertex(
  const int level_idx,
  const double x,
  const double y)
{
  printf("add_scenario_vertex(%d, %.3f, %.3f)\n", level_idx, x, y);
  if (scenario_idx < 0 || scenario_idx >= static_cast<int>(scenarios.size()))
    return;
  scenarios[scenario_idx]->add_vertex(building.levels[level_idx].name, x, y);
}

void Project::scenario_row_clicked(const int row)
{
  printf("Project::scenario_row_clicked(%d)\n", row);
  if (row < 0 || row >= static_cast<int>(scenarios.size()))
  {
    scenario_idx = -1;
    return;
  }
  scenario_idx = row;
}

void Project::draw(
  QGraphicsScene* scene,
  const int level_idx,
  std::vector<EditorModel>& editor_models)
{
  std::lock_guard<std::mutex> building_guard(building.building_mutex);

  if (building.levels.empty())
  {
    printf("nothing to draw!\n");
    return;
  }

  building.levels[level_idx].draw(scene, editor_models, rendering_options);
  building.draw_lifts(scene, level_idx);

  if (scenario_idx >= 0)
    scenarios[scenario_idx]->draw(
      scene,
      building.levels[level_idx].name,
      building.levels[level_idx].drawing_meters_per_pixel,
      editor_models);
}

void Project::clear_selection(const int level_idx)
{
  if (building.levels.empty())
    return;
  building.levels[level_idx].clear_selection();

  if (scenario_idx >= 0)
    scenarios[scenario_idx]->clear_selection(building.levels[level_idx].name);
}

bool Project::can_delete_current_selection(const int level_idx)
{
  if (level_idx >= static_cast<int>(building.levels.size()))
    return false;
  return building.levels[level_idx].can_delete_current_selection();
}

bool Project::delete_selected(const int level_idx)
{
  if (building.levels.empty())
    return false;
  if (!building.delete_selected(level_idx))
    return false;
  const std::string level_name = building.levels[level_idx].name;
  if (scenario_idx >= 0 &&
    !scenarios[scenario_idx]->delete_selected(level_name))
    return false;
  return true;
}

void Project::get_selected_items(
  const int level_idx,
  std::vector<BuildingLevel::SelectedItem>& selected)
{
  building.levels[level_idx].get_selected_items(selected);
}

Project::NearestItem Project::nearest_items(
  EditorModeId mode,
  const int level_index,
  const double x,
  const double y)
{
  NearestItem ni;

  if (level_index >= static_cast<int>(building.levels.size()))
    return ni;
  const BuildingLevel& building_level = building.levels[level_index];

  if (mode == MODE_BUILDING)
  {
    for (size_t i = 0; i < building_level.vertices.size(); i++)
    {
      const Vertex& p = building_level.vertices[i];
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double dist = std::sqrt(dx*dx + dy*dy);
      if (dist < ni.vertex_dist)
      {
        ni.vertex_dist = dist;
        ni.vertex_idx = i;
      }
    }

    for (size_t i = 0; i < building_level.fiducials.size(); i++)
    {
      const Fiducial& f = building_level.fiducials[i];
      const double dx = x - f.x;
      const double dy = y - f.y;
      const double dist = std::sqrt(dx*dx + dy*dy);
      if (dist < ni.fiducial_dist)
      {
        ni.fiducial_dist = dist;
        ni.fiducial_idx = i;
      }
    }

    for (size_t i = 0; i < building_level.models.size(); i++)
    {
      const Model& m = building_level.models[i];
      const double dx = x - m.state.x;
      const double dy = y - m.state.y;
      const double dist = std::sqrt(dx*dx + dy*dy);  // no need for sqrt each time
      if (dist < ni.model_dist)
      {
        ni.model_dist = dist;
        ni.model_idx = i;
      }
    }
  }
  else if (mode == MODE_SCENARIO)
  {
    if (scenario_idx < 0 ||
      scenario_idx >= static_cast<int>(scenarios.size()))
      return ni;
    const Scenario& scenario = *scenarios[scenario_idx];

    for (const ScenarioLevel& scenario_level : scenario.levels)
    {
      if (scenario_level.name != building_level.name)
        continue;

      for (size_t i = 0; i < scenario_level.vertices.size(); i++)
      {
        const Vertex& p = scenario_level.vertices[i];
        const double dx = x - p.x;
        const double dy = y - p.y;
        const double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < ni.vertex_dist)
        {
          ni.vertex_dist = dist;
          ni.vertex_idx = i;
        }
      }
    }
  }

  return ni;
}

ScenarioLevel* Project::scenario_level(const int building_level_idx)
{
  if (building_level_idx >= static_cast<int>(building.levels.size()))
    return nullptr;
  const BuildingLevel& building_level = building.levels[building_level_idx];

  if (scenario_idx < 0 ||
    scenario_idx >= static_cast<int>(scenarios.size()))
    return nullptr;
  // I'm sure this is a horrific abomination. Fix someday.
  Scenario& scenario = *scenarios[scenario_idx];
  for (size_t i = 0; i < scenario.levels.size(); i++)
  {
    if (scenario.levels[i].name == building_level.name)
      return &scenario.levels[i];
  }
  return nullptr;
}

void Project::mouse_select_press(
  const EditorModeId mode,
  const int level_idx,
  const double x,
  const double y,
  QGraphicsItem* graphics_item)
{
  clear_selection(level_idx);
  const NearestItem ni = nearest_items(mode, level_idx, x, y);

  const double vertex_dist_thresh =
    building.levels[level_idx].vertex_radius /
    building.levels[level_idx].drawing_meters_per_pixel;

  if (mode == MODE_BUILDING)
  {
    // todo: use QGraphics stuff to see if we clicked a model pixmap...
    const double model_dist_thresh = 0.5 /
      building.levels[level_idx].drawing_meters_per_pixel;

    if (rendering_options.show_models &&
      ni.model_idx >= 0 &&
      ni.model_dist < model_dist_thresh)
      building.levels[level_idx].models[ni.model_idx].selected = true;
    else if (ni.vertex_idx >= 0 && ni.vertex_dist < vertex_dist_thresh)
      building.levels[level_idx].vertices[ni.vertex_idx].selected = true;
    else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
      building.levels[level_idx].fiducials[ni.fiducial_idx].selected = true;
    else
    {
      // use the QGraphics stuff to see if it's an edge segment or polygon
      if (graphics_item)
      {
        switch (graphics_item->type())
        {
          case QGraphicsLineItem::Type:
            set_selected_line_item(
              level_idx,
              qgraphicsitem_cast<QGraphicsLineItem*>(graphics_item),
              mode);
            break;

          case QGraphicsPolygonItem::Type:
            set_selected_containing_polygon(mode, level_idx, x, y);
            break;

          default:
            printf("clicked unhandled type: %d\n",
              static_cast<int>(graphics_item->type()));
            break;
        }
      }
    }
  }
  else if (mode == MODE_TRAFFIC || mode == MODE_CROWD_SIM)
  {
    // todo: keep traffic-map vertices separate from building vertices
    // for now, they're using the same vertex list.

    if (ni.vertex_idx >= 0 && ni.vertex_dist < vertex_dist_thresh)
      building.levels[level_idx].vertices[ni.vertex_idx].selected = true;
    else
    {
      // use the QGraphics stuff to see if it's an edge segment or polygon
      if (graphics_item)
      {
        switch (graphics_item->type())
        {
          case QGraphicsLineItem::Type:
            set_selected_line_item(
              level_idx,
              qgraphicsitem_cast<QGraphicsLineItem*>(graphics_item),
              mode);
            break;

          default:
            printf("clicked unhandled type: %d\n",
              static_cast<int>(graphics_item->type()));
            break;
        }
      }
    }
  }
  else if (mode == MODE_SCENARIO && scenario_idx >= 0)
  {
    ScenarioLevel* level = scenario_level(level_idx);
    if (ni.vertex_dist < 10.0)
      level->vertices[ni.vertex_idx].selected = true;
    else
    {
      // use the QGraphics stuff to see if it's an edge segment or polygon
      if (graphics_item)
      {
        switch (graphics_item->type())
        {
          case QGraphicsPolygonItem::Type:
            set_selected_containing_polygon(mode, level_idx, x, y);
            break;

          default:
            printf("clicked unhandled type: %d\n",
              static_cast<int>(graphics_item->type()));
            break;
        }
      }

    }
  }
}

void Project::set_selected_line_item(
  const int level_idx,
  QGraphicsLineItem* line_item,
  const EditorModeId mode)
{
  clear_selection(level_idx);

  if (line_item == nullptr)
    return;

  // find if any of our lanes match those vertices
  for (auto& edge : building.levels[level_idx].edges)
  {
    if (mode == MODE_TRAFFIC)
    {
      if (edge.type != Edge::LANE)
        continue;
      if (edge.get_graph_idx() != traffic_map_idx)
        continue;
    }

    if (mode == MODE_CROWD_SIM)
    {
      if (edge.type != Edge::HUMAN_LANE)
        continue;
      if (edge.get_graph_idx() != traffic_map_idx)
        continue;
    }

    if (mode == MODE_BUILDING &&
      (edge.type == Edge::LANE || edge.type == Edge::HUMAN_LANE) )
      continue;

    // look up the line's vertices
    const double x1 = line_item->line().x1();
    const double y1 = line_item->line().y1();
    const double x2 = line_item->line().x2();
    const double y2 = line_item->line().y2();

    const auto& v_start = building.levels[level_idx].vertices[edge.start_idx];
    const auto& v_end = building.levels[level_idx].vertices[edge.end_idx];

    // calculate distances
    const double dx1 = v_start.x - x1;
    const double dy1 = v_start.y - y1;
    const double dx2 = v_end.x - x2;
    const double dy2 = v_end.y - y2;
    const double v1_dist = std::sqrt(dx1*dx1 + dy1*dy1);
    const double v2_dist = std::sqrt(dx2*dx2 + dy2*dy2);

    const double thresh = 10.0;  // it should be really tiny if it matches
    if (v1_dist < thresh && v2_dist < thresh)
    {
      edge.selected = true;
      return;  // stop after first one is found, don't select multiple
    }
  }
}

Polygon::EdgeDragPolygon Project::polygon_edge_drag_press(
  const EditorModeId mode,
  const int level_idx,
  const Polygon* polygon,
  const double x,
  const double y)
{
  Polygon::EdgeDragPolygon edp;

  if (level_idx < 0 || level_idx > static_cast<int>(building.levels.size()))
    return edp;// oh no

  if (mode == MODE_BUILDING)
    return building.levels[level_idx].polygon_edge_drag_press(polygon, x, y);
  else if (mode == MODE_SCENARIO)
  {
    ScenarioLevel* slevel = scenario_level(level_idx);
    if (slevel == nullptr)
      return edp;
    return slevel->polygon_edge_drag_press(polygon, x, y);
  }

  return edp;
}

Polygon* Project::get_selected_polygon(
  const EditorModeId mode,
  const int level_idx)
{
  if (mode == MODE_BUILDING)
  {
    for (size_t i = 0; i < building.levels[level_idx].polygons.size(); i++)
    {
      if (building.levels[level_idx].polygons[i].selected)
        return &building.levels[level_idx].polygons[i];// abomination
    }
  }
  else if (mode == MODE_SCENARIO)
  {
    ScenarioLevel* slevel = scenario_level(level_idx);
    if (slevel)
    {
      for (size_t i = 0; i < slevel->polygons.size(); i++)
      {
        if (slevel->polygons[i].selected)
          return &slevel->polygons[i];// abomination
      }
    }
  }
  return nullptr;
}

void Project::set_selected_containing_polygon(
  const EditorModeId mode,
  const int level_idx,
  const double x,
  const double y)
{
  Level* level = nullptr;
  if (mode == MODE_BUILDING)
    level = &building.levels[level_idx];
  else if (mode == MODE_SCENARIO)
    level = scenario_level(level_idx);

  if (level == nullptr)
    return;

  // holes are "higher" in our Z-stack (to make them clickable), so first
  // we need to make a list of all polygons that contain this point.
  vector<Polygon*> containing_polygons;
  for (size_t i = 0; i < level->polygons.size(); i++)
  {
    Polygon& polygon = level->polygons[i];
    QVector<QPointF> polygon_vertices;
    for (const auto& vertex_idx: polygon.vertices)
    {
      const Vertex& v = level->vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    QPolygonF qpolygon(polygon_vertices);
    if (qpolygon.containsPoint(QPoint(x, y), Qt::OddEvenFill))
      containing_polygons.push_back(&level->polygons[i]);
  }

  // first search for holes
  for (Polygon* p : containing_polygons)
  {
    if (p->type == Polygon::HOLE)
    {
      p->selected = true;
      return;
    }
  }

  // if we get here, just return the first thing.
  for (Polygon* p : containing_polygons)
  {
    p->selected = true;
    return;
  }
}

void Project::clear()
{
  building.clear();
  name.clear();
  filename.clear();
  scenarios.clear();
  scenario_idx = -1;
}

#ifdef HAS_IGNITION_PLUGIN
void Project::sim_tick()
{
  if (scenario_idx < 0 || scenario_idx >= static_cast<int>(scenarios.size()))
    return;
  scenarios[scenario_idx]->sim_tick(building);
}

void Project::sim_reset()
{
  if (scenario_idx < 0 || scenario_idx >= static_cast<int>(scenarios.size()))
    return;
  scenarios[scenario_idx]->sim_reset(building);
}
#endif

void Project::clear_scene()
{
  building.clear_scene();

#ifdef HAS_IGNITION_PLUGIN
  for (auto& scenario : scenarios)
    scenario->clear_scene();
#endif
}

void Project::add_lane(
  const int level_idx,
  const int start_idx,
  const int end_idx)
{
  building.add_lane(level_idx, start_idx, end_idx, traffic_map_idx);
}

#ifdef HAS_IGNITION_PLUGIN
void Project::scenario_scene_update(
  QGraphicsScene* scene,
  const int level_idx)
{
  if (scenario_idx < 0 || scenario_idx >= static_cast<int>(scenarios.size()))
    return;
  scenarios[scenario_idx]->scene_update(scene, building, level_idx);
}

bool Project::has_sim_plugin()
{
  for (const auto& scenario : scenarios)
  {
    if (scenario->sim_plugin)
      return true;
  }
  return false;
}
#endif

bool Project::set_filename(const std::string& _fn)
{
  const string suffix(".project.yaml");

  // ensure there is at least one character in addition to the suffix length
  if (_fn.size() <= suffix.size())
  {
    printf("Project::set_filename() too short: [%s]\n", _fn.c_str());
    return false;
  }

  // ensure the filename ends in .project.yaml
  // it should, because the "save as" dialog appends it, but...
  if (_fn.compare(_fn.size() - suffix.size(), suffix.size(), suffix))
  {
    printf(
      "Project::set_filename() filename had unexpected suffix: [%s]\n",
      _fn.c_str());
    return false;
  }

  const string no_suffix(_fn.substr(0, _fn.size() - suffix.size()));

  const size_t last_slash_pos = no_suffix.rfind('/', no_suffix.size());

  const string stem(
    (last_slash_pos == string::npos) ?
    no_suffix :
    string(no_suffix, last_slash_pos + 1));

  filename = _fn;

  if (name.empty())
  {
    name = stem;
  }

  if (building.name.empty())
  {
    building.name = stem;
  }
  if (building.filename.empty())
  {
    building.filename = stem + std::string(".building.yaml");
  }

  printf(
    "set project filename to [%s] stem: [%s] building filename: [%s]\n",
    filename.c_str(),
    stem.c_str(),
    building.filename.c_str());
  return true;
}
