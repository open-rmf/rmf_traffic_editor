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

#include <QtGlobal>
#include <QGraphicsPixmapItem>
#include <QGraphicsColorizeEffect>

#include "model.h"
using std::string;

// String comparison helper
bool iequals(const string& a, const string& b)
{
  return std::equal(a.begin(), a.end(),
      b.begin(), b.end(),
      [](char _a, char _b)
      {
        return tolower(_a) == tolower(_b);
      });
}

Model::Model()
{
  uuid = QUuid::createUuid();
}

void Model::from_yaml(
  const YAML::Node& data,
  const string& level_name,
  const CoordinateSystem& coordinate_system)
{
  if (!data.IsMap())
    throw std::runtime_error("Model::from_yaml() expected a map");

  if (!coordinate_system.is_global())
  {
    state.x = data["x"].as<double>();
    state.y = data["y"].as<double>();
  }
  else
  {
    CoordinateSystem::WGS84Point wgs84_point;
    wgs84_point.lon = data["x"].as<double>();
    wgs84_point.lat = data["y"].as<double>();

    CoordinateSystem::ProjectedPoint p =
      coordinate_system.to_epsg3857(wgs84_point);
    state.x = p.x;
    state.y = p.y;
  }

  if (data["z"])
  {
    state.z = data["z"].as<double>();
  }
  else
  {
    qWarning(
      "parsed a deprecated .building.yaml, models should have z defined.");
    state.z = 0.0;
  }
  state.yaw = data["yaw"].as<double>();

  model_name = data["model_name"].as<string>();
  instance_name = data["name"].as<string>();

  state.level_name = level_name;
  starting_level = level_name;

  if (data["static"])
    is_static = data["static"].as<bool>();
  else
    is_static = true;

  if (data["dispensable"])
  {
    is_dispensable = data["dispensable"].as<bool>();
  }
  else
  {
    // Although the dispensable parameter is needed, its default value of false
    // does not break any behavior downstream, as long as future saves populate
    // that in the output yaml file. No warning needs to be printed.
    is_dispensable = false;
  }
}

YAML::Node Model::to_yaml(const CoordinateSystem& coordinate_system) const
{
  YAML::Node n;
  n.SetStyle(YAML::EmitterStyle::Flow);

  if (!coordinate_system.is_global())
  {
    // in either image or cartesian-meters coordinate spaces, we're
    // fine with rounding to 3 decimal places
    n["x"] = std::round(state.x * 1000.0) / 1000.0;
    n["y"] = std::round(state.y * 1000.0) / 1000.0;
  }
  else
  {
    // convert back to WGS84 and save with as many decimal places as possible
    CoordinateSystem::WGS84Point p =
      coordinate_system.to_wgs84({state.x, state.y});
    n["x"] = p.lon;
    n["y"] = p.lat;
  }
  n["z"] = std::round(state.z * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = std::round(state.yaw * 10000.0) / 10000.0;
  n["name"] = instance_name;
  n["model_name"] = model_name;
  n["static"] = is_static;
  n["dispensable"] = is_dispensable;
  return n;
}

void Model::set_param(const std::string& name, const std::string& value)
{
  if (name == "elevation")
  {
    try
    {
      state.z = std::stod(value);
    }
    catch (const std::exception& e)
    {
      qWarning("[elevation] field can only be a double/float.");
    }
  }
  else if (name == "static")
  {
    // not sure if there is a super elite way to parse 'true' in STL
    string lowercase(value);
    std::transform(
      lowercase.begin(),
      lowercase.end(),
      lowercase.begin(),
      [](char c) { return std::tolower(c); });

    if (value == "true")
      is_static = true;
    else
      is_static = false;
  }
  else if (name == "dispensable")
  {
    // not sure if there is a super elite way to parse 'true' in STL
    string lowercase(value);
    std::transform(
      lowercase.begin(),
      lowercase.end(),
      lowercase.begin(),
      [](char c) { return std::tolower(c); });

    if (value == "true")
      is_dispensable = true;
    else
      is_dispensable = false;
  }
  else if (name == "name")
  {
    instance_name = value;
  }
  else
  {
    printf("WARNING: setting unknown model parameter: [%s]\n", name.c_str());
  }
}

void Model::draw(
  QGraphicsScene* scene,
  std::vector<EditorModel>& editor_models,
  const double drawing_meters_per_pixel)
{
  if (pixmap_item == nullptr)
  {
    // find the pixmap we need for this model
    QPixmap pixmap;
    double model_meters_per_pixel = 1.0;  // will get overridden
    for (auto& editor_model : editor_models)
    {
      if (editor_model.name == model_name)
      {
        pixmap = editor_model.get_pixmap();
        model_meters_per_pixel = editor_model.meters_per_pixel;
        break;
      }
    }
    if (pixmap.isNull())
    {
      // BACKWARDS COMPATIBILITY PATCH: Try again, but...
      // Use the first matching namespaced thumbnail for a
      // specified non-namespaced model, with warnings.

      // (Also modifies the model name inplace!)
      for (auto& editor_model : editor_models)
      {
        // Get ending token
        std::string ending_token;
        std::size_t delimiter_index = editor_model.name.find("/");

        if (delimiter_index != std::string::npos)
        {
          ending_token = editor_model.name
            .substr(delimiter_index + 1, editor_model.name.length());
        }
        else
        {
          ending_token = editor_model.name;
        }

        // Check if namespaced model_name is the name we are looking for
        // Match mismatched cases
        if (iequals(ending_token, model_name))
        {
          // Skip rematches from previous for loop
          if (model_name == editor_model.name)
            continue;

          pixmap = editor_model.get_pixmap();
          model_meters_per_pixel = editor_model.meters_per_pixel;

          printf("\n[WARNING] Thumbnail %1$s not found, "
            "substituting %2$s instead!\n"
            "(%1$s will be saved as %2$s)\n\n",
            model_name.c_str(), editor_model.name.c_str());

          // And reassign it!
          model_name = editor_model.name;
          break;
        }
      }

      // Check again for pixmap find status
      if (pixmap.isNull())
      {
        if (!error_printed)
        {
          printf("[ERROR] No thumbnail found: %s\n", model_name.c_str());
          error_printed = true;
        }
        return;  // couldn't load the pixmap; ignore it.
      }
    }

    pixmap_item = scene->addPixmap(pixmap);
    pixmap_item->setOffset(-pixmap.width()/2, -pixmap.height()/2);
    pixmap_item->setScale(model_meters_per_pixel / drawing_meters_per_pixel);
    pixmap_item->setZValue(100.0);  // just anything taller than 0
  }

  pixmap_item->setPos(state.x, state.y);
  pixmap_item->setRotation((-state.yaw + M_PI / 2.0) * 180.0 / M_PI);

  // make the model "glow" if it is selected
  if (selected)
  {
    QGraphicsColorizeEffect* colorize = new QGraphicsColorizeEffect;
    colorize->setColor(QColor::fromRgbF(1.0, 0.2, 0.0, 1.0));
    colorize->setStrength(1.0);
    pixmap_item->setGraphicsEffect(colorize);
  }
}

void Model::clear_scene()
{
  pixmap_item = nullptr;
}
