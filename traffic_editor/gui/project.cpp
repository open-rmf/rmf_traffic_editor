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

#include "project.h"
#include <QFileInfo>
#include <QDir>

using std::string;


Project::Project()
{
}

Project::~Project()
{
}

bool Project::load_yaml_file(const std::string& _filename)
{
  filename = _filename;

  // This function may throw exceptions. Caller should be ready for them!
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

  return true;
}

bool Project::save_yaml_file() const
{
  return true;
}
