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

#ifndef EDITOR_MODEL_H
#define EDITOR_MODEL_H

/*
 * Represents a simulation model class and related helpers for rendering.
 */

#include <string>
#include <QPixmap>

class EditorModel
{
public:
  EditorModel(const std::string _name, const double _meters_per_pixel);
  ~EditorModel();

  std::string name, name_lowercase;
  QPixmap pixmap;
  double meters_per_pixel;

  QPixmap get_pixmap();  // will load if needed
};

#endif
