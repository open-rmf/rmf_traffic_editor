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

#include <QDir>
#include <QImage>
#include <QImageReader>
#include <QSettings>

#include "traffic_editor/editor_model.h"

using std::string;


EditorModel::EditorModel(const string _name, const double _meters_per_pixel)
: name(_name),
  name_lowercase(_name),
  meters_per_pixel(_meters_per_pixel)
{
  // make a lowercase copy for fast auto-complete
  std::transform(
    name_lowercase.begin(),
    name_lowercase.end(),
    name_lowercase.begin(),
    [](unsigned char c) { return std::tolower(c); });
}

EditorModel::~EditorModel()
{
}

QPixmap EditorModel::get_pixmap()
{
  if (!pixmap.isNull())
    return pixmap;

  // if we get here, we have to load the image from disk and generate pixmap

  const QString THUMBNAIL_PATH_KEY("editor/thumbnail_path");
  QSettings settings;
  QString thumbnail_path(settings.value(THUMBNAIL_PATH_KEY).toString());

  string filename =
    thumbnail_path.toStdString() +
    "/images/cropped/" +
    name +
    string(".png");
  // qInfo("loading: [%s]", filename.c_str());
  QImageReader image_reader(QString::fromStdString(filename));
  image_reader.setAutoTransform(true);  // not sure what this does
  QImage image = image_reader.read();
  if (image.isNull())
  {
    qWarning("unable to read %s: %s",
      filename.c_str(),
      qUtf8Printable(image_reader.errorString()));
    return QPixmap();
  }
  pixmap = QPixmap::fromImage(image);
  return pixmap;
}
