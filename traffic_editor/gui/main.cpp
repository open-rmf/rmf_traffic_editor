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

#include "editor.h"
#include "preferences_keys.h"
#include <QtWidgets>
#include <string>
#include <QSettings>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  app.setOrganizationName("open-robotics");
  app.setOrganizationDomain("openrobotics.org");
  app.setApplicationName("traffic-editor");
  app.setApplicationDisplayName("Traffic Editor");

  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addPositionalArgument("[file]", "File to open");
  parser.process(QCoreApplication::arguments());

  Editor editor;
  QString filename;
  QSettings settings;

  const bool load_previous = settings.value(
      preferences_keys::open_previous_file, QVariant(true)).toBool();

  if (!parser.positionalArguments().isEmpty())
    editor.load_project(parser.positionalArguments().front());
  else if (load_previous)
    editor.load_previous_project();

  editor.show();

  return app.exec();
}
