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

int main(int argc, char* argv[])
{
  QApplication app(argc, argv);
  app.setOrganizationName("open-robotics");
  app.setOrganizationDomain("openrobotics.org");
  app.setApplicationName("traffic-editor");

  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addPositionalArgument("[project]", "Project to open");
  parser.process(QCoreApplication::arguments());

  Editor editor;
  QSettings settings;

  const bool load_previous = settings.value(
    preferences_keys::open_previous_project, QVariant(true)).toBool();

  if (load_previous && parser.positionalArguments().isEmpty())
    editor.load_previous_project();

  if (parser.positionalArguments().length() >= 1)
    editor.load_project(parser.positionalArguments().at(0));

  editor.show();

  editor.restore_previous_viewport();

  return app.exec();
}
