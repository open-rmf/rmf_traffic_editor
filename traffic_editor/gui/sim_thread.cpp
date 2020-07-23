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

#include "sim_thread.h"
#include <QTimer>
#include "editor.h"

SimThread::SimThread()
: QThread()
{
}

SimThread::~SimThread()
{
}

void SimThread::run()
{
  printf("entering SimThread::run()\n");
  while (true)
  {
    usleep(100);  // todo: user-editable parameter here from GUI...
    if (isInterruptionRequested())
      break;
    Editor::get_instance()->sim_tick();
  }
}
