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

#ifndef LIFT_PREVIEW_WIDGET_H
#define LIFT_PREVIEW_WIDGET_H

#include <QWidget>
#include "lift.h"

class LiftPreviewWidget : public QWidget
{
  Q_OBJECT

public:
  LiftPreviewWidget(const Lift& lift);
  ~LiftPreviewWidget();

private:
  const Lift& _lift;

  void paintEvent(QPaintEvent *e);
};

#endif
