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

#ifndef LAYER_GRAPHICS_EFFECT_HPP
#define LAYER_GRAPHICS_EFFECT_HPP

#include <QGraphicsEffect>

class QPainter;

class LayerGraphicsEffect : public QGraphicsEffect
{
public:
  LayerGraphicsEffect();
  virtual ~LayerGraphicsEffect();

protected:
  virtual void draw(QPainter* painter);
};

#endif
