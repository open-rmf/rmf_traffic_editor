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

#ifndef RENDERING_OPTIONS_H
#define RENDERING_OPTIONS_H

#include <array>

class RenderingOptions
{
public:
  static const int NUM_BUILDING_LANES = 10;
  std::array<bool, NUM_BUILDING_LANES> show_building_lanes;

  bool show_models = true;
  int active_traffic_map_idx = 0;

  RenderingOptions();
};

#endif
