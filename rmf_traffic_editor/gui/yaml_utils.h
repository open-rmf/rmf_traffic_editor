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

#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <yaml-cpp/yaml.h>

namespace yaml_utils {

// Recursive function to write YAML ordered maps. Credit: Dave Hershberger
void write_node(const YAML::Node& node, YAML::Emitter& emitter);

}

#endif
