/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef CALLBACKS__H
#define CALLBACKS__H

#include <iostream>

template <typename T>
class Callbacks
{
  public:
  Callbacks(std::vector<std::pair<void (T::*)(), T*>> callbacks):_callbacks(callbacks){}
  ~Callbacks(){}
  
  void initiate(){
    for(auto callback: _callbacks)
    {
      auto obj = callback.second;
      auto func = callback.first;
      (obj->*func)();
    }
  }

  std::vector<std::pair<void (T::*)(), T*>> _callbacks;
};

#endif