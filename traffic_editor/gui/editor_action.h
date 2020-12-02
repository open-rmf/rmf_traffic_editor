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

#ifndef EDITOR_ACTION_H
#define EDITOR_ACTION_H

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

enum ParameterType 
{
    FLOAT=1,
    STRING=2,
    INT=3
};

struct EditorParam 
{
    ParameterType pt;
    std::string str_val;
    float flt_val;
    int int_val;
};


typedef std::unordered_map<std::string, EditorParam> EditorParams;

class EditorAction 
{
public:
    EditorAction(std::function<void(EditorParams)> action, std::function<void(EditorParams)> undo_action, EditorParams type);
    void undo();
    void redo();

private:

    std::function<void(EditorParams)> _action;
    std::function<void(EditorParams)> _undo_action;
    EditorParams _params; // Think about it later
};
#endif