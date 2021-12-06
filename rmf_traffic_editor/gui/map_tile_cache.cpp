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

#include "map_tile_cache.h"

MapTileCache::MapTileCache()
{
  // use a deque so that LRU cleanup of the cache is fast?
  // or use an unordered_map so that lookup is fast?
}

MapTileCache::~MapTileCache()
{
}

bool MapTileCache::contains(const int zoom, const int x, const int y) const
{
  return false;
}

const QPixmap* const MapTileCache::get(const int zoom, const int x, const int y) const
{
  return nullptr;
}

void MapTileCache::set(const int zoom, const int x, const int y, QPixmap* item)
{
}
