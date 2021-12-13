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
  std::string tile_cache_root = QStandardPaths::writableLocation(
    QStandardPaths::CacheLocation).toStdString();
  printf("tile_cache_root: %s\n", tile_cache_root.c_str());
}

MapTileCache::~MapTileCache()
{
}

std::optional<const QPixmap> MapTileCache::get(
  const int zoom,
  const int x,
  const int y) const
{
  for (auto it = cache.begin(); it != cache.end(); ++it)
  {
    if (it->zoom == zoom && it->x == x && it->y == y)
      return {it->pixmap};
  }
  return {};
}

void MapTileCache::set(
  const int zoom,
  const int x,
  const int y,
  const QPixmap& pixmap)
{
  for (auto it = cache.begin(); it != cache.end(); ++it)
  {
    if (it->zoom == zoom && it->x == x && it->y == y)
    {
      it->pixmap = pixmap;
      return;
    }
  }
  // if we get here, we didn't find it, so we should create a cache entry
  MapTileCacheElement e;
  e.zoom = zoom;
  e.x = x;
  e.y = y;
  e.pixmap = pixmap;

  cache.push_front(e);
  
  if (cache.size() > MAX_CACHE_SIZE)
  {
    printf("cache has %d elements. trimming oldest element...\n", (int)cache.size());
    //MapTileCacheElement old = cache.back();
    //printf("about to delete z=%d, x=%d, y=%d...\n", old.zoom, old.x, old.y);
    //delete old.pixmap;
    //printf("done\n");
    cache.pop_back();
  }
}
