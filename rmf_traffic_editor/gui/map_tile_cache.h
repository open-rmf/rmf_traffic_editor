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

#ifndef TRAFFIC_EDITOR_MAP_TILE_CACHE_H
#define TRAFFIC_EDITOR_MAP_TILE_CACHE_H

#include <deque>
#include <optional>

#include <QPixmap>

class MapTileCache
{
public:
  MapTileCache();
  ~MapTileCache();

  std::optional<const QByteArray> get(
    const int zoom,
    const int x,
    const int y) const;

  void set(
    const int zoom,
    const int x,
    const int y,
    const QByteArray& bytes);

  struct CacheSize
  {
    int bytes = 0;
    int files = 0;
  };

  CacheSize getSize();

private:
  /*
  struct MapTileCacheElement
  {
    int zoom = 0;
    int x = 0;
    int y = 0;
    QPixmap pixmap;
  };
  */

  // std::deque<MapTileCacheElement> cache;
  // const size_t MAX_CACHE_SIZE = 1000;

  QString tile_cache_root;

  QString tile_path(
    int zoom,
    int x,
    int y) const;

  bool modified_since_last_size_check = true;  // for an initial size check
  CacheSize last_size = {0, 0};
};

#endif
