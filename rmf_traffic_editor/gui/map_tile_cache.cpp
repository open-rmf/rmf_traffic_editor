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

#include <QDir>
#include <QDirIterator>
#include <QSaveFile>
#include <QStandardPaths>
#include "map_tile_cache.h"

MapTileCache::MapTileCache()
{
  tile_cache_root =
    QStandardPaths::writableLocation(QStandardPaths::CacheLocation)
    + QDir::separator()
    + QString("tiles");
  printf("tile_cache_root: %s\n", tile_cache_root.toStdString().c_str());
  QDir tile_cache_dir(tile_cache_root);
  if (!tile_cache_dir.exists())
  {
    printf("creating tile cache directory in %s\n",
      tile_cache_root.toStdString().c_str());
    QDir::root().mkpath(tile_cache_root);
  }
  getSize();
}

MapTileCache::~MapTileCache()
{
}

std::optional<const QByteArray> MapTileCache::get(
  const int zoom,
  const int x,
  const int y) const
{
  QString path = tile_path(zoom, x, y);
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly))
  {
    // printf("cache miss: %d %d %d\n", zoom, x, y);
    return {};
  }
  // printf("cache hit: %s\n", path.toStdString().c_str());
  return {file.readAll()};
}

void MapTileCache::set(
  const int zoom,
  const int x,
  const int y,
  const QByteArray& bytes)
{
  QString path = tile_path(zoom, x, y);
  printf("cache write to %s\n", path.toStdString().c_str());
  QSaveFile save_file(path);
  save_file.open(QIODevice::WriteOnly);
  save_file.write(bytes);
  save_file.commit();

  modified_since_last_size_check = true;
  /*
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
    printf("cache has %d elements. trimming oldest element...\n",
      (int)cache.size());
    //MapTileCacheElement old = cache.back();
    //printf("about to delete z=%d, x=%d, y=%d...\n", old.zoom, old.x, old.y);
    //delete old.pixmap;
    //printf("done\n");
    cache.pop_back();
  }
  */
}

QString MapTileCache::tile_path(int zoom, int x, int y) const
{
  // sanitize the input...
  if (zoom < 0)
    zoom = 0;
  if (x < 0)
    x = 0;
  if (y < 0)
    y = 0;

  QString path = tile_cache_root
    + QDir::separator()
    + QString::number(zoom)
    + QString("_")
    + QString::number(y)
    + QString("_")
    + QString::number(x)
    + QString(".png");

  return path;
}

MapTileCache::CacheSize MapTileCache::getSize()
{
  if (!modified_since_last_size_check)
    return last_size;

  CacheSize size = {0, 0};
  QDirIterator it(tile_cache_root, QDir::Files);
  while (it.hasNext())
  {
    const QFileInfo info = it.fileInfo();
    it.next();

    size.files++;
    size.bytes += info.size();
  }
  printf("cache: %d files, %.3f MB\n", size.files, size.bytes / 1.0e6);
  modified_since_last_size_check = false;
  last_size = size;
  return size;
}
