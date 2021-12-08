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

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <vector>

#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QWheelEvent>

#include "building.h"
#include "map_tile_cache.h"

class MapView : public QGraphicsView
{
  Q_OBJECT

public:
  MapView(QWidget* parent, const Building& building_);
  void zoom_fit(int level_index);
  void set_show_tiles(const bool show_tiles_)
  {
    show_tiles = show_tiles_;
  }
  void draw_tiles();
  void update_tiles();

protected:
  void wheelEvent(QWheelEvent* event);
  void mouseMoveEvent(QMouseEvent* e);
  void mousePressEvent(QMouseEvent* e);
  void mouseReleaseEvent(QMouseEvent* e);
  void resizeEvent(QResizeEvent* e);

private:
  bool is_panning = false;
  int pan_start_x = 0;
  int pan_start_y = 0;

  const Building& building;
  bool show_tiles = false;  // ignore first few resize events during startup

  MapTileCache tile_cache;

  struct MapTilePixmapItem
  {
    int zoom = 0;
    int x = 0;
    int y = 0;
    QGraphicsPixmapItem* item = nullptr;
  };
  std::vector<MapTilePixmapItem> tile_pixmap_items;

  struct MapTileRequest
  {
    int zoom = 0;
    int x = 0;
    int y = 0;
    // todo: time it was requested?
  };
  std::vector<MapTileRequest> tile_requests;

  void request_tile(const int zoom, const int x, const int y);
  void render_tile(const int zoom, const int x, const int y, QPixmap* pixmap);
};

#endif
