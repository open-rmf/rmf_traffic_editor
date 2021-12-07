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

#include <cmath>
#include <QScrollBar>
#include "map_view.h"

MapView::MapView(QWidget* parent, const Building& building_)
: QGraphicsView(parent),
  building(building_)
{
  setMouseTracking(true);
  viewport()->setMouseTracking(true);
  setTransformationAnchor(QGraphicsView::NoAnchor);
}

void MapView::wheelEvent(QWheelEvent* e)
{
  // calculate the map position before we scale things
  const QPointF p_start = mapToScene(e->pos());

  // sanity check: clamp the scale if it has become super tiny
  const double scale_factor = transform().m11();

  if (e->delta() > 0)
  {
    if (scale_factor < 100)
      scale(1.1, 1.1);
  }
  else
  {
    if (scale_factor > 0.01)
      scale(0.9, 0.9);
  }

  // calculate the mouse map position now that we've scaled
  const QPointF p_end = mapToScene(e->pos());

  // translate the map back so hopefully the mouse stays in the same spot
  const QPointF diff = p_end - p_start;
  translate(diff.x(), diff.y());
  update_tiles();
}

void MapView::mousePressEvent(QMouseEvent* e)
{
  if (e->button() == Qt::MiddleButton)
  {
    is_panning = true;
    pan_start_x = e->x();
    pan_start_y = e->y();
    // setCursor(Qt::ClosedHandCursor);
    e->accept();
    return;
  }
  e->ignore();
}

void MapView::mouseReleaseEvent(QMouseEvent* e)
{
  if (e->button() == Qt::MiddleButton)
  {
    is_panning = false;
    e->accept();
    return;
  }
  e->ignore();
}

void MapView::mouseMoveEvent(QMouseEvent* e)
{
  if (is_panning)
  {
    const int dx = e->x() - pan_start_x;
    const int dy = e->y() - pan_start_y;
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - dx);
    verticalScrollBar()->setValue(verticalScrollBar()->value() - dy);
    pan_start_x = e->x();
    pan_start_y = e->y();
    e->accept();
    update_tiles();
    return;
  }
  e->ignore();
}

void MapView::resizeEvent(QResizeEvent*)
{
  update_tiles();
}

void MapView::zoom_fit(int level_index)
{
  if (building.levels.empty())
    return;
  const Level& level = building.levels[level_index];
  const int w = level.drawing_width;
  const int h = level.drawing_height;
  const double cx = w / 2;
  const double cy = h / 2;
  // todo: this doesn't seem to work. not sure how to use this function.
  ensureVisible(cx, cy, w, h);
  //resetTransform();
  //fitInView(cx, cy, w, h, Qt::KeepAspectRatio);
  //centerOn(cx, cy);
}

void MapView::draw_tiles()
{
  if (!show_tiles || !building.coordinate_system.has_tiles())
    return;
  printf("MapView::draw_tiles()\n");
}

void MapView::update_tiles()
{
  if (!show_tiles || !building.coordinate_system.has_tiles())
    return;
  printf("MapView::update_tiles()\n");
  const int viewport_width = viewport()->width();
  const int viewport_height = viewport()->height();
  QPointF ul = mapToScene(QPoint(0, 0));
  QPointF lr = mapToScene(QPoint(viewport_width, viewport_height));
  printf("  ul: (%.3f, %.3f)\n", ul.x(), ul.y());
  printf("  lr: (%.3f, %.3f)\n", lr.x(), lr.y());

  const double lon_extent = lr.x() - ul.x();
  printf("  lon extent: %.3f\n", lon_extent);
  const double tiles_visible_x = viewport_width / 256.0;
  printf("  tiles_visible_x: %.3f\n", tiles_visible_x);
  const double lon_per_tile = lon_extent / tiles_visible_x;
  printf("  lon_per_tile: %.3f\n", lon_per_tile);
  const double zoom_exact = log(360 / lon_per_tile) / log(2);
  printf("  zoom_exact: %.3f\n", zoom_exact);

  int zoom_approx = static_cast<int>(ceil(zoom_exact));
  const int MAX_ZOOM = 19;
  if (zoom_approx < 0)
    zoom_approx = 0;
  if (zoom_approx > MAX_ZOOM)
    zoom_approx = MAX_ZOOM;
  printf("  zoom_approx = %d\n", zoom_approx);

  const double ulx_clamped = std::max(std::min(ul.x(), 180.0), -180.0);
  const double lrx_clamped = std::max(std::min(lr.x(), 180.0), -180.0);
  printf("  clamped lon range: (%.3f, %.3f)\n", ulx_clamped, lrx_clamped);

  const int x_min_tile =
    floor((ulx_clamped + 180.) / 360. * (1 << zoom_approx));
  const int x_max_tile =
    floor((lrx_clamped + 180.) / 360. * (1 << zoom_approx));
  printf("  x tile range: [%d, %d]\n", x_min_tile, x_max_tile);

  const double MAX_LAT = 85.0511;
  const double uly_clamped = std::max(std::min(ul.y(), MAX_LAT), -MAX_LAT);
  const double lry_clamped = std::max(std::min(lr.y(), MAX_LAT), -MAX_LAT);
  printf("  clamped lat range: (%.3f, %.3f)\n", lry_clamped, uly_clamped);

  // some trig magic from the OpenStreetMap "Slippy map tilenames" wiki page
  const int y_min_tile = floor(
    (1.0 - asinh(tan(uly_clamped * M_PI / 180.)) / M_PI)
    / 2.0 * (1 << zoom_approx));

  const int y_max_tile = floor(
    (1.0 - asinh(tan(lry_clamped * M_PI / 180.)) / M_PI)
    / 2.0 * (1 << zoom_approx));

  printf("  y tile range: [%d, %d]\n", y_min_tile, y_max_tile);

  std::vector<size_t> remove_idx;
  for (size_t i = 0; i < tile_pixmap_items.size(); i++)
  {
    const MapTilePixmapItem& item = tile_pixmap_items[i];
    if (item.zoom != zoom_approx
      || item.x < x_min_tile
      || item.x > x_max_tile
      || item.y < y_min_tile
      || item.y > y_max_tile)
      remove_idx.push_back(i);
  }

  for (auto idx_it = remove_idx.rbegin(); idx_it != remove_idx.rend(); ++idx_it)
  {
    MapTilePixmapItem& item = tile_pixmap_items[*idx_it];
    scene()->removeItem(item.item);
    delete item.item;
    tile_pixmap_items.erase(tile_pixmap_items.begin() + *idx_it);
  }

  for (int y = y_min_tile; y <= y_max_tile; y++)
  {
    for (int x = x_min_tile; x <= x_max_tile; x++)
    {
      bool found = false;
      for (size_t i = 0; i < tile_pixmap_items.size(); i++)
      {
        const MapTilePixmapItem& item = tile_pixmap_items[i];
        if (item.x == x && item.y == y)
        {
          found = true;
          break;
        }
      }
      if (found)
        continue;

      printf("request zoom=%d, x=%d, y=%d)\n", zoom_approx, x, y);
      QPixmap* pixmap = tile_cache.get(zoom_approx, x, y);
      if (pixmap)
      {
        printf("  HOORAY found the pixmap\n");
      }
      else
      {
        request_tile(zoom_approx, x, y);
      }
    }
  }

  // todo: loop through the tile X, Y rectangle
  //   * see if we're rendering any tiles outside that rectangle or wrong zoom. remove them from the scene
  //   * see if we're already rendering the needed tile.
  //     * if not, see if it's in the cache.
  //     * if it's not in the cache, request it from the tile server
  // todo: when a request from the tile server returns, add it to the cache and render it

}

void MapView::request_tile(const int zoom, const int x, const int y)
{
  printf("  requesting tile: zoom=%d, x=%d, y=%d\n", zoom, x, y);
}
