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
#include <QLabel>
#include <QScrollBar>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include "map_view.h"

using std::string;

MapView::MapView(QWidget* parent, const Building& building_)
: QGraphicsView(parent),
  building(building_)
{
  network = new QNetworkAccessManager(this);
  connect(
    network,
    &QNetworkAccessManager::finished,
    this,
    &MapView::request_finished);

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

  if (!building.coordinate_system.is_global())
  {
    if (e->delta() > 0)
    {
      scale(1.25, 1.25);
    }
    else
    {
      if (scale_factor > 0.01)
        scale(0.75, 0.75);
    }
  }
  else
  {
    //const double dx = transform().dx();
    //const double dy = transform().dy();
    //const double scale_pow2 = pow(2, round(log(scale_factor)/log(2)));
    //resetTransform();
    //scale(scale_pow2, scale_pow2);
    //translate(dx, dy);

    // in the global frame when we're rendering map tiles,
    // we want to keep to an even scaling of the tile
    // so it doesn't have too many aliasing effects
    if (e->delta() > 0)
    {
      //double max_scale_factor = 100;
      // go to the nearest power of 2 above where we are
      // const double next_scale = pow(2, round(log(scale_factor*2)/log(2)));
      scale(2.0, 2.0);
    }
    else
    {
      // double min_scale_factor = 0.01;

      // go to the nearest power of 2 below where we are
      // const double next_scale = pow(2, round(log(scale_factor/2)/log(2)));

      // limit to 2^-17 = 0.0000076294
      //if (next_scale < 0.0000076294)
      //  next_scale = 0.0000076294;

      if (scale_factor > 0.0000076294)
        scale(0.5, 0.5);
    }
  }

  printf("scale factor: %.3f\n", scale_factor);
  //const double nearest_nice_scale_factor = pow(2.0, round(log(scale_factor)/log(2.0)));
  //printf("nearest nice scale factor: %.3f\n", nearest_nice_scale_factor);

  // calculate the mouse map position now that we've scaled
  const QPointF p_end = mapToScene(e->pos());

  // translate the map back so hopefully the mouse stays in the same spot
  const QPointF diff = p_end - p_start;
  translate(diff.x(), diff.y());
  draw_tiles();
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
    draw_tiles();
    return;
  }
  e->ignore();
}

void MapView::resizeEvent(QResizeEvent*)
{
  draw_tiles();
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
  const int viewport_width = viewport()->width();
  const int viewport_height = viewport()->height();
  QPointF ul = mapToScene(QPoint(0, 0));
  QPointF lr = mapToScene(QPoint(viewport_width, viewport_height));
  last_center = mapToScene(QPoint(viewport_width/2, viewport_height/2));
  // printf("viewport center: (%.3f, %.3f)\n", c.x(), c.y());
  // printf("  ul: (%.3f, %.3f)\n", ul.x(), ul.y());
  // printf("  lr: (%.3f, %.3f)\n", lr.x(), lr.y());

  const double x_extent = lr.x() - ul.x();
  // printf("  x extent: %.3f\n", x_extent);
  const double tiles_visible_x = viewport_width / 256.0;
  // printf("  tiles_visible_x: %.3f\n", tiles_visible_x);
  const double x_meters_per_tile = x_extent / tiles_visible_x;
  // printf("  x_meters_per_tile: %.3f\n", x_meters_per_tile);
  const double zoom_exact =
    log(M_PI * CoordinateSystem::WGS84_A / x_meters_per_tile) / log(2);
  // printf("  zoom_exact: %.3f\n", zoom_exact);

  int zoom_approx = static_cast<int>(ceil(zoom_exact));
  const int MAX_ZOOM = 20;
  if (zoom_approx < 0)
    zoom_approx = 0;
  if (zoom_approx > MAX_ZOOM)
    zoom_approx = MAX_ZOOM;
  // printf("  zoom_approx = %d\n", zoom_approx);

  const double eps = 0.0001;
  const double MAX_X = M_PI * CoordinateSystem::WGS84_A;
  const double ulx_clamped = std::max(std::min(ul.x(), MAX_X - eps), -MAX_X);
  const double lrx_clamped = std::max(std::min(lr.x(), MAX_X - eps), -MAX_X);
  // printf("  clamped x range: (%.3f, %.3f)\n", ulx_clamped, lrx_clamped);

  const int x_min_tile =
    floor((ulx_clamped + MAX_X) / (2. * MAX_X) * (1 << zoom_approx));
  const int x_max_tile =
    floor((lrx_clamped + MAX_X) / (2. * MAX_X) * (1 << zoom_approx));
  // printf("  x tile range: [%d, %d]\n", x_min_tile, x_max_tile);

  // because EPSG 3857 is a square, we can use MAX_X for the Y extents also
  const double uly_clamped = std::max(std::min(ul.y(), MAX_X - eps), -MAX_X);
  const double lry_clamped = std::max(std::min(lr.y(), MAX_X - eps), -MAX_X);
  // printf("  clamped y range: (%.3f, %.3f)\n", lry_clamped, uly_clamped);

  const int y_min_tile =
    (1 << zoom_approx)
    - 1
    - floor((uly_clamped + MAX_X) / (2. * MAX_X) * (1 << zoom_approx));
  const int y_max_tile =
    (1 << zoom_approx)
    - 1
    - floor((lry_clamped + MAX_X) / (2. * MAX_X) * (1 << zoom_approx));
  // printf("  y tile range: [%d, %d]\n", y_min_tile, y_max_tile);

  std::vector<size_t> remove_idx;
  for (size_t i = 0; i < tile_pixmap_items.size(); i++)
  {
    const MapTilePixmapItem& item = tile_pixmap_items[i];
    if (item.zoom != zoom_approx
      || item.x < x_min_tile
      || item.x > x_max_tile
      || item.y < y_min_tile
      || item.y > y_max_tile)
    {
      remove_idx.push_back(i);
      printf("  need to remove remove tile idx %d: zoom=%d, (%d, %d)\n",
        (int)i,
        item.zoom,
        item.x,
        item.y);
    }
  }

  for (auto idx_it = remove_idx.rbegin(); idx_it != remove_idx.rend(); ++idx_it)
  {
    MapTilePixmapItem& item = tile_pixmap_items[*idx_it];
    printf("  now removing tile idx %d: zoom=%d, (%d, %d)\n",
      (int)*idx_it,
      item.zoom,
      item.x,
      item.y);
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

      //printf("request zoom=%d, x=%d, y=%d)\n", zoom_approx, x, y);
      std::optional<const QByteArray> p = tile_cache.get(zoom_approx, x, y);
      if (p.has_value())
      {
        //printf("  HOORAY found the pixmap\n");
        QImage image;
        bool parse_ok = image.loadFromData(p.value());
        if (parse_ok)
        {
          QPixmap pixmap(QPixmap::fromImage(image.convertToFormat(
              QImage::Format_Grayscale8)));
          // QPixmap pixmap(QPixmap::fromImage(image));
          render_tile(zoom_approx, x, y, pixmap);
        }
        else
        {
          printf("oh no! couldn't parse image file in cache\n");
        }
      }
      else
      {
        request_tile(zoom_approx, x, y);
      }
    }
  }
}

void MapView::request_tile(const int zoom, const int x, const int y)
{
  static int s_num_requests = 0;

  s_num_requests++;

  if (s_num_requests <= 2000)
  {
    printf("  requesting tile %d: zoom=%d, x=%d, y=%d\n",
      s_num_requests,
      zoom,
      x,
      y);

    QString request_url;
    request_url.sprintf(
      "http://tiles.demo.open-rmf.org/tile/%d/%d/%d.png",
      zoom,
      x,
      y);
    QNetworkRequest request;
    request.setUrl(QUrl(request_url));
    request.setRawHeader("User-Agent", "TrafficEditor/1.4 (http://open-rmf.org)");
    network->get(request);
  }

  // create a dummy image for debugging
  QImage image(256, 256, QImage::Format_RGB888);
  image.fill(qRgb(255, 255, 255));

  QPainter painter;
  painter.begin(&image);
  painter.setPen(QPen(Qt::red));
  QString label;
  label.sprintf("%d (%d,%d)", zoom, x, y);
  painter.drawText(10, 10, 245, 100, Qt::AlignLeft, label);
  painter.end();
  QPixmap pixmap(QPixmap::fromImage(image));

  // tile_cache.set(zoom, x, y, pixmap);

  render_tile(zoom, x, y, pixmap);
}

void MapView::render_tile(
  const int zoom,
  const int tile_x,
  const int tile_y,
  const QPixmap& pixmap)
{
  const double MAX_X = M_PI * CoordinateSystem::WGS84_A;
  // printf("  adding tile: zoom=%d, (%d, %d)\n", zoom, x, y);
  QGraphicsPixmapItem* pixmap_item = scene()->addPixmap(pixmap);
  pixmap_item->setScale(2. * MAX_X / 256.0 / (1 << zoom));
  pixmap_item->setZValue(-10.0);
  //pixmap_item->setScale(360. / 256.0 / (1 << zoom));

  // magic math from the OpenStreetMap "Slippy map tilenames" wiki page
  // const double lon_deg = x / static_cast<double>(1 << zoom) * 360. - 180.;
  // const double n = M_PI - 2.0 * M_PI * y / static_cast<double>(1 << zoom);
  // const double lat = atan(0.5 * (exp(n) - exp(-n)));
  const double x = tile_x / static_cast<double>(1 << zoom) * 2. * MAX_X - MAX_X;
  const double y =
    -tile_y / static_cast<double>(1 << zoom) * 2. * MAX_X + MAX_X;

  // project the latitude to "web mercator" latitude
  //const double lat_deg_mercator = 180. / M_PI * log(tan(lat) + 1. / cos(lat));

  // printf("  %.5f -> %.5f\n", 180. / M_PI * lat, lat_deg_mercator);
  //pixmap_item->setPos(lon_deg, lat_deg_mercator);
  pixmap_item->setPos(x, y);

  // flip Y because we want +Y = up
  pixmap_item->setTransform(pixmap_item->transform().scale(1., -1.));

  // todo: pixmap_item->setOffset() to deal with the upside-down rendering?
  // or setTransformOriginPoint?

  MapTilePixmapItem item;
  item.zoom = zoom;
  item.x = tile_x;
  item.y = tile_y;
  item.item = pixmap_item;
  tile_pixmap_items.push_back(item);
}

void MapView::request_finished(QNetworkReply* reply)
{
  /*
  printf("mapview::request_finished()\n");
  printf("  request url: %s\n",
    reply->request().url().path().toStdString().c_str());
  */
  const string url(reply->request().url().path().toStdString());
  if (url.size() < 10)
    return;
  const size_t zoom_start = 6;
  const size_t zoom_end = url.find('/', zoom_start);
  if (zoom_end == string::npos)
    return;
  const string zoom_str = url.substr(zoom_start, zoom_end - zoom_start);

  const size_t x_start = zoom_end + 1;
  if (x_start >= url.size())
    return;
  const size_t x_end = url.find('/', x_start);
  const string x_str = url.substr(x_start, x_end - x_start);

  const size_t y_start = x_end + 1;
  if (y_start >= url.size())
    return;
  const size_t y_end = url.find('.', y_start);
  const string y_str = url.substr(y_start, y_end - y_start);

  const int zoom = std::stoi(zoom_str);
  const int x = std::stoi(x_str);
  const int y = std::stoi(y_str);

  QByteArray bytes = reply->readAll();
  printf("received %d-byte tile: zoom=%d x=%d y=%d\n",
    bytes.length(),
    zoom,
    x,
    y);

  QImage image;
  bool parse_ok = image.loadFromData(bytes);
  if (parse_ok)
  {
    // printf("  parse OK to %d x %d image\n", image.width(), image.height());
    // QPixmap pixmap(QPixmap::fromImage(image));
    QPixmap pixmap(QPixmap::fromImage(image.convertToFormat(
        QImage::Format_Grayscale8)));
    tile_cache.set(zoom, x, y, bytes);
    // find the placeholder pixmapitem and update its pixmap
    for (auto& item : tile_pixmap_items)
    {
      if (item.x == x && item.y == y && item.zoom == zoom)
      {
        item.item->setPixmap(pixmap);
        break;
      }
    }
  }
  else
  {
    printf("  unable to parse\n");
  }
}

void MapView::clear()
{
  tile_pixmap_items.clear();
}

void MapView::update_cache_size_label(QLabel* label)
{
  if (!label)
    return;

  MapTileCache::CacheSize size = tile_cache.getSize();
  label->setText(
    QString("Tile cache: %1 files, %2 MB")
    .arg(size.files)
    .arg(size.bytes / 1.0e6, 0, 'g', 3));
}
