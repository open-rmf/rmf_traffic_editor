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

#include "map_view.h"
#include <QScrollBar>

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

void MapView::resizeEvent(QResizeEvent *e)
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
  QPointF ul = mapToScene(QPoint(0, 0));
  QPointF lr = mapToScene(QPoint(viewport()->width(), viewport()->height()));
  printf("  ul: (%.3f, %.3f)\n", ul.x(), ul.y());
  printf("  lr: (%.3f, %.3f)\n", lr.x(), lr.y());
}
