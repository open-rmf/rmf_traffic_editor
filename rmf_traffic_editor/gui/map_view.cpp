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

#include "map_view.h"
#include <QScrollBar>

MapView::MapView(QWidget* parent)
: QGraphicsView(parent),
  is_panning(false),
  pan_start_x(0),
  pan_start_y(0)
{
  setMouseTracking(true);
  viewport()->setMouseTracking(true);
  setTransformationAnchor(QGraphicsView::NoAnchor);
}

void MapView::wheelEvent(QWheelEvent* e)
{
  // calculate the map position before we scale things
  const QPointF p_start = mapToScene(e->pos());

  // scale things
  if (e->delta() > 0)
    scale(1.1, 1.1);
  else
    scale(0.9, 0.9);

  // calculate the mouse map position now that we've scaled
  const QPointF p_end = mapToScene(e->pos());

  // translate the map back so hopefully the mouse stays in the same spot
  const QPointF diff = p_end - p_start;
  translate(diff.x(), diff.y());
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
    return;
  }
  e->ignore();
}

void MapView::zoom_fit(const Building& building, int level_index)
{
  if (building.levels.empty())
    return;
  const BuildingLevel& level = building.levels[level_index];
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
