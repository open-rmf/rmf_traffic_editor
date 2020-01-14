#include "lift_preview_widget.h"
#include <QPainter>

LiftPreviewWidget::LiftPreviewWidget(const Lift& lift)
: _lift(lift)
{
}

LiftPreviewWidget::~LiftPreviewWidget()
{
}

void LiftPreviewWidget::paintEvent(QPaintEvent * /*e*/)
{
  QPainter painter(this);
  painter.setPen(QPen(QBrush(QColor(0, 0, 0, 180)), 1));
  painter.setBrush(QBrush(QColor(255, 255, 255, 120)));
  painter.drawRect(50, 50, 200, 100);
}
