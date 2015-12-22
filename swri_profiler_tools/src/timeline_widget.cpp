// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************
#include <swri_profiler_tools/timeline_widget.h>
#include <swri_profiler_tools/profile_database.h>

#include <QMouseEvent>
#include <QPainter>

namespace swri_profiler_tools
{
TimelineWidget::TimelineWidget(QWidget *parent)
  :
  QWidget(parent),
  max_value_(0),
  resolution_(1.0),
  current_index_(0),
  hover_active_(false),
  hover_index_(0),
  roi_min_(0),
  roi_max_(0)
{
  setMouseTracking(true);
}

TimelineWidget::~TimelineWidget()
{
}

QSize TimelineWidget::sizeHint() const
{
  return QSize(0, 45);
}

void TimelineWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  painter.setPen(Qt::NoPen);
  painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));

  qWarning("paint %zu", max_value_);
  if (max_value_ == 0) {
    return;
  }

  double tick_center_line = height()/2.0;
  double tick_half_height = 10;

  painter.setPen(Qt::black);
  int ticks = 10;
  double tick_margin = 20;
  double tick_spacing = (width() - 2*tick_margin) / (ticks-1);
  for (int tick = 0; tick < ticks; tick++) {
    painter.drawLine(tick_margin + tick*tick_spacing, tick_center_line - tick_half_height,
                     tick_margin + tick*tick_spacing, tick_center_line + tick_half_height);
  }    
}

void TimelineWidget::enterEvent(QEvent *event)
{
  qWarning("mouse enters");
}

void TimelineWidget::leaveEvent(QEvent *event)
{
  qWarning("mouse leaves");
}

void TimelineWidget::mouseMoveEvent(QMouseEvent *event)
{
  qWarning("move %d,%d", event->x(), event->y());
}

void TimelineWidget::mousePressEvent(QMouseEvent *event)
{
  qWarning("click %d,%d", event->x(), event->y());
}

void TimelineWidget::setMaximum(size_t max)
{
  if (max == max_value_) {
    return;
  }

  // todo(exjohnson): adjust current/hover/roi as appropriate.
  max_value_ = max;  
  emit maximumChanged(max_value_);
  update();
}

void TimelineWidget::setResolution(double resolution)
{
  if (qFuzzyCompare(resolution, resolution_)) {
    return;
  }

  resolution_ = resolution;
  emit resolutionChanged(resolution_);
  update();
}

void TimelineWidget::setCurrent(size_t index)
{
  index = std::min(max_value_, index);

  if (index == current_index_) {
    return;
  }

  current_index_ == index;
  emit currentChanged(index);  
  update();
}

void TimelineWidget::setHover(bool active, size_t index)
{
  index = std::min(max_value_, index);

  if (active == hover_active_ && (!hover_active_ || hover_index_ == index)) {
    return;
  }

  hover_active_ = active;
  hover_index_ = index;
  emit hoverChanged(hover_active_, hover_index_);  
  update();
}

void TimelineWidget::setRoi(size_t min, size_t max)
{
  // Guarantee that roi_max_ < max_value
  max = std::min(max_value_, max);
  // Gaurantee that roi_min_ <= roi_max_
  min = std::min(min, max);
    
  if (min == roi_min_ && max == roi_max_) {
    return;
  }

  roi_min_ = min;
  roi_max_ = max;
  emit roiChanged(roi_min_, roi_max_);    
  update();
}
}  // namespace swri_profiler_tools
