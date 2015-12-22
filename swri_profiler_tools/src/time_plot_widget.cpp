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
#include <swri_profiler_tools/time_plot_widget.h>

#include <QPainter>
#include <QMouseEvent>

#include <swri_profiler_tools/profile_database.h>

namespace swri_profiler_tools
{
TimePlotWidget::TimePlotWidget(QWidget *parent)
  :
  QWidget(parent),
  db_(NULL)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

TimePlotWidget::~TimePlotWidget()
{
}

QSize TimePlotWidget::sizeHint() const
{
  return QSize(200, 200);
}

void TimePlotWidget::setDatabase(ProfileDatabase *db)
{
  if (db_) {
    // note(exjohnson): we can implement this later if desired, but
    // currently no use case for it.
    qWarning("TimePlotWidget: Cannot change the profile database.");
    return;
  }

  db_ = db;
}

void TimePlotWidget::setActiveNode(int profile_key, int node_key)
{
}

void TimePlotWidget::enterEvent(QEvent *event)
{
}

void TimePlotWidget::leaveEvent(QEvent *event)
{
}

void TimePlotWidget::mouseMoveEvent(QMouseEvent *event)
{
}

void TimePlotWidget::mousePressEvent(QMouseEvent *event)
{
}

void TimePlotWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
}

void TimePlotWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  painter.setPen(Qt::NoPen);
  painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));
  painter.setPen(Qt::black);
      
}    
}  // namespace swri_profiler_tools
