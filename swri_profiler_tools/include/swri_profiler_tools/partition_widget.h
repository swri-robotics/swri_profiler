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
#ifndef SWRI_PROFILER_TOOLS_PARTITION_WIDGET_H_
#define SWRI_PROFILER_TOOLS_PARTITION_WIDGET_H_

#include <QWidget>
#include <QColor>
#include <QRectF>
#include <swri_profiler_tools/database_key.h>

QT_BEGIN_NAMESPACE
class QHelpEvent;
QT_END_NAMESPACE

namespace swri_profiler_tools
{
class Profile;
class ProfileDatabase;
class VariantAnimation;
class PartitionWidget : public QWidget
{
  Q_OBJECT;

 public:
  PartitionWidget(QWidget *parent=0);
  ~PartitionWidget();
  void setDatabase(ProfileDatabase *db);

 public Q_SLOTS:
  void setActiveNode(int profile_key, int node_key);

 Q_SIGNALS:
  void activeNodeChanged(int profile_key, int node_key);

  
 private:
  // This structure stores information about how profile nodes are
  // laid out.
  struct LayoutItem
  {
    int node_key;
    bool exclusive;
    QRectF rect;
  };
  typedef std::vector<LayoutItem> Layout;
  
  ProfileDatabase *db_;
  DatabaseKey active_key_;

  // Controls animation of the rect that defines the view area in the
  // data space.
  VariantAnimation *view_animator_;
  QTransform win_from_data_;
  
  Layout current_layout_;
  Layout layoutProfile(const Profile &profile);

  void renderLayout(QPainter &painter,
                    const QTransform &win_from_rect,
                    const Layout &layout,
                    const Profile &profile);

  QTransform getTransform(const QRectF &win_rect,
                          const QRectF &data_rect);

  QRectF dataRect(const Layout &layout) const;
                                             
  int itemAtPoint(const QPointF &point) const;                                             
                                             
 private Q_SLOTS:
  void updateData();
  
 protected:
  bool event(QEvent *event);
  void toolTipEvent(QHelpEvent *event);
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseDoubleClickEvent(QMouseEvent *event);
};  // class PartitionWidget  
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_PARTITION_WIDGET_H_

