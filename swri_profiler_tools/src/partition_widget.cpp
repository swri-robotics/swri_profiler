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
#include <swri_profiler_tools/partition_widget.h>

#include <QGraphicsView>
#include <QVBoxLayout>
#include <QDebug>

#include <swri_profiler_tools/profile_database.h>


namespace swri_profiler_tools
{
struct PartitionLayoutItem
{
  int node_key;
  bool exclusive;
  uint64_t span_start;
  uint64_t span_end;
};

struct PartitionGraphicsItem
{
  QColor color;
  QRect rect;
};

typedef std::vector<std::vector<PartitionLayoutItem> > PartitionLayout;

static QColor colorFromString(const QString &name)
{
  size_t name_hash = std::hash<std::string>{}(name.toStdString());
  
  int h = (name_hash >> 0) % 255;
  int s = (name_hash >> 8) % 200 + 55;
  int v = (name_hash >> 16) % 200 + 55;
  return QColor::fromHsv(h, s, v);
}

static void layoutPartition(PartitionLayout &layout,
                            const Profile &profile)
{
  const ProfileNode &root_node = profile.rootNode();
  if (!root_node.isValid()) {
    qWarning("Profile returned invalid root node.");
    return;
  }

  if (root_node.data().empty()) {
    qWarning("Profile has no data.");
    return;
  }
  
  PartitionLayoutItem root_item;
  root_item.node_key = root_node.nodeKey();
  root_item.exclusive = false;
  root_item.span_start = 0;
  root_item.span_end = root_node.data().back().cumulative_inclusive_duration_ns;
  layout.emplace_back();
  layout.back().push_back(root_item);

  bool keep_going = root_node.hasChildren();

  while (keep_going) {
    // We going to stop unless we see some children.
    keep_going = false;
    
    layout.emplace_back();
    const std::vector<PartitionLayoutItem> &parents = layout[layout.size()-2];
    std::vector<PartitionLayoutItem> &children = layout[layout.size()-1];

    size_t span_start = 0;
    for (auto const &parent_item : parents) {      
      const ProfileNode &parent_node = profile.node(parent_item.node_key);

      // Add the carry-over exclusive item.
      {
        PartitionLayoutItem item;
        item.node_key = parent_item.node_key;
        item.exclusive = true;
        item.span_start = span_start;
        item.span_end = span_start + parent_node.data().back().cumulative_exclusive_duration_ns;
        children.push_back(item);
        span_start = item.span_end;
      }

      // Don't add children for an exclusive item because they've already been added.
      if (parent_item.exclusive) {
        continue;
      }
      
      for (int child_key : parent_node.childKeys()) {
        const ProfileNode &child_node = profile.node(child_key);
        
        PartitionLayoutItem item;
        item.node_key = child_key;
        item.exclusive = false;
        item.span_start = span_start;
        item.span_end = span_start + child_node.data().back().cumulative_inclusive_duration_ns;
        children.push_back(item);
        span_start = item.span_end;

        keep_going |= child_node.hasChildren();
      }

      if (span_start != parent_item.span_end) {
        qWarning("Unexpected database inconsistency (1): %zu vs %zu", span_start, parent_item.span_end);
      }
    }

    if (span_start != root_item.span_end) {
      qWarning("Unexpected database inconsistency (2): %zu vs %zu", span_start, root_item.span_end);
    }
  }
}                      

void renderPartition(std::vector<PartitionGraphicsItem> &items,
                     const Profile &profile,
                     const PartitionLayout &layout,
                     int width, int height)
{
  if (layout.empty() || layout.back().empty()) {
    return;
  }
  
  double px_per_col = static_cast<double>(width-1) / layout.size();
  double px_per_span = static_cast<double>(height-1) / layout.back().back().span_end;
  
  for (size_t col = 0; col < layout.size(); col++) {
    for (size_t row = 0; row < layout[col].size(); row++) {
      const PartitionLayoutItem &layout_item = layout[col][row];

      if (layout_item.exclusive) {
        continue;
      }
      
      const ProfileNode &node = profile.node(layout_item.node_key);

      PartitionGraphicsItem gfx_item;
      gfx_item.color = colorFromString(node.name());

      QPoint top_left(std::round(px_per_col*col),
                      std::round(px_per_span*layout_item.span_start));
      QPoint bottom_right(width, height);
              
      gfx_item.rect = QRect(top_left, bottom_right);      
      items.push_back(gfx_item);
    }
  }
}

PartitionWidget::PartitionWidget(QWidget *parent)
  :
  QWidget(parent),
  db_(NULL)
{
}

PartitionWidget::~PartitionWidget()
{
}

void PartitionWidget::setDatabase(ProfileDatabase *db)
{
  if (db_) {
    // note(exjohnson): we can implement this later if desired, but
    // currently no use case for it.
    qWarning("PartitionWidget: Cannot change the profile database.");
    return;
  }

  db_ = db;

  update();

  QObject::connect(db_, SIGNAL(dataAdded(int)),
                   this, SLOT(update()));
  QObject::connect(db_, SIGNAL(profileAdded(int)),
                   this, SLOT(update()));
  QObject::connect(db_, SIGNAL(nodesAdded(int)),
                   this, SLOT(update()));
}

void PartitionWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  painter.setPen(Qt::NoPen);
  painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));
  
  if (!active_key_.isValid()) {
    return;
  }    

  const Profile &profile = db_->profile(active_key_.profileKey());
  PartitionLayout layout;
  layoutPartition(layout, profile);

  std::vector<PartitionGraphicsItem> gfx_items;
  renderPartition(gfx_items, profile, layout, width(), height());

  painter.setPen(Qt::black);  
  for (auto const &item : gfx_items) {
    painter.setBrush(item.color);
    painter.drawRect(item.rect.adjusted(0,0,-2,-2));
  }
}

void PartitionWidget::setActiveNode(int profile_key, int node_key)
{
  active_key_ = DatabaseKey(profile_key, node_key);
  update();
}
}  // namespace swri_profiler_tools
