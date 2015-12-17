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

#include <swri_profiler_tools/profile_database.h>


namespace swri_profiler_tools
{
struct PartitionItem
{
  int node_key;
  bool implicit;
  uint64_t span_start;
  uint64_t span_end;
};

typedef std::vector<std::vector<PartitionItem> > PartitionResult;

QColor colorFromName(const QString &name)
{
  size_t name_hash = std::hash<std::string>{}(name.toStdString());
  
  int h = (name_hash >> 0) % 255;
  int s = (name_hash >> 8) % 200 + 55;
  int v = (name_hash >> 16) % 200 + 55;
  return QColor::fromHsv(h, s, v);
}

// void generatePartition(PartitionResult &result,
//                        const Profile &profile)
// {
//   const ProfileNode &root_node = profile.rootNode();
//   if (!root_node.isValid()) {
//     qWarning("Profile returned invalid root node.");
//     return;
//   }

//   if (root_node.data().empty()) {
//     qWarning("Profile has no data.");
//     return;
//   }
  
//   PartitionItem item;
//   item.node_key = 0;
//   item.span_start = 0;
//   item.span_end = root_node.data().back().cumulative_inclusive_duration_ns;
//   result.emplace_back();
//   result.back().push_back(item);

// }                      


PartitionWidget::PartitionWidget(QWidget *parent)
  :
  QWidget(parent),
  db_(NULL)
{
  view_ = new QGraphicsView(this);

  auto *main_layout = new QVBoxLayout();
  main_layout->addWidget(view_);
  main_layout->setContentsMargins(0,0,0,0);
  setLayout(main_layout);  
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
  
  synchronizeWidget();

  QObject::connect(db_, SIGNAL(profileModified(int)),
                   this, SLOT(handleProfileAdded(int)));
  QObject::connect(db_, SIGNAL(profileAdded(int)),
                   this, SLOT(handleProfileAdded(int)));
  QObject::connect(db_, SIGNAL(nodesAdded(int)),
                   this, SLOT(handleNodesAdded(int)));
}
  
void PartitionWidget::handleProfileAdded(int profile_key)
{
  // We can optimize these to be specific later if necessary.
  synchronizeWidget();
}

void PartitionWidget::handleNodesAdded(int profile_key)
{
  // We can optimize these to be specific later if necessary.
  synchronizeWidget();
}

void PartitionWidget::synchronizeWidget()
{
}

void PartitionWidget::paintEvent(QPaintEvent *)
{
  qWarning("repaint");
  QPainter painter(this);

  painter.setPen(Qt::NoPen);
  painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));
}

void PartitionWidget::setActiveNode(int profile_key, int node_key)
{
  active_key_ = DatabaseKey(profile_key, node_key);
  repaint();
}
}  // namespace swri_profiler_tools
