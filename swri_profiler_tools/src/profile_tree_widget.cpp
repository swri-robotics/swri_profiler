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
#include <swri_profiler_tools/profile_tree_widget.h>
#include <swri_profiler_tools/profile_database.h>
#include <swri_profiler_tools/profile.h>

namespace swri_profiler_tools
{
ProfileTreeWidget::ProfileTreeWidget(QWidget *parent)
  :
  QTreeWidget(parent),
  db_(NULL)
{
}

ProfileTreeWidget::~ProfileTreeWidget()
{
}

void ProfileTreeWidget::setDatabase(ProfileDatabase *db)
{
  if (db_) {
    // note(exjohnson): we can implement this later if desired, but
    // currently no use case for it.
    qWarning("ProfileTreeWidget: Cannot change the profile database.");
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

void ProfileTreeWidget::handleProfileAdded(int profile_key)
{
  // We can optimize these to be specific later if necessary.
  synchronizeWidget();
}

void ProfileTreeWidget::handleNodesAdded(int profile_key)
{
  // We can optimize these to be specific later if necessary.
  synchronizeWidget();
}

void ProfileTreeWidget::synchronizeWidget()
{
  qWarning("synced!");
  clear();

  if (!db_) {
    return;
  }

  std::vector<int> keys = db_->profileKeys();
  for (auto key : keys) {
    const Profile &profile = db_->profile(key);
    QTreeWidgetItem *profile_item = new QTreeWidgetItem(
      QStringList(profile.name()));
    addTopLevelItem(profile_item);
    for (auto key : profile.rootNode().childrenKeys()) {
      addNode(profile_item, profile, key);
    }
  }  
}

void ProfileTreeWidget::addNode(QTreeWidgetItem *parent,
                                const Profile &profile,
                                const int node_key)
{
  const ProfileNode &node = profile.node(node_key);
  if (!node.isValid()) {
    return;
  }
  
  QTreeWidgetItem *item = new QTreeWidgetItem(
    QStringList(node.name()));
  parent->addChild(item);
  
  for (auto key : node.childrenKeys()) {
    addNode(item, profile, key);
  }    
}
}  // namespace swri_profiler_tools
