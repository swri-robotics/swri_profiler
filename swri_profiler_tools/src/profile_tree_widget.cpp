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

#include <QVBoxLayout>
#include <QTreeWidget>
#include <QMenu>

#include <swri_profiler_tools/profile_database.h>
#include <swri_profiler_tools/profile.h>

namespace swri_profiler_tools
{
enum ProfileTreeTypes {
  ProfileType = QTreeWidgetItem::UserType,
  NodeType
};

enum ProfileTreeRoles {
  ProfileKeyRole = Qt::UserRole,
  NodeKeyRole,
};

ProfileTreeWidget::ProfileTreeWidget(QWidget *parent)
  :
  QWidget(parent),
  db_(NULL)
{
  tree_widget_ = new QTreeWidget(this);
  tree_widget_->setFont(QFont("Ubuntu Mono", 9));
  tree_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
  tree_widget_->setExpandsOnDoubleClick(false);
  
  QObject::connect(tree_widget_, SIGNAL(customContextMenuRequested(const QPoint&)),
                   this, SLOT(handleTreeContextMenuRequest(const QPoint&)));

  QObject::connect(tree_widget_, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
                   this, SLOT(handleItemActivated(QTreeWidgetItem*, int)));
  
  auto *main_layout = new QVBoxLayout();  

  main_layout->addWidget(tree_widget_);
  main_layout->setContentsMargins(0,0,0,0);
  setLayout(main_layout);
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
  tree_widget_->clear();
  items_.clear();

  if (!db_) {
    return;
  }
  
  std::vector<int> keys = db_->profileKeys();
  for (auto key : keys) {
    addProfile(key);
  }
}

void ProfileTreeWidget::addProfile(int profile_key)
{
  const Profile &profile = db_->profile(profile_key);
  if (!profile.isValid()) {
    qWarning("Invald profile for key %d.", profile_key);
    return;
  }
  
  QTreeWidgetItem *item = new QTreeWidgetItem(ProfileType);

  item->setText(0, profile.name());    
  item->setData(0, ProfileKeyRole, profile_key);
  item->setData(0, NodeKeyRole, -1);
  tree_widget_->addTopLevelItem(item);
  items_[DatabaseKey(profile.profileKey())] = item;

  for (auto child_key : profile.rootNode().childKeys()) {
    addNode(item, profile, child_key);
  }
  
  if (active_key_.isValid()) {
    if (items_.count(active_key_)) {
      markItemActive(active_key_);
    } else {
      active_key_ = DatabaseKey();
      emit activeNodeChanged(-1,-1);
    }
  }
}

void ProfileTreeWidget::addNode(QTreeWidgetItem *parent,
                                const Profile &profile,
                                const int node_key)
{
  const ProfileNode &node = profile.node(node_key);
  if (!node.isValid()) {
    qWarning("Invalid node for key %d", node_key);
    return;
  }
  
  QTreeWidgetItem *item = new QTreeWidgetItem(NodeType);
  item->setText(0, node.name());
  item->setData(0, ProfileKeyRole, profile.profileKey());
  item->setData(0, NodeKeyRole, node.nodeKey());
  parent->addChild(item);
  items_[DatabaseKey(profile.profileKey(), node.nodeKey())] = item;

  for (auto child_key : node.childKeys()) {
    addNode(item, profile, child_key);
  }
}

void ProfileTreeWidget::handleTreeContextMenuRequest(const QPoint &pos)
{
  QTreeWidgetItem *item = tree_widget_->itemAt(pos);

  auto menu = new QMenu(this);
  auto expand_all_action = menu->addAction("Expand All");
  QObject::connect(expand_all_action, SIGNAL(triggered()),
                   tree_widget_, SLOT(expandAll()));
  
  auto collapse_all_action = menu->addAction("Collapse All");
  QObject::connect(collapse_all_action, SIGNAL(triggered()),
                   tree_widget_, SLOT(collapseAll()));

  menu->popup(tree_widget_->mapToGlobal(pos));  
  QObject::connect(menu, SIGNAL(aboutToHide()), menu, SLOT(deleteLater()));

  if (!item) {
    qWarning("No item under mouse");
  } else if (item->type() == ProfileType) {
    int profile_key = item->data(0, ProfileKeyRole).toInt();
    qWarning("profile %d", profile_key);
  } else if (item->type() == NodeType) {
    int profile_key = item->data(0, ProfileKeyRole).toInt();
    int node_key = item->data(0, NodeKeyRole).toInt();
    qWarning("node %d/%d", profile_key, node_key);    
  } else {
    qWarning("Unknown item type: %d", item->type());
  }    
}

void ProfileTreeWidget::handleItemActivated(QTreeWidgetItem *item, int column)
{
  DatabaseKey new_key(item->data(0, ProfileKeyRole).toInt(),
                      item->data(0, NodeKeyRole).toInt());

  if (active_key_ != new_key) {
    markItemInactive(active_key_);
    active_key_ = new_key;
    markItemActive(active_key_);    
    emit activeNodeChanged(active_key_.profileKey(),
                           active_key_.isProfile() ? 0 : active_key_.nodeKey());
  }
}

QString ProfileTreeWidget::nameForKey(const DatabaseKey &key) const
{
  if (key.isProfile()) {
    return db_->profile(key.profileKey()).name();
  } else if (key.isNode()) {
    return db_->profile(key.profileKey()).node(key.nodeKey()).name();
  } else {
    return "<INVALID KEY>";
  }
}

void ProfileTreeWidget::markItemActive(const DatabaseKey &key)
{
  if (items_.count(key) == 0) {
    return;
  }

  items_.at(key)->setText(0, "[" + nameForKey(key) + "]");
}

void ProfileTreeWidget::markItemInactive(const DatabaseKey &key)
{ 
  if (items_.count(key) == 0) {
    return;
  }

  items_.at(key)->setText(0, nameForKey(key));
}
}  // namespace swri_profiler_tools
