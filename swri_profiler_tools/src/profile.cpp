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
#include <swri_profiler_tools/profile.h>
#include <algorithm>
#include <set>
#include <QStringList>
#include <QDebug>

namespace swri_profiler_tools
{
Profile::Profile()
  :
  db_handle_(-1),
  min_time_s_(0),
  max_time_s_(0)
{
}

Profile::~Profile()
{
}

void Profile::initialize(int db_handle, const QString &name)
{
  if (isValid()) {
    qWarning("Re-initializing a valid profile (%d,%s) with (%d,%s). "
             "Something is probably horribly wrong.",
             db_handle_, qPrintable(name_),
             db_handle, qPrintable(name));
  }
  db_handle_ = db_handle;
  name_ = name;
}

void Profile::addData(const NewProfileDataVector &data)
{
  if (db_handle_ < 0) {
    qWarning("Attempt to add %zu elements to an invalid profile.", data.size());
    return;
  }

  if (data.size() == 0) {
    return;
  }

  std::set<uint64_t> modified_times;
  
  bool blocks_added = false;
  for (auto const &item : data) {
    expandTimeline(item.wall_stamp_sec);
    blocks_added |= touchBlock(item.label);

    size_t index = indexFromSec(item.wall_stamp_sec);
    ProfileBlock &block = blocks_[item.label];
    block.data[index].valid = true;
    block.data[index].measured = true;
    block.data[index].cumulative_call_count = item.cumulative_call_count;
    block.data[index].cumulative_inclusive_duration_ns = item.cumulative_inclusive_duration_ns;
    block.data[index].incremental_inclusive_duration_ns = item.incremental_inclusive_duration_ns;
    block.data[index].incremental_max_duration_ns = item.incremental_max_duration_ns;

    modified_times.insert(item.wall_stamp_sec);
  }  

  if (blocks_added) {
    rebuildIndices();
    Q_EMIT blocksAdded(db_handle_);
  }

  for (auto const &t : modified_times) {
    updateDerivedData(indexFromSec(t));
  }
  Q_EMIT dataAdded(db_handle_);
}

void Profile::expandTimeline(const uint64_t sec)
{
  if (sec >= min_time_s_ && sec < max_time_s_) {
    // This time is already in our timeline, so ignore it.
  } else if (min_time_s_ == max_time_s_) {
    // The timeline is empty
    min_time_s_ = sec;
    max_time_s_ = sec+1;
    addDataToAllBlocks(true, 1);
  } else if (sec >= max_time_s_) {
    // New data extends the back of the timeline.
    size_t new_elements = sec - max_time_s_ + 1;
    max_time_s_ = sec+1;
    addDataToAllBlocks(true, new_elements);
  } else {
    // New data must be at the front of the timeline.
    size_t new_elements = min_time_s_ - sec;
    min_time_s_ = sec;
    addDataToAllBlocks(false, new_elements);
  }    
}

void Profile::addDataToAllBlocks(const bool back, const size_t count)
{
  if (back) {
    for (auto &it : blocks_) {
      std::deque<ProfileEntry> &data = it.second.data;
      data.insert(data.end(), count, ProfileEntry());
    }
  } else {
    for (auto &it : blocks_) {
      std::deque<ProfileEntry> &data = it.second.data;
      data.insert(data.begin(), count, ProfileEntry());
    }
  }
}

bool Profile::touchBlock(const QString &path)
{
  if (blocks_.count(path)) {
    return false;
  }

  QStringList all_parts = path.split('/');
  if (all_parts.isEmpty()) {
    qWarning("Path block does not have a root component? '%s'", qPrintable(path));
    return false;
  }

  int depth = 0;
  QString this_path = all_parts.takeFirst();
  if (!blocks_.count(this_path)) {
    addBlock(this_path, this_path, depth);
  }

  while (!all_parts.isEmpty()) {
    QString this_name = all_parts.takeFirst();
    depth++;
    
    this_path = this_path + "/" + this_name;
    if (!blocks_.count(this_path)) {
      addBlock(this_path, this_name, depth);
    }
  }

  return true;
}

void Profile::addBlock(const QString &path, const QString &name, int depth)
{
  ProfileBlock &block = blocks_[path];
  block.name = name;
  block.path = path;
  block.depth = depth;
  block.data.resize(max_time_s_ - min_time_s_);
}

void Profile::rebuildIndices()
{
  rebuildFlatIndex();
  rebuildTreeIndex();
}

void Profile::rebuildFlatIndex()
{
  QStringList index;
  for (auto const &it : blocks_) {
    index.append(it.first);
  }
  index.sort();
  flat_index_ = index;
}

// Compares the first N items of two string lists.
static bool compareInitialStringList(
  const QStringList &list1,
  const QStringList &list2)
{
  int size = std::min(list1.size(), list2.size());
  
  if (size == 0) {
    return true;
  }

  // Comparing in reverse because, in our use case, the differences
  // are more likely to be at the end of the lists.
  for (int i = size; i > 0; i--) {
    if (list1[i-1] != list2[i-1]) {
      return false;
    }
  }
  return true;    
}

// static void printTree(ProfileTreeNode *node, const QString &prefix)
// {
//   qWarning(qPrintable(prefix + node->name));
//   for (size_t i = 0; i < node->child_nodes.size(); i++) {
//     printTree(&(node->child_nodes[i]), prefix + "  ");
//   }
// }

void Profile::rebuildTreeIndex()
{
  tree_root_.path = "";
  tree_root_.parent_node = NULL;
  tree_root_.child_nodes.clear();

  ProfileTreeNode *current = &tree_root_;
  QStringList stack;
  stack.push_back("");

  for (int i = 0; i < flat_index_.size(); i++) {
    qDebug() << i << ": " << flat_index_[i];
  }

  // Start at 1 because the first key is the root node.
  for (int i = 1; i < flat_index_.size(); i++) {    
    QStringList parts = flat_index_[i].split('/');

    while (stack.size() > 1 && !compareInitialStringList(stack, parts)) {
      stack.pop_back();
      current = current->parent_node;
    }

    while (stack.size() < parts.size()) {
      QString name = parts[stack.size()];
      stack.push_back(name);

      QString path = stack.join("/") + "/" + name;
      current->child_nodes.emplace_back();
      ProfileTreeNode *new_node = &(current->child_nodes.back());
      new_node->name = name;
      new_node->path = path;
      new_node->parent_node = current;
      current = new_node;
    }
  }
}

size_t Profile::findLastValidIndex(const ProfileBlock& block, size_t index)
{
  while (index > 0 && !block.data[index].valid) { index--; }
  return index;
}

void Profile::updateDerivedData(size_t index)
{
  updateDerivedDataInternal(&tree_root_, index);
}

void Profile::updateDerivedDataInternal(ProfileTreeNode *node, size_t index)
{
  uint64_t children_cum_call_count = 0;
  uint64_t children_cum_incl_duration = 0;
  uint64_t children_inc_incl_duration = 0;
  uint64_t children_inc_max_duration = 0;

  for (auto &child : node->child_nodes) {
    updateDerivedDataInternal(&child, index);
    ProfileBlock &block = blocks_[child.path];    
    int valid_index = findLastValidIndex(block, index);
    ProfileEntry &data = block.data[valid_index];
    children_cum_call_count += data.cumulative_call_count;
    children_cum_incl_duration += data.cumulative_inclusive_duration_ns;
    children_inc_incl_duration += data.incremental_inclusive_duration_ns;
    children_inc_max_duration = std::max(children_inc_max_duration, data.incremental_max_duration_ns);
  }

  auto& data = blocks_[node->path].data[index];
  if (!data.measured) {
    data.valid = true;
    data.cumulative_call_count = children_cum_call_count;
    data.cumulative_inclusive_duration_ns = children_cum_incl_duration;
    data.incremental_inclusive_duration_ns = children_inc_incl_duration;
    data.incremental_max_duration_ns = children_inc_max_duration;
  }

  data.cumulative_exclusive_duration_ns = data.cumulative_inclusive_duration_ns - children_cum_incl_duration;
  data.incremental_exclusive_duration_ns = data.incremental_inclusive_duration_ns - children_inc_incl_duration;
}
}  // namespace swri_profiler_tools
