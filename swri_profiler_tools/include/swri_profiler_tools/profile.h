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

#ifndef SWRI_PROFILER_TOOLS_PROFILE_H_
#define SWRI_PROFILER_TOOLS_PROFILE_H_

#include <deque>
#include <map>

#include <QObject>
#include <QString>
#include <QStringList>
#include <swri_profiler_tools/new_profile_data.h>

namespace swri_profiler_tools
{
class ProfileDatabase;

class ProfileEntry
{
  bool valid;
  bool measured;
  uint64_t cumulative_call_count;
  uint64_t cumulative_inclusive_duration_ns;
  uint64_t incremental_inclusive_duration_ns;
  uint64_t cumulative_exclusive_duration_ns;
  uint64_t incremental_exclusive_duration_ns;
  uint64_t incremental_max_duration_ns;

  friend class Profile;

 public:
  ProfileEntry()
    :
    valid(false),
    measured(false),
    cumulative_call_count(0),
    cumulative_inclusive_duration_ns(0),
    incremental_inclusive_duration_ns(0),
    cumulative_exclusive_duration_ns(0),
    incremental_exclusive_duration_ns(0),
    incremental_max_duration_ns(0)
  {}
};  // class ProfileEntry

class ProfileBlock
{
  QString name;
  QString path;
  int depth;
  std::deque<ProfileEntry> data;
  
  friend class Profile;
};  // class ProfileBlock

class ProfileTreeNode
{
  QString path;
  QString parent_path;
  QStringList child_paths;
  
  friend class Profile;
};

class Profile : public QObject
{
  Q_OBJECT;

  int db_handle_;
  QString name_;
  uint64_t min_time_s_;
  uint64_t max_time_s_;
  std::map<QString, ProfileBlock> blocks_;
  QStringList flat_index_;
  ProfileTreeNode tree_root_;

  // The ProfileDatabase is the only place we want to create valid
  // profiles.
  friend class ProfileDatabase;
  void initialize(int db_handle, const QString &name);

  void expandTimeline(const uint64_t sec);
  void addDataToAllBlocks(const bool back, const size_t count);

  bool touchBlock(const QString &path);
  void addBlock(const QString &path, const QString &name, int depth);

  size_t indexFromSec(const uint64_t secs) const { return secs - min_time_s_; }

  void rebuildIndices();
  void rebuildFlatIndex();
  void rebuildTreeIndex();
  
  void updateDerivedData(size_t index);
  
 public:
  Profile();
  ~Profile();

  void addData(const NewProfileDataVector &data);
  const bool isValid() const { return db_handle_ >= 0; }
  const int dbHandle() const { return db_handle_; }
  const QString& name() const { return name_; }

 Q_SIGNALS:
  void blocksAdded(int handle);
  void dataAdded(int handle);  
};  // class Profile
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_PROFILE_H_
