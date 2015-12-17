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
#include <set>
#include <map>
#include <unordered_map>

#include <QObject>
#include <QString>
#include <QStringList>
#include <swri_profiler_tools/new_profile_data.h>

namespace swri_profiler_tools
{
class ProfileDatabase;

class ProfileEntry
{

 public:
  // projected is for internal use in the profiler.  This flag
  // indicates that the item's current data was projected from a
  // previous data point.  This is used to keep the profile data
  // consistent despite missing or late data.  This flag only applies
  // to data in measured nodes.
  
  bool projected;
  uint64_t cumulative_call_count;
  uint64_t cumulative_inclusive_duration_ns;
  uint64_t incremental_inclusive_duration_ns;
  uint64_t cumulative_exclusive_duration_ns;
  uint64_t incremental_exclusive_duration_ns;
  uint64_t incremental_max_duration_ns;

  ProfileEntry()
    :
    projected(false),
    cumulative_call_count(0),
    cumulative_inclusive_duration_ns(0),
    incremental_inclusive_duration_ns(0),
    cumulative_exclusive_duration_ns(0),
    incremental_exclusive_duration_ns(0),
    incremental_max_duration_ns(0)
  {}
};  // class ProfileEntry

class ProfileNode
{
  // This is the node's key within it's profile.  It must be positive
  // to be a valid node.
  int node_key_;
  
  // The name of this node.  This is the last element of the path (c
  // in a/b/c).
  QString name_;

  // The full path of the node in the call tree.  (e.g. a/b/c)
  QString path_;

  // Nodes can be either measured or inferred.  Measured nodes have
  // their inclusive timing information provided by the profiler.
  // These are your typical blocks measured by
  // SWRI_PROFILE(). Inferred nodes are nodes that were created to
  // fill in the call tree.  These are typically your root node and
  // nodes corresponding to ROS namespaces.
  bool measured_;

  // The data stored by the node.  The array is managed by the
  // profile.  Each element corresponds to a time which is determined
  // by the Profile's min_time and max_time.
  std::deque<ProfileEntry> data_;

  // The node's depth in the tree.
  int depth_;

  // The key of this node's parent node.  This will be invalid
  // (negative) for the root node.
  int parent_;

  // They node's children, in alphabetical order according to their
  // paths.
  std::vector<int> children_;

  // The ProfileNode is a "dumb" data storage object with read-only
  // access to the rest of the world.  The node is managed and
  // manipulated directly by the profile.
  friend class Profile;

 public:
  ProfileNode() 
    :
    node_key_(-1),
    measured_(false),
    depth_(-1),
    parent_(-1)
  {}
  
  bool isValid() const { return node_key_ >= 0; }
  int nodeKey() const { return node_key_; }
  const QString& name() const { return name_; }
  const QString& path() const { return path_; }
  bool isMeasured() const { return measured_; }
  const std::deque<ProfileEntry>& data() const { return data_; }
  int depth() const { return depth_; }
  int parentKey() const { return parent_; }
  const std::vector<int>& childKeys() const { return children_; }
  bool hasChildren() const { return !children_.empty(); }
};  // class ProfileNode

class Profile : public QObject
{
  Q_OBJECT;


  // The key of this profile in the database.  This is negative for
  // an invalid profile.
  int profile_key_;

  // Name of the profile.  This is initialized by the source and may
  // be modified by the user.
  QString name_;

  // All node data is stored in dense arrays of the same size.  The
  // min_time_s_ and max_time_s_ correspond to the timespan currently
  // covered by the array.  They are inclusive and exclusive,
  // respectively (index 0 => min_time_s, index size() => max_time_s.
  uint64_t min_time_s_;
  uint64_t max_time_s_;

  // Nodes are stored in an unordered_map so that we can provide
  // persistent keys with fast look ups.  We could use the node's path
  // as the key (though we need an adapter hash a QString rather than
  // std::string), but they we're constantly hashing very long
  // strings.  Instead, we assign a unique integer key when the node
  // is added.  This map provides reasonable reverse-lookups.  We
  // could actually change nodes_ to a vector or deque and still have
  // persistent indices as long as we don't allow nodes to be deleted.
  std::map<QString, int> node_key_from_path_;
  std::unordered_map<int, ProfileNode> nodes_;

  // The flat index stores all the profile's nodes in alphabetical by
  // path order.  Traversing in order corresponds to visiting the call
  // tree in a depth-first pattern.
  std::vector<int> flat_index_;

  
  // The ProfileDatabase is the only place we want to create valid
  // profiles.  A valid profile is created by initializing a default
  // profile.  Initialization is only allowed to happen once.
  friend class ProfileDatabase;
  void initialize(int profile_key, const QString &name);

  void expandTimeline(const uint64_t sec);
  void addDataToAllNodes(const bool back, const size_t count);

  bool touchNode(const QString &path);

  void storeItemData(std::set<uint64_t> &modified_times,
                     const int node_key,
                     const NewProfileData &item);
  
  size_t indexFromSec(const uint64_t secs) const { return secs - min_time_s_; }
  uint64_t secFromIndex(const uint64_t index) const { return index + min_time_s_; }

  void rebuildIndices();
  void rebuildFlatIndex();
  void rebuildTreeIndex();
  
  void updateDerivedData(size_t index);
  void updateDerivedDataInternal(ProfileNode& node, size_t index);

 public:
  Profile();
  ~Profile();

  void addData(const NewProfileDataVector &data);
  const bool isValid() const { return profile_key_ >= 0; }
  const int profileKey() const { return profile_key_; }

  const QString& name() const { return name_; }
  void setName(const QString &name);

  const ProfileNode& node(int node_key) const;
  const ProfileNode& rootNode() const;
  const int rootKey() const { return 0; }
  const std::vector<int>& nodeKeys() const;
  
 Q_SIGNALS:
  // Emitted when the profile is renamed.
  void profileModified(int profile_key);
  void nodesAdded(int profile_key);
  void dataAdded(int profile_key);  
};  // class Profile
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_PROFILE_H_
