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

#ifndef SWRI_PROFILER_TOOLS_DATABASE_KEY_H_
#define SWRI_PROFILER_TOOLS_DATABASE_KEY_H_

namespace swri_profiler_tools
{
struct DatabaseKey
{
 private:
  int profile_key_;
  int node_key_;

 public:
  DatabaseKey() : profile_key_(-1), node_key_(-1) {}
  DatabaseKey(int profile_key, int node_key)
    : profile_key_(profile_key), node_key_(node_key) {}

  bool isValid() const { return profile_key_ >= 0 && node_key_ >= 0; }

  int profileKey() const { return profile_key_; }
  int nodeKey() const { return node_key_; }
  

  bool operator==(const DatabaseKey &other) const
  {
    return (profile_key_ == other.profile_key_ &&
            node_key_ == other.node_key_);
  }

  bool operator!=(const DatabaseKey &other) const
  {
    return !(*this == other);
  }

  bool operator<(const DatabaseKey &other) const
  {
    if (profile_key_ == other.profile_key_) {
      return node_key_ < other.node_key_;
    } else {
      return profile_key_ < other.profile_key_;
    }
  }
};
}  // namespace swri_profiler_tools

// Define a hash function on a DatabaseKey to support
// std::unordered_map and std::unordered_set.
namespace std {
template <>
struct hash<swri_profiler_tools::DatabaseKey> {
  size_t operator () (const swri_profiler_tools::DatabaseKey &key) const
  {
    // This should be a fine hash function because it's very unlikely
    // someone would be working with hundreds of profiles at once.
    return key.profileKey()*1000 + key.nodeKey();
  }
};
}  // namespace std
#endif // SWRI_PROFILER_TOOLS_DATABASE_KEY_H_

