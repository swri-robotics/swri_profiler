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

#ifndef SWRI_PROFILER_TOOLS_PROFILE_DATABASE_H_
#define SWRI_PROFILER_TOOLS_PROFILE_DATABASE_H_

#include <unordered_map>

#include <QObject>
#include <swri_profiler_tools/new_profile_data.h>
#include <swri_profiler_tools/profile.h>

namespace swri_profiler_tools
{
class ProfileDatabase : public QObject
{
  Q_OBJECT;

  Profile invalid_profile_;
  
  // We have to store these as pointers if we want to use an unordered
  // map because the map relies on operations that Qt doesn't permit
  // on QObject descendents.
  std::unordered_map<int, Profile*> profiles_;
  // This provides stable ordering for the profiles.
  std::vector<int> profiles_list_;
  
 public:
  ProfileDatabase();
  ~ProfileDatabase();

  int createProfile(const QString &name);

  std::vector<int> profileKeys() const { return profiles_list_; }

  Profile& profile(int key);
  const Profile& profile(int key) const;
  
 Q_SIGNALS:
  void profileAdded(int profile_key);
  void profileModified(int profile_key); 
  void nodesAdded(int profile_key);
  void dataAdded(int profile_key);
};  // class ProfileDatabase
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_PROFILE_DATABASE_H_
