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

#ifndef SWRI_PROFILER_TOOLS_PROFILER_MSG_ADAPTER_H_
#define SWRI_PROFILER_TOOLS_PROFILER_MSG_ADAPTER_H_

#include <map>
#include <QString>
#include <swri_profiler_tools/new_profile_data.h>
#ifdef ROS2_BUILD
#include <swri_profiler_msgs/msg/profile_index_array.hpp>
#include <swri_profiler_msgs/msg/profile_data_array.hpp>
#else
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileDataArray.h>
#endif

namespace swri_profiler_tools
{
class ProfilerMsgAdapter
{
  // The index stores, for each node, a map from a profile key to it's
  // full label.
  std::map<QString, std::map<int, QString> > index_;

 public:
  ProfilerMsgAdapter();
  ~ProfilerMsgAdapter();

#ifdef ROS2_BUILD
  void processIndex(const swri_profiler_msgs::msg::ProfileIndexArray &msg);
  bool processData(NewProfileDataVector &out_data, const swri_profiler_msgs::msg::ProfileDataArray &msg);
#else
  void processIndex(const swri_profiler_msgs::ProfileIndexArray &msg);
  bool processData(NewProfileDataVector &out_data, const swri_profiler_msgs::ProfileDataArray &msg);
#endif
  void reset();
};  // class ProfilerMsgAdapter
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_PROFILER_MSG_ADAPTER_H_
