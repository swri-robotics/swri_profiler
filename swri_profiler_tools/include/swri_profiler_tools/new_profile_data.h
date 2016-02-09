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

#ifndef SWRI_PROFILER_TOOLS_NEW_PROFILE_DATA_H_
#define SWRI_PROFILER_TOOLS_NEW_PROFILE_DATA_H_

#include <string>
#include <vector>

namespace swri_profiler_tools
{
// This structure represents a single chunk of profile information and
// is used to pass new data to the profile database.  I don't like the
// name but I more don't like not making progress from sitting around
// thinking of a better name.
struct NewProfileData
{
  QString label;
  uint64_t wall_stamp_sec;
  uint64_t ros_stamp_ns;
  uint64_t cumulative_call_count;
  uint64_t cumulative_inclusive_duration_ns;
  uint64_t incremental_inclusive_duration_ns;
  uint64_t incremental_max_duration_ns;
};  // struct NewProfileData

typedef std::vector<NewProfileData> NewProfileDataVector;
}  // namespace swri_profiler_tools
#endif  // SWRI_PROFILER_TOOLS_NEW_PROFILE_DATA_H_
