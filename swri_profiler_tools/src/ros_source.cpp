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
#include <swri_profiler_tools/ros_source.h>
#include <swri_profiler_tools/ros_source_backend.h>
#include <swri_profiler_tools/profile_database.h>

namespace swri_profiler_tools
{
static const QString LIVE_PROFILE_NAME = "ROS Capture [current]";
static const QString DEAD_PROFILE_NAME = "ROS Capture";

RosSource::RosSource(ProfileDatabase *db)
  :
  db_(db),
  backend_(NULL),
  profile_key_(-1),
  connected_(false)
{
}

RosSource::~RosSource()
{
  ros_thread_.quit();
  if (!ros_thread_.wait(500)) {
    qWarning("ROS thread is not closing in timely fashion.  This can happen "
             "when the network connection is lost or ROS master has shutdown. "
             "We will attempt to forcibly terminate the thread.");
    ros_thread_.terminate();
  }
}

void RosSource::start()
{
  if (backend_) {
    return;
  }
  
  // We run the backend in another thread.  Aside from starting up the
  // backend, this object's main function is to keep track of state
  // and provide access to that information without having to deal
  // with thread-safe data access.
  backend_ = new RosSourceBackend();
  backend_->moveToThread(&ros_thread_);

  QObject::connect(&ros_thread_, SIGNAL(finished()),
                   backend_, SLOT(deleteLater()));
  
  QObject::connect(backend_, SIGNAL(connected(bool, QString)),
                   this, SLOT(handleConnected(bool, QString)));
  QObject::connect(backend_, SIGNAL(indexReceived(swri_profiler_msgs::ProfileIndexArray)),
                   this, SLOT(handleIndex(swri_profiler_msgs::ProfileIndexArray)));
  QObject::connect(backend_, SIGNAL(dataReceived(swri_profiler_msgs::ProfileDataArray)),
                   this, SLOT(handleData(swri_profiler_msgs::ProfileDataArray)));

  ros_thread_.start();
}

void RosSource::handleConnected(bool is_connected, QString uri)
{
  connected_ = is_connected;
  master_uri_ = uri;
  Q_EMIT connected(connected_, master_uri_);

  if (!connected_) {
    msg_adapter_.reset();

    if (profile_key_ >= 0) {
      Profile &profile = db_->profile(profile_key_);
      if (profile.isValid() && profile.name() == LIVE_PROFILE_NAME) {
        profile.setName(DEAD_PROFILE_NAME);
      }
    }      
    profile_key_ = -1;
  }
}

void RosSource::handleIndex(swri_profiler_msgs::ProfileIndexArray msg)
{
  msg_adapter_.processIndex(msg);
}

void RosSource::handleData(swri_profiler_msgs::ProfileDataArray msg)
{
  NewProfileDataVector new_data;
  if (!msg_adapter_.processData(new_data, msg)) {
    return;
  }

  // todo(elliotjo): If we detect a large gap, we should create a new
  // profile for the data to handle the use case of leaving the
  // profiler open throughout a development session.  We should also
  // consider the case where data is coming from a computer with a bad
  // clock.  We'll either allocate a huge timespan or constantly
  // generate new profiles.  Either are really bad.
  
  if (profile_key_ < 0) {
    profile_key_ = db_->createProfile(LIVE_PROFILE_NAME);
    if (profile_key_ < 0) {
      qWarning("Failed to create a new profile. Dropping data.");
      return;
    }
  }
  
  db_->profile(profile_key_).addData(new_data);
}
}  // namespace swri_profiler_tools
