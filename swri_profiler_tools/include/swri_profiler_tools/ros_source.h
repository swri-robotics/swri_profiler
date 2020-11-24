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

#ifndef SWRI_PROFILER_TOOLS_ROS_SOURCE_H_
#define SWRI_PROFILER_TOOLS_ROS_SOURCE_H_

#include <QObject>
#include <QThread>
#ifdef ROS2_BUILD
#include <swri_profiler_msgs/msg/profile_index_array.hpp>
#include <swri_profiler_msgs/msg/profile_data_array.hpp>
#else
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileDataArray.h>
#endif

#include <swri_profiler_tools/profiler_msg_adapter.h>

namespace swri_profiler_tools
{
// RosSource implements a Qt-friendly interface around ROS.  It
// creates/monitors the connection with the ROS master and handles our
// subscriptions.  Messages are provided via the Qt signal/slot
// framework.  RosSource maintains a separate thread so that the GUI
// doesn't block when some of the ROS functions are taking their sweet
// time.
class ProfileDatabase;
class RosSourceBackend;
class RosSource : public QObject
{
  Q_OBJECT;

 public:
  RosSource(ProfileDatabase *db);
  ~RosSource();

  bool isConnected() const { return connected_; }
  const QString& masterUri() const { return master_uri_; }

  void start();
  
 Q_SIGNALS:
  void connected(bool connected, QString uri);

 private Q_SLOTS:
  void handleConnected(bool connected, QString uri);

#ifdef ROS2_BUILD
  void handleIndex(swri_profiler_msgs::msg::ProfileIndexArray msg);
  void handleData(swri_profiler_msgs::msg::ProfileDataArray msg);
#else
  void handleIndex(swri_profiler_msgs::ProfileIndexArray msg);
  void handleData(swri_profiler_msgs::ProfileDataArray msg);
#endif
  
 private:
  ProfileDatabase *db_;
  
  QThread ros_thread_;
  RosSourceBackend *backend_;

  ProfilerMsgAdapter msg_adapter_;
  int profile_key_;
  
  bool connected_;
  QString master_uri_;
};  // class RosSource
}  // namespace swri_profiler_tools
#endif // SWRI_PROFILER_TOOLS_ROS_SOURCE_H_
