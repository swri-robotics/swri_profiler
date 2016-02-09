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

#include <swri_profiler_tools/ros_source_backend.h>
#include <QCoreApplication>
#include <ros/ros.h>

namespace swri_profiler_tools
{
RosSourceBackend::RosSourceBackend()
  :
  is_connected_(false)
{
  // We have to store this as a local variable because ros::init()
  // takes a non-const ref object.
  int argc = QCoreApplication::argc();
  ros::init(argc, QCoreApplication::argv(),
            "profiler",
            ros::init_options::AnonymousName);

  startTimer(50);
}
  
RosSourceBackend::~RosSourceBackend()
{
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void RosSourceBackend::startRos()
{
  ros::start();
  is_connected_ = true;

  ros::NodeHandle nh;
  index_sub_ = nh.subscribe("/profiler/index", 1000, &RosSourceBackend::handleIndex, this);
  data_sub_ = nh.subscribe("/profiler/data", 1000, &RosSourceBackend::handleData, this);

  std::string uri = ros::master::getURI();
  Q_EMIT connected(true, QString::fromStdString(uri));
}

void RosSourceBackend::stopRos()
{
  ros::shutdown();
  is_connected_ = false;
  Q_EMIT connected(false, QString());
}

void RosSourceBackend::timerEvent(QTimerEvent *event)
{
  bool master_status = ros::master::check();

  if (!is_connected_ && master_status) {
    startRos();
  } else if (is_connected_ && !master_status) {
    stopRos();
  } else if (is_connected_ && master_status) {
    ros::spinOnce();
  }    
}

void RosSourceBackend::handleIndex(const swri_profiler_msgs::ProfileIndexArray &msg)
{
  Q_EMIT indexReceived(msg);
}

void RosSourceBackend::handleData(const swri_profiler_msgs::ProfileDataArray &msg)
{
  Q_EMIT dataReceived(msg);
}

}  // namespace swri_profiler_tools
