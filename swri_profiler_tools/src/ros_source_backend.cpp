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
#include <QStringList>

#ifdef ROS2_BUILD
#include <rclcpp/rclcpp.hpp>
#include <functional>
#else
#include <ros/ros.h>
#endif

namespace swri_profiler_tools
{
RosSourceBackend::RosSourceBackend()
  :
  is_connected_(false)
{
  int argc = QCoreApplication::arguments().size();
  std::vector<char*> argv;
  // This is so ugly, but ros::init expects a "char**", so...
  for (const auto& arg : QCoreApplication::arguments())
  {
    auto* temp_str = new char[arg.size()];
    strcpy(temp_str, arg.toStdString().c_str());
    argv.push_back(temp_str);
  }
#ifdef ROS2_BUILD
  rclcpp::init(argc, &argv[0]);
#else
  ros::init(argc, &argv[0],
            "profiler",
            ros::init_options::AnonymousName);
#endif
  for (const auto& arg : argv)
  {
    delete[] arg;
  }

  startTimer(50);
}
  
RosSourceBackend::~RosSourceBackend()
{
#ifdef ROS2_BUILD
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
#else
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
#endif
}

void RosSourceBackend::startRos()
{
#ifdef ROS2_BUILD
  std::stringstream node_name;
  node_name << "profiler";
  char buf[200];
  std::snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)rclcpp::Clock().now().nanoseconds());
  node_name << buf;
  node_ = std::make_shared<rclcpp::Node>(node_name.str());
#else
  ros::start();
  ros::NodeHandle nh;
#endif

  is_connected_ = true;

#ifdef ROS2_BUILD
  index_sub_ = node_->create_subscription<swri_profiler_msgs::msg::ProfileIndexArray>(
    "/profiler/index",
    rclcpp::QoS(1000).transient_local(),
    [this] (const swri_profiler_msgs::msg::ProfileIndexArray::SharedPtr msg) {
      handleIndex(msg);
    });
  data_sub_ = node_->create_subscription<swri_profiler_msgs::msg::ProfileDataArray>(
    "/profiler/data",
    1000,
    [this] (const swri_profiler_msgs::msg::ProfileDataArray::SharedPtr msg) {
      handleData(msg);
    });
  std::string uri = "N/A";
#else
  index_sub_ = nh.subscribe("/profiler/index", 1000, &RosSourceBackend::handleIndex, this);
  data_sub_ = nh.subscribe("/profiler/data", 1000, &RosSourceBackend::handleData, this);
  std::string uri = ros::master::getURI();
#endif

  Q_EMIT connected(true, QString::fromStdString(uri));
}

void RosSourceBackend::stopRos()
{
#ifdef ROS2_BUILD
  rclcpp::shutdown();
#else
  ros::shutdown();
#endif
  is_connected_ = false;
  Q_EMIT connected(false, QString());
}

void RosSourceBackend::timerEvent(QTimerEvent *event)
{
#ifdef ROS2_BUILD
  bool master_status = rclcpp::ok();
#else
  bool master_status = ros::master::check();
#endif

  if (!is_connected_ && master_status) {
    startRos();
  } else if (is_connected_ && !master_status) {
    stopRos();
  } else if (is_connected_ && master_status) {
#ifdef ROS2_BUILD
    rclcpp::spin_some(node_);
#else
    ros::spinOnce();
#endif
  }    
}

#ifdef ROS2_BUILD
void RosSourceBackend::handleIndex(const swri_profiler_msgs::msg::ProfileIndexArray::SharedPtr& msg)
#else
void RosSourceBackend::handleIndex(const swri_profiler_msgs::ProfileIndexArrayPtr &msg)
#endif
{
  Q_EMIT indexReceived(*msg);
}

#ifdef ROS2_BUILD
void RosSourceBackend::handleData(const swri_profiler_msgs::msg::ProfileDataArray::SharedPtr& msg)
#else
void RosSourceBackend::handleData(const swri_profiler_msgs::ProfileIndexArrayPtr &msg)
#endif
{
  Q_EMIT dataReceived(*msg);
}

}  // namespace swri_profiler_tools
