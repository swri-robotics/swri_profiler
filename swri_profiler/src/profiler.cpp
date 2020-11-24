#ifdef ROS2_BUILD
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/this_node.h>
#include <ros/publisher.h>
#endif
#include <swri_profiler/profiler.h>

#ifdef ROS2_BUILD
#include <swri_profiler_msgs/msg/profile_data.hpp>
#include <swri_profiler_msgs/msg/profile_data_array.hpp>
#include <swri_profiler_msgs/msg/profile_index.hpp>
#include <swri_profiler_msgs/msg/profile_index_array.hpp>
#else
#include <swri_profiler_msgs/ProfileIndex.h>
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileData.h>
#include <swri_profiler_msgs/ProfileDataArray.h>
#endif

#ifdef ROS2_BUILD
namespace spm = swri_profiler_msgs::msg;
#else
namespace spm = swri_profiler_msgs;
#endif

namespace swri_profiler
{
// Define/initialize static member variables for the Profiler class.
std::unordered_map<std::string, Profiler::ClosedInfo> Profiler::closed_blocks_;
std::unordered_map<std::string, Profiler::OpenInfo> Profiler::open_blocks_;
boost::thread_specific_ptr<Profiler::TLS> Profiler::tls_;
SpinLock Profiler::lock_;

// Declare some more variables.  These are essentially more private
// static members for the Profiler, but by using static global
// variables instead we are able to keep more of the implementation
// isolated.
static bool profiler_initialized_ = false;
static boost::thread profiler_thread_;
#ifdef ROS2_BUILD
std::shared_ptr<rclcpp::Node> swri_profiler::Profiler::node_;
static std::shared_ptr<rclcpp::Publisher<spm::ProfileIndexArray> > profiler_index_pub_;
static std::shared_ptr<rclcpp::Publisher<spm::ProfileDataArray> > profiler_data_pub_;
#else
static ros::Publisher profiler_index_pub_;
static ros::Publisher profiler_data_pub_;
#endif

// collectAndPublish resets the closed_blocks_ member after each
// update to reduce the amount of copying done (which might block the
// threads doing actual work).  The incremental snapshots are
// collected here in all_closed_blocks_;
static std::unordered_map<std::string, spm::ProfileData> all_closed_blocks_;

#ifdef ROS2_BUILD
static rclcpp::Duration durationFromWall(const rclcpp::Duration &src)
{
  return rclcpp::Duration(src.seconds(), src.nanoseconds());
}

static rclcpp::Time timeFromWall(const rclcpp::Time &src)
{
  return rclcpp::Time(src.seconds(), src.nanoseconds());
}
#else
static ros::Duration durationFromWall(const ros::Duration &src)
{
  return src;
}

static ros::Duration durationFromWall(const ros::WallDuration &src)
{
  return {src.sec, src.nsec};
}

static ros::Time timeFromWall(const ros::WallTime &src)
{
  return {src.sec, src.nsec};
}
#endif

void Profiler::initializeProfiler()
{
  SpinLockGuard guard(lock_);
  if (profiler_initialized_) {
    return;
  }

#ifdef ROS2_BUILD
  RCLCPP_INFO(node_->get_logger(), "Initializing swri_profiler...");
  profiler_index_pub_ = node_->create_publisher<spm::ProfileIndexArray>("/profiler/index",
                                                                        rclcpp::QoS(1).transient_local());
  profiler_data_pub_ = node_->create_publisher<spm::ProfileDataArray>("/profiler/data", 100);
#else
  ROS_INFO("Initializing swri_profiler...");
  ros::NodeHandle nh;
  profiler_index_pub_ = nh.advertise<spm::ProfileIndexArray>("/profiler/index", 1, true);
  profiler_data_pub_ = nh.advertise<spm::ProfileDataArray>("/profiler/data", 100, false);
#endif
  profiler_thread_ = boost::thread(Profiler::profilerMain);   
  profiler_initialized_ = true;
}

void Profiler::initializeTLS()
{
  if (tls_.get()) {
#ifdef ROS2_BUILD
    RCLCPP_ERROR(node_->get_logger(),
#else
    ROS_ERROR(
#endif
      "Attempt to initialize thread local storage again.");
    return;
  }

  tls_.reset(new TLS());
  tls_->stack_depth = 0;
  tls_->stack_str = "";

  char buffer[256];
  snprintf(buffer, sizeof(buffer), "%p/", tls_.get());
  tls_->thread_prefix = std::string(buffer);

  initializeProfiler();
}

void Profiler::profilerMain()
{
#ifdef ROS2_BUILD
  RCLCPP_DEBUG(node_->get_logger(), "swri_profiler thread started.");
  rclcpp::Rate one_sec(1.0);
  while (rclcpp::ok())
  {
    one_sec.sleep();
    collectAndPublish();
  }
  RCLCPP_DEBUG(node_->get_logger(), "swri_profiler thread stopped.");
#else
  ROS_DEBUG("swri_profiler thread started.");
  while (ros::ok()) {
    // Align updates to approximately every second.
    ros::WallTime now = ros::WallTime::now();
    ros::WallTime next(now.sec+1,0);
    (next-now).sleep();
    collectAndPublish();
  }
  ROS_DEBUG("swri_profiler thread stopped.");
#endif
}

void Profiler::collectAndPublish()
{
  static bool first_run = true;
  // Grab a snapshot of the current state.
  std::unordered_map<std::string, ClosedInfo> new_closed_blocks;
  std::unordered_map<std::string, OpenInfo> threaded_open_blocks;

#ifdef ROS2_BUILD
  static rclcpp::Time last_now = rclcpp::Clock().now();
  rclcpp::Time now = rclcpp::Clock().now();
  rclcpp::Time ros_now = rclcpp::Clock(RCL_ROS_TIME).now();
#else
  static ros::WallTime last_now = ros::WallTime::now();
  ros::WallTime now = ros::WallTime::now();
  ros::Time ros_now = ros::Time::now();
#endif
  {
    SpinLockGuard guard(lock_);
    new_closed_blocks.swap(closed_blocks_);
    for (auto &pair : open_blocks_) {
      threaded_open_blocks[pair.first].t0 = pair.second.t0;
      pair.second.last_report_time = now;
    }
  }

  // Reset all relative max durations.
  for (auto &pair : all_closed_blocks_) {
#ifdef ROS2_BUILD
    pair.second.rel_total_duration = rclcpp::Duration(0);
    pair.second.rel_max_duration = rclcpp::Duration(0);
#else
    pair.second.rel_total_duration = ros::Duration(0);
    pair.second.rel_max_duration = ros::Duration(0);
#endif
  }

  // Flag to indicate if a new item was added.
  bool update_index = false;

  // Merge the new stats into the absolute stats
  for (auto const &pair : new_closed_blocks) {
    const auto &label = pair.first;
    const auto &new_info = pair.second;

    auto &all_info = all_closed_blocks_[label];

    if (all_info.key == 0) {
      update_index = true;
      all_info.key = all_closed_blocks_.size();
    }
    
    all_info.abs_call_count += new_info.count;
    all_info.abs_total_duration = durationFromWall(all_info.abs_total_duration) +
      durationFromWall(new_info.total_duration);
    all_info.rel_total_duration = durationFromWall(all_info.rel_total_duration) +
      durationFromWall(new_info.rel_duration);
    all_info.rel_max_duration = std::max(durationFromWall(all_info.rel_max_duration),
                                         durationFromWall(new_info.max_duration));
  }
  
  // Combine the open blocks from all threads into a single
  // map.
  std::unordered_map<std::string, spm::ProfileData> combined_open_blocks;
  for (auto const &pair : threaded_open_blocks) {
    const auto &threaded_label = pair.first;
    const auto &threaded_info = pair.second;

    size_t slash_index = threaded_label.find('/');
    if (slash_index == std::string::npos) {
#ifdef ROS2_BUILD
      RCLCPP_ERROR(node_->get_logger(),
#else
      ROS_ERROR(
#endif
        "Missing expected slash in label: %s", threaded_label.c_str());
      continue;
    }

#ifdef ROS2_BUILD
    rclcpp::Duration duration = durationFromWall(now - threaded_info.t0);
#else
    ros::Duration duration = durationFromWall(now - threaded_info.t0);
#endif
    
    const auto label = threaded_label.substr(slash_index+1);
    auto &new_info = combined_open_blocks[label];

    if (new_info.key == 0) {
      auto &all_info = all_closed_blocks_[label];
      if (all_info.key == 0) {
        update_index = true;
        all_info.key = all_closed_blocks_.size();
      }
      new_info.key = all_info.key;
    }

    new_info.abs_call_count++;
    new_info.abs_total_duration = durationFromWall(new_info.abs_total_duration) + duration;
    if (first_run) {
      new_info.rel_total_duration = durationFromWall(new_info.rel_total_duration) + duration;
    } else {
      new_info.rel_total_duration = durationFromWall(new_info.rel_total_duration) + std::min(
        durationFromWall(now - last_now), duration);
    }
    new_info.rel_max_duration = std::max(durationFromWall(new_info.rel_max_duration), duration);
  }

  if (update_index) {
    spm::ProfileIndexArray index;
    index.header.stamp = timeFromWall(now);
#ifdef ROS2_BUILD
    index.header.frame_id = node_->get_name();
#else
    index.header.frame_id = ros::this_node::getName();
#endif
    index.data.resize(all_closed_blocks_.size());
    
    for (auto const &pair : all_closed_blocks_)
    {
      size_t i = pair.second.key - 1;
      index.data[i].key = pair.second.key;
      index.data[i].label = pair.first;
    }
#ifdef ROS2_BUILD
    profiler_index_pub_->publish(index);
#else
    profiler_index_pub_.publish(index);
#endif
  }

  // Generate output message
  spm::ProfileDataArray msg;
  msg.header.stamp = timeFromWall(now);
#ifdef ROS2_BUILD
  msg.header.frame_id = node_->get_name();
#else
  msg.header.frame_id = ros::this_node::getName();
#endif
  msg.rostime_stamp = ros_now;
  
  msg.data.resize(all_closed_blocks_.size());
  for (auto &pair : all_closed_blocks_) {
    auto const &item = pair.second;
    size_t i = item.key - 1;

    msg.data[i].key = item.key;
    msg.data[i].abs_call_count = item.abs_call_count;
    msg.data[i].abs_total_duration = item.abs_total_duration;
    msg.data[i].rel_total_duration = item.rel_total_duration;
    msg.data[i].rel_max_duration = item.rel_max_duration;
  }

  for (auto &pair : combined_open_blocks) {
    auto const &item = pair.second;
    size_t i = item.key - 1;
    msg.data[i].abs_call_count += item.abs_call_count;
    msg.data[i].abs_total_duration = durationFromWall(msg.data[i].abs_total_duration) +
      durationFromWall(item.abs_total_duration);
    msg.data[i].rel_total_duration = durationFromWall(msg.data[i].rel_total_duration) +
      durationFromWall(item.rel_total_duration);
    msg.data[i].rel_max_duration = std::max(
      durationFromWall(msg.data[i].rel_max_duration),
      durationFromWall(item.rel_max_duration));
  }

#ifdef ROS2_BUILD
  profiler_data_pub_->publish(msg);
#else
  profiler_data_pub_.publish(msg);
#endif
  first_run = false;
  last_now = now;
}
}  // namespace swri_profiler
