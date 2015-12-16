#include <ros/this_node.h>
#include <swri_profiler/profiler.h>
#include <ros/publisher.h>

#include <swri_profiler_msgs/ProfileIndex.h>
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileData.h>
#include <swri_profiler_msgs/ProfileDataArray.h>

namespace spm = swri_profiler_msgs;

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
static ros::Publisher profiler_index_pub_;
static ros::Publisher profiler_data_pub_;
static boost::thread profiler_thread_;

// collectAndPublish resets the closed_blocks_ member after each
// update to reduce the amount of copying done (which might block the
// threads doing actual work).  The incremental snapshots are
// collected here in all_closed_blocks_;
static std::unordered_map<std::string, spm::ProfileData> all_closed_blocks_;

static ros::Duration durationFromWall(const ros::WallDuration &src)
{
  return ros::Duration(src.sec, src.nsec);
}

static ros::Time timeFromWall(const ros::WallTime &src)
{
  return ros::Time(src.sec, src.nsec);
}

void Profiler::initializeProfiler()
{
  SpinLockGuard guard(lock_);
  if (profiler_initialized_) {
    return;
  }
  
  ROS_INFO("Initializing swri_profiler...");
  ros::NodeHandle nh;
  profiler_index_pub_ = nh.advertise<spm::ProfileIndexArray>("/profiler/index", 1, true);
  profiler_data_pub_ = nh.advertise<spm::ProfileDataArray>("/profiler/data", 100, false);
  profiler_thread_ = boost::thread(Profiler::profilerMain);   
  profiler_initialized_ = true;
}

void Profiler::initializeTLS()
{
  if (tls_.get()) {
    ROS_ERROR("Attempt to initialize thread local storage again.");
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
  ROS_DEBUG("swri_profiler thread started.");
  while (ros::ok()) {
    // Align updates to approximately every second.
    ros::WallTime now = ros::WallTime::now();
    ros::WallTime next(now.sec+1,0);
    (next-now).sleep();
    collectAndPublish();
  }
  
  ROS_DEBUG("swri_profiler thread stopped.");
}

void Profiler::collectAndPublish()
{
  static bool first_run = true;
  static ros::WallTime last_now = ros::WallTime::now();
  
  // Grab a snapshot of the current state.  
  std::unordered_map<std::string, ClosedInfo> new_closed_blocks;
  std::unordered_map<std::string, OpenInfo> threaded_open_blocks;
  ros::WallTime now = ros::WallTime::now();
  ros::Time ros_now = ros::Time::now();  
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
    pair.second.rel_total_duration = ros::Duration(0);
    pair.second.rel_max_duration = ros::Duration(0);
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
    all_info.abs_total_duration += durationFromWall(new_info.total_duration);
    all_info.rel_total_duration += durationFromWall(new_info.rel_duration);
    all_info.rel_max_duration = std::max(all_info.rel_max_duration,
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
      ROS_ERROR("Missing expected slash in label: %s", threaded_label.c_str());
      continue;
    }

    ros::Duration duration = durationFromWall(now - threaded_info.t0);
    
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
    new_info.abs_total_duration += duration;
    if (first_run) {
      new_info.rel_total_duration += duration;
    } else {
      new_info.rel_total_duration += std::min(
        durationFromWall(now - last_now), duration);
    }
    new_info.rel_max_duration = std::max(new_info.rel_max_duration, duration);
  }

  if (update_index) {
    spm::ProfileIndexArray index;
    index.header.stamp = timeFromWall(now);
    index.header.frame_id = ros::this_node::getName();
    index.data.resize(all_closed_blocks_.size());
    
    for (auto const &pair : all_closed_blocks_) {
      size_t i = pair.second.key - 1;
      index.data[i].key = pair.second.key;
      index.data[i].label = pair.first;
    }        
    profiler_index_pub_.publish(index);
  }

  // Generate output message
  spm::ProfileDataArray msg;
  msg.header.stamp = timeFromWall(now);
  msg.header.frame_id = ros::this_node::getName();
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
    msg.data[i].abs_total_duration += item.abs_total_duration;
    msg.data[i].rel_total_duration += item.rel_total_duration;
    msg.data[i].rel_max_duration = std::max(
      msg.data[i].rel_max_duration,
      item.rel_max_duration);
  }
  
  profiler_data_pub_.publish(msg);
  first_run = false;
  last_now = now;
}
}  // namespace swri_profiler
