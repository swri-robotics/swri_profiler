#include <swri_profiler/profiler.h>
#include <ros/this_node.h>
#include <ros/publisher.h>

#include <swri_profiler_msgs/ProfileIndex.h>
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileData.h>
#include <swri_profiler_msgs/ProfileDataArray.h>

namespace spm = swri_profiler_msgs;

namespace swri_profiler
{
// Define/initialize static member variables for the ProfilerBackend class.
std::unordered_map<std::string, ProfilerBackend::Stats> ProfilerBackend::rel_stats_;
boost::thread_specific_ptr<ProfilerBackend::TLS> ProfilerBackend::tls_;
SpinLock ProfilerBackend::lock_;
size_t ProfilerBackend::error_count_ = 0;

// Declare some more variables.  These are essentially more private
// static members for the ProfilerBackend, but by using static global
// variables instead we are able to keep more of the implementation
// isolated.
static bool profiler_initialized_ = false;
static ros::Publisher profiler_index_pub_;
static ros::Publisher profiler_data_pub_;
static ros::WallTimer profiler_pub_timer_;

static std::unordered_map<std::string, spm::ProfileData> abs_stats_;

static void timerCallback(const ros::WallTimerEvent &ignored)
{
  (void)ignored;
  ProfilerBackend::publish();
}

void ProfilerBackend::initializeProfiler()
{
  SpinLockGuard guard(lock_);
  if (profiler_initialized_) {
    return;
  }
  
  ROS_INFO("Initializing global profiler...");
  ros::NodeHandle nh;
  profiler_index_pub_ = nh.advertise<spm::ProfileIndexArray>("/profiler/index", 1, true);
  profiler_data_pub_ = nh.advertise<spm::ProfileDataArray>("/profiler/data", 100, false);
  profiler_pub_timer_ = nh.createWallTimer(ros::WallDuration(1.0),
                                           &timerCallback);
  profiler_initialized_ = true;
}

void ProfilerBackend::initializeTLS()
{
  if (tls_.get()) {
    ROS_ERROR("Attempt to initialize thread local storage again.");
    return;
  }

  tls_.reset(new TLS());
  tls_->stack_depth = 0;
  tls_->stack_str = "";

  initializeProfiler();
}

static ros::Duration durationFromWall(const ros::WallDuration &src)
{
  return ros::Duration(src.sec, src.nsec);
}

void ProfilerBackend::publish()
{
  initializeProfiler();
  
  // Grab a snapshot of the current state.  
  std::unordered_map<std::string, Stats> new_stats;
  size_t error_count;
  {
    SpinLockGuard guard(lock_);
    new_stats.swap(rel_stats_);
    error_count = error_count_;
  }
  
  bool update_index = false;
  
  // Merge the relative stats into the absolute stats
  for (auto const &pair : new_stats) {
    const auto &label = pair.first;
    const auto &rel_stat = pair.second;
    
    spm::ProfileData &stat = abs_stats_[label];
    if (stat.call_count == 0) {
      update_index = true;
      stat.key = abs_stats_.size();
      stat.call_count = rel_stat.count;
      stat.total_duration = durationFromWall(rel_stat.total_duration);
      stat.abs_max_duration = durationFromWall(rel_stat.max_duration);
    } else {
      stat.call_count += rel_stat.count;
      stat.total_duration += durationFromWall(rel_stat.total_duration);
      stat.abs_max_duration = std::max(stat.abs_max_duration, durationFromWall(rel_stat.max_duration));
    }
    stat.rel_max_duration = durationFromWall(rel_stat.max_duration);
  }
  
  if (update_index) {
    spm::ProfileIndexArray index;
    index.header.stamp = ros::Time::now();
    index.header.frame_id = ros::this_node::getName();
    
    for (auto const &pair : abs_stats_) {
      index.data.emplace_back();
      index.data.back().key = pair.second.key;
      index.data.back().label = pair.first;
    }        

    profiler_index_pub_.publish(index);
  }

  spm::ProfileDataArray msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = ros::this_node::getName();
  msg.error_count = error_count;
  
  // Generate diagnostics.
  for (auto &pair : abs_stats_) {
    msg.data.push_back(pair.second);
    pair.second.rel_max_duration = ros::Duration(0);
  }

  profiler_data_pub_.publish(msg);
}
}  // namespace swri_profiler
