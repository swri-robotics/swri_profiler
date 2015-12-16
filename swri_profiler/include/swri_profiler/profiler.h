#ifndef SWRI_PROFILER_PROFILER_H_
#define SWRI_PROFILER_PROFILER_H_

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <atomic>

#include <ros/time.h>
#include <ros/console.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace swri_profiler
{
class SpinLock
{
  std::atomic_flag locked_;
 public:
  SpinLock() : locked_(ATOMIC_FLAG_INIT) {}
    
  void acquire()
  { 
    while (locked_.test_and_set(std::memory_order_acquire)) { ; }
  }

  void release()
  {
    locked_.clear(std::memory_order_release);
  }
};

class SpinLockGuard
{
  SpinLock &lock_;
  
 public:
  SpinLockGuard(SpinLock &lock) : lock_(lock) { lock_.acquire(); }
  ~SpinLockGuard() { lock_.release(); }
};

class Profiler
{
  // OpenInfo stores data for profiled blocks that are currently
  // executing.
  struct OpenInfo
  {
    ros::WallTime t0;
    ros::WallTime last_report_time;
    OpenInfo() : last_report_time(0) {}
  };

  // ClosedInfo stores data for profiled blocks that have finished
  // executing.
  struct ClosedInfo
  {
    size_t count;
    ros::WallDuration total_duration;
    ros::WallDuration rel_duration;
    ros::WallDuration max_duration;  
    ClosedInfo() : count(0) {}
  };

  // Thread local storage for the profiler.
  struct TLS
  {
    // We support multiple threads by tracking the call stack
    // independently per thread.  We also track the stack depth to
    // guard against problems from recursion.
    size_t stack_depth;
    std::string stack_str;
    std::string thread_prefix;
  };

  // open_blocks_ stores data for profiled blocks that are currently
  // executing.  It maps a thread_prefix + stack_address to an OpenInfo
  // block.  This is stored as a shared static variable instead of a
  // TLS because the publishing thread needs to access it.
  static std::unordered_map<std::string, OpenInfo> open_blocks_;

  // closed_blocks_ stored data for profiled blocks that have finished
  // executing.  It maps a stack_address to a ClosedInfo block.  This
  // map is cleared out regularly.
  static std::unordered_map<std::string, ClosedInfo> closed_blocks_;

  // tls_ stores the thread local storage so that the profiler can
  // maintain a separate stack for each thread.
  static boost::thread_specific_ptr<TLS> tls_;

  // This spinlock guards access to open_blocks_ and closed_blocks_.
  static SpinLock lock_;

  // Other static methods implemented in profiler.cpp
  static void initializeProfiler();
  static void initializeTLS();
  static void profilerMain();
  static void collectAndPublish();

  static bool open(const std::string &name, const ros::WallTime &t0)
  {
    if (!tls_.get()) { initializeTLS(); }

    if (name.empty()) {
      ROS_ERROR("Profiler error: Profiled section has empty name. "
                "Current stack is '%s'.",
                tls_->stack_str.c_str());
      return false;
    }
    
    if (tls_->stack_depth >= 100) {
      ROS_ERROR("Profiler error: reached max stack size (%zu) while "
                "opening '%s'. Current stack is '%s'.",
                tls_->stack_depth,
                name.c_str(),
                tls_->stack_str.c_str());
      return false;
    }

    tls_->stack_depth++;
    tls_->stack_str = tls_->stack_str + "/" + name;

    std::string open_index = tls_->thread_prefix + tls_->stack_str;
    {
      SpinLockGuard guard(lock_);
      OpenInfo &info = open_blocks_[open_index];
      info.t0 = t0;
      info.last_report_time = ros::WallTime(0,0);
    }

    return true;
  }
  
  static void close(const std::string &name, const ros::WallTime &tf)
  {    
    std::string open_index = tls_->thread_prefix + tls_->stack_str;
    {
      SpinLockGuard guard(lock_);

      auto const open_it = open_blocks_.find(open_index);
      if (open_it == open_blocks_.end()) {
        ROS_ERROR("Missing entry for '%s' in open_index. Profiler is probably corrupted.",
                  name.c_str());
        return;
      }
      
      ros::WallDuration abs_duration = tf - open_it->second.t0;
      ros::WallDuration rel_duration;
      if (open_it->second.last_report_time > open_it->second.t0) {
        rel_duration = tf - open_it->second.last_report_time;
      } else {
        rel_duration = tf - open_it->second.t0;
      }
      open_blocks_.erase(open_it);
      
      ClosedInfo &info = closed_blocks_[tls_->stack_str];
      info.count++;
      if (info.count == 1) {
        info.total_duration = abs_duration;
        info.max_duration = abs_duration;
        info.rel_duration = rel_duration;
      } else {
        info.total_duration += abs_duration;
        info.rel_duration += rel_duration;
        info.max_duration = std::max(info.max_duration, abs_duration);
      }
    }

    const size_t len = name.size()+1;  
    tls_->stack_str.erase(tls_->stack_str.size()-len, len);
    tls_->stack_depth--;    
  }

 private:
  std::string name_;
  
 public:
  Profiler(const std::string &name)
  {
    if (open(name, ros::WallTime::now())) {
      name_ = name;
    } else {
      name_ = "";
    }
  }
  
  ~Profiler()
  {
    if (!name_.empty()) {
      close(name_, ros::WallTime::now());
    }
  }
};  
}  // namespace swri_profiler

// Macros for string concatenation that work with built in macros.
#define SWRI_PROFILER_CONCAT_DIRECT(s1,s2) s1##s2
#define SWRI_PROFILER_CONCAT(s1, s2) SWRI_PROFILER_CONCAT_DIRECT(s1,s2)

#define SWRI_PROFILER_IMP(block_var, name)             \
  swri_profiler::Profiler block_var(name);             \

#ifndef DISABLE_SWRI_PROFILER
#define SWRI_PROFILE(name) SWRI_PROFILER_IMP(      \
    SWRI_PROFILER_CONCAT(prof_block_, __LINE__),   \
    name)
#else // ndef DISABLE_SWRI_PROFILER
#define SWRI_PROFILE(name)
#endif // def DISABLE_SWRI_PROFILER

#endif  // SWRI_PROFILER_PROFILER_H_
