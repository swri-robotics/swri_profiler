#ifndef SWRI_PROFILER_PROFILER_IMPL_H_
#define SWRI_PROFILER_PROFILER_IMPL_H_

#include <atomic>

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

class ProfilerBackend
{
 public:
  static void open(const std::string &name);
  static void close(const std::string &name, const ros::WallDuration &duration);
  static void publish();

 private:
  struct Stats
  {
    size_t count;
    ros::WallDuration total_duration;
    ros::WallDuration max_duration;
    
    Stats() : count(0) {}
  };
  static std::unordered_map<std::string, Stats> rel_stats_;  

  struct TLS
  {
    // We support multiple threads by tracking the call stack
    // independently per thread.  We also track the stack depth to
    // guard against problems from recursion.
    size_t stack_depth;
    std::string stack_str;
  };
  static boost::thread_specific_ptr<TLS> tls_;
  static void initializeTLS();
  static void initializeProfiler();

  // This spinlock guards access to the variables that are
  // shared across threads (rel_stats_ and error_count_).
  static SpinLock lock_;
  
  static bool endsWith(const std::string &test, const std::string &suffix);
  static void incErrorCount();
  static size_t error_count_;
};

inline
void ProfilerBackend::incErrorCount()
{
  SpinLockGuard guard(lock_);
  error_count_++;  
}

inline
void ProfilerBackend::open(const std::string &name)
{
  if (!tls_.get()) { initializeTLS(); }

  if (tls_->stack_depth >= 100) {
    ROS_ERROR("Profiler stack error: reached max stack size (%zu) while opening '%s'.  Current stack is '%s'.",
              tls_->stack_depth, name.c_str(), tls_->stack_str.c_str());
    incErrorCount();
    return;
  }

  tls_->stack_depth++;
  tls_->stack_str = tls_->stack_str + "/" + name;
}

inline
void ProfilerBackend::close(const std::string &name, const ros::WallDuration &duration)
{
  if (!tls_.get()) { initializeTLS(); }

  if (tls_->stack_depth == 0) {
    ROS_ERROR("Profiler stack error: under run detected closing '%s'",
              name.c_str());
    incErrorCount();
    return;
  }    
  
  if (!endsWith(tls_->stack_str, name)) {
    ROS_ERROR("Profiler stack error detected while closing '%s'. Current stack is: '%s'",
              name.c_str(), tls_->stack_str.c_str());
    incErrorCount();
    return;
  }

  {
    SpinLockGuard guard(lock_);
    Stats &stat = rel_stats_[tls_->stack_str];
    stat.count++;
    if (stat.count == 1) {
      stat.total_duration = duration;
      stat.max_duration = duration;
    } else {
      stat.total_duration += duration;
      stat.max_duration = std::max(stat.max_duration, duration);
    }
  }

  const size_t len = name.size()+1;  
  tls_->stack_str.erase(tls_->stack_str.size()-len, len);
  tls_->stack_depth--;
}

inline
bool ProfilerBackend::endsWith(const std::string &test, const std::string &suffix)
{
  if (test.size() < suffix.size()) {
    return false;
  }
  for (size_t i = 0; i < suffix.size(); i++) {
    if (test[test.size()-i-1] != suffix[suffix.size()-i-1]) {
      return false;
    }
  }
  return true;
}

inline
Profile::Profile(const std::string &name)
  :
  name_(name),
  t0_(ros::WallTime::now())    
{
  ProfilerBackend::open(name_);
}

inline
Profile::~Profile()
{
  ProfilerBackend::close(name_, duration());
}

inline
ros::WallDuration Profile::duration() const
{
  return ros::WallTime::now() - t0_;
}

inline
double Profile::duration_s() const
{
  return duration().toSec();
}

inline
double Profile::duration_ms() const
{
  return duration().toSec()*1000.0;
}
}  // namespace swri_profiler
#endif  // SWRI_PROFILER_PROFILER_IMPL_H_
