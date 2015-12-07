#ifndef SWRI_PROFILER_PROFILER_H_
#define SWRI_PROFILER_PROFILER_H_

#include <algorithm>
#include <limits>
#include <unordered_map>

#include <ros/time.h>
#include <ros/console.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace swri_profiler
{
class Profile
{
 public:
  Profile(const std::string &name);
  ~Profile();
  ros::WallDuration duration() const;
  double duration_s() const;
  double duration_ms() const;

 private:
  const std::string name_;
  const ros::WallTime t0_;

  // Prevent copy/assign
  Profile(const Profile&);
  Profile& operator=(const Profile&);
};
}  // namespace swri_profiler

// These three macros are used together to generate unique symbol
// names by appending the current line number to the end of the a base
// symbol.
#define SWRI_PROFILER_CONCAT_DIRECT(s1,s2) s1##s2
#define SWRI_PROFILER_CONCAT(s1, s2) SWRI_PROFILER_CONCAT_DIRECT(s1,s2)
#define SWRI_PROFILER_UNIQ_NAME(base) SWRI_PROFILER_CONCAT(base, __LINE__)

// A convenience macro that creates a Profile object with a unique name.
#define SWRI_PROFILE(name) swri_profiler::Profile SWRI_PROFILER_UNIQ_NAME(prof)(name);

// Include inline implementations
#include "profiler_impl.h"

#endif  // SWRI_PROFILER_PROFILER_H_
