#include <QMetaType>
#ifdef ROS2_BUILD
#include <swri_profiler_msgs/msg/profile_index_array.hpp>
#include <swri_profiler_msgs/msg/profile_data_array.hpp>
#else
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileDataArray.h>
#endif

namespace swri_profiler_tools
{
void registerMetaTypes()
{
  // We need to register our ROS message types with Qt to be able to
  // pass them in Qt queued signals/slots (across threads).
#ifdef ROS2_BUILD
  qRegisterMetaType<swri_profiler_msgs::msg::ProfileIndexArray>("swri_profiler_msgs::msg::ProfileIndexArray");
  qRegisterMetaType<swri_profiler_msgs::msg::ProfileDataArray>("swri_profiler_msgs::msg::ProfileDataArray");
#else
  qRegisterMetaType<swri_profiler_msgs::ProfileIndexArray>("swri_profiler_msgs::ProfileIndexArray");
  qRegisterMetaType<swri_profiler_msgs::ProfileDataArray>("swri_profiler_msgs::ProfileDataArray");
#endif
}
}  // namespace swri_profiler_tools

