#include <QMetaType>
#include <swri_profiler_msgs/ProfileIndexArray.h>
#include <swri_profiler_msgs/ProfileDataArray.h>

namespace swri_profiler_tools
{
void registerMetaTypes()
{
  // We need to register our ROS message types with Qt to be able to
  // pass them in Qt queued signals/slots (across threads).
  qRegisterMetaType<swri_profiler_msgs::ProfileIndexArray>("swri_profiler_msgs::ProfileIndexArray");
  qRegisterMetaType<swri_profiler_msgs::ProfileDataArray>("swri_profiler_msgs::ProfileDataArray");
}
}  // namespace swri_profiler_tools

