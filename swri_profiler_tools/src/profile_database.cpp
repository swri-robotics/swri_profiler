#include <swri_profiler_tools/profile_database.h>
#include <QDebug>

namespace swri_profiler_tools
{
ProfileDatabase::ProfileDatabase()
{
}

ProfileDatabase::~ProfileDatabase()
{
}

int ProfileDatabase::createHandle(const QString &name)
{
  int handle = next_handle_++;

  profiles_[handle].name = name;
  Q_EMIT profileAdded(handle);
  return handle;
}

int ProfileDatabase::addData(int handle, const std::vector<NewProfileData> &data)
{
  if (profiles_.count(handle) == 0) {
    qWarning("Invalid profile handle: %d", handle);
    return -1;
  }

  Profile& profile = profiles_[handle];

  return handle;
}
}  // namespace swri_profiler_tools
