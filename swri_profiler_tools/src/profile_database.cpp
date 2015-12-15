#include <swri_profiler_tools/profile_database.h>
#include <QDebug>

namespace swri_profiler_tools
{
ProfileDatabase::ProfileDatabase()
{
}

ProfileDatabase::~ProfileDatabase()
{
  for (auto &item : profiles_) {
    delete item.second;
  }
}

int ProfileDatabase::createHandle(const QString &name)
{
  if (name.isEmpty()) {
    qWarning("Refusing to create a nameless profile.");
    return -1;
  }

  // Find an available handle
  int handle = profiles_.size();
  while (profiles_.count(handle) != 0) { handle++; }

  // We are creating a new handle
  profiles_[handle] = new Profile();
  Profile &profile = *(profiles_.at(handle));
  profile.initialize(handle, name);

  // todo(elliotjo): connect profile signals here.
    
  Q_EMIT profileAdded(handle);
  return handle;
}

Profile& ProfileDatabase::getProfile(int handle)
{
  if (profiles_.count(handle) == 0) {
    qWarning("Invalid profile handle: %d", handle);
    return invalid_profile_;
  }

  return *(profiles_.at(handle));
}

const Profile& ProfileDatabase::getProfile(int handle) const
{
  if (profiles_.count(handle) == 0) {
    qWarning("Invalid profile handle: %d", handle);
    return invalid_profile_;
  }

  return *(profiles_.at(handle));
}

std::vector<int> ProfileDatabase::allHandles() const
{
  std::vector<int> handles;
  handles.reserve(profiles_.size());
  
  for (auto const &it : profiles_) {
    handles.push_back(it.first);
  }
  return handles;
}
}  // namespace swri_profiler_tools
