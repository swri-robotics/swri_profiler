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

int ProfileDatabase::createProfile(const QString &name)
{
  // Find an available key
  int key = profiles_.size();
  while (profiles_.count(key) != 0) { key++; }

  // We are creating a new key
  profiles_[key] = new Profile();
  Profile &profile = *(profiles_.at(key));
  profile.initialize(key, name);

  // We rebroadcast the individual profile signals in bulk so that
  // other objects can just connect to us and not deal with
  // adding/removing connections as profiles are added or deleted.
  QObject::connect(&profile, SIGNAL(blocksAdded(int)),
                   this, SIGNAL(blocksAdded(int)));
  QObject::connect(&profile, SIGNAL(dataAdded(int)),
                   this, SIGNAL(dataAdded(int)));
    
  Q_EMIT profileAdded(key);
  return key;
}

Profile& ProfileDatabase::profile(int key)
{
  if (profiles_.count(key) == 0) {
    qWarning("Invalid profile key: %d", key);
    return invalid_profile_;
  }

  return *(profiles_.at(key));
}

const Profile& ProfileDatabase::profile(int key) const
{
  if (profiles_.count(key) == 0) {
    qWarning("Invalid profile key: %d", key);
    return invalid_profile_;
  }

  return *(profiles_.at(key));
}

std::vector<int> ProfileDatabase::profileKeys() const
{
  std::vector<int> keys;
  keys.reserve(profiles_.size());
  
  for (auto const &it : profiles_) {
    keys.push_back(it.first);
  }
  return keys;
}
}  // namespace swri_profiler_tools
