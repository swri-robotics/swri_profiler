Header header
# The header contains the node's name in the frame id and the wall
# time in the stamp.

time rostime_stamp
# rostime_stamp contains the current ros::Time::now() to make it easier to
# compare data between different runs driven by the same recorded bag
# data.

ProfileData[] data
