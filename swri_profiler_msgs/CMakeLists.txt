cmake_minimum_required(VERSION 3.5)
project(swri_profiler_msgs)

if ($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    DIRECTORY ros1_msg
    FILES
    ProfileData.msg
    ProfileDataArray.msg
  )
  add_message_files(
    DIRECTORY msg
    FILES
    ProfileIndex.msg
    ProfileIndexArray.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
  )
else()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ProfileData.msg
    msg/ProfileDataArray.msg
    msg/ProfileIndex.msg
    msg/ProfileIndexArray.msg
    DEPENDENCIES builtin_interfaces std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
