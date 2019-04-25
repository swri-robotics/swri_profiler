cmake_minimum_required(VERSION 2.8.3)
project(swri_profiler)

set(BUILD_DEPS
  diagnostic_updater
  roscpp
  rospy
  std_msgs 
  swri_profiler_msgs
)

set(RUNTIME_DEPS
  diagnostic_updater
  roscpp
  rospy
  std_msgs 
  swri_profiler_msgs 
)


### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

include_directories(include
  ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


add_library(${PROJECT_NAME}
  src/profiler.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-std=c++0x"
)

add_executable(basic_profiler_example_node src/nodes/basic_profiler_example_node.cpp)
target_link_libraries(basic_profiler_example_node ${PROJECT_NAME})
set_target_properties(basic_profiler_example_node PROPERTIES
  COMPILE_FLAGS "-std=c++0x"
)

add_dependencies(${PROJECT_NAME} swri_profiler_msgs_generate_messages_cpp)

### Install Test Node and Headers ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  basic_profiler_example_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/record_profiler_data scripts/profiler_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY html/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/html
)

