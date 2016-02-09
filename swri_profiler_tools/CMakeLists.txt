cmake_minimum_required(VERSION 2.8.3)
project(swri_profiler_tools)

set(BUILD_DEPS
  std_msgs 
  swri_profiler_msgs
  roscpp)

set(RUNTIME_DEPS
  std_msgs 
  swri_profiler_msgs 
  roscpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${QT_INCLUDE_DIR} )
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

catkin_package(
  CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  DEPENDS Qt4
  LIBRARIES ${PROJECT_NAME})


include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)

set(UI_FILES
  ui/profiler_window.ui
  )

set(MOC_HEADER_FILES
  include/swri_profiler_tools/profiler_window.h
  include/swri_profiler_tools/profiler_master.h
  include/swri_profiler_tools/ros_source.h
  include/swri_profiler_tools/ros_source_backend.h
  include/swri_profiler_tools/profile_database.h
  include/swri_profiler_tools/profile.h
  include/swri_profiler_tools/profile_tree_widget.h
  include/swri_profiler_tools/partition_widget.h
  include/swri_profiler_tools/variant_animation.h
  include/swri_profiler_tools/time_plot_widget.h
  )

set(SRC_FILES
  src/register_meta_types.cpp
  src/profiler_window.cpp
  src/profiler_master.cpp
  src/ros_source.cpp
  src/ros_source_backend.cpp
  src/profile_database.cpp
  src/profile.cpp
  src/profiler_msg_adapter.cpp
  src/profile_tree_widget.cpp
  src/util.cpp
  src/partition_widget.cpp
  src/time_plot_widget.cpp
  )
qt4_add_resources(RCC_SRCS resources/images.qrc)


qt4_wrap_ui(SRC_FILES ${UI_FILES})
qt4_wrap_cpp(SRC_FILES ${MOC_HEADER_FILES})

add_executable(profiler 
  src/main.cpp
  ${SRC_FILES} 
  ${RCC_SRCS})
target_link_libraries(profiler
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES})

add_dependencies(profiler swri_profiler_msgs_generate_messages_cpp)

### Install Test Node and Headers ###

install(TARGETS
  profiler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
