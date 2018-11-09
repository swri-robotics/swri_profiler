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

### QT ###
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  find_package(Qt4 COMPONENTS REQUIRED QtCore QtGui)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${QT_INCLUDE_DIR};${QT_QTCORE_INCLUDE_DIR};${QT_QTGUI_INCLUDE_DIR}")
  set(Qt_LIBRARIES "${QT_QTCORE_LIBRARY};${QT_QTGUI_LIBRARY}")
  set(Qt_LIBS
      Qt4::QtCore
      Qt4::QtGui
      )
else()
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  # Not necessary for this project, but these Qt_* variables must be set in order
  # for catkin_package to DEPENDS on Qt
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
  set(Qt_LIBRARIES "${Qt5Core_LIBRARIES};${Qt5Gui_LIBRARIES};${Qt5Widgets_LIBRARIES}")
endif()

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)

catkin_package(
  CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  DEPENDS Qt
  LIBRARIES ${PROJECT_NAME})

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

if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  qt4_add_resources(RCC_SRCS resources/images.qrc)
  qt4_wrap_ui(SRC_FILES ${UI_FILES})
  qt4_wrap_cpp(SRC_FILES ${MOC_HEADER_FILES})
else()
  qt5_add_resources(RCC_SRCS resources/images.qrc)
  qt5_wrap_ui(SRC_FILES ${UI_FILES})
  qt5_wrap_cpp(SRC_FILES ${MOC_HEADER_FILES})
endif()

add_executable(profiler 
  src/main.cpp
  ${SRC_FILES} 
  ${RCC_SRCS})
target_link_libraries(profiler
  ${catkin_LIBRARIES}
  ${Qt_LIBRARIES}
  )

add_dependencies(profiler swri_profiler_msgs_generate_messages_cpp)

### Install Test Node and Headers ###

install(TARGETS
  profiler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
