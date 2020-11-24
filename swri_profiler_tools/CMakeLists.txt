cmake_minimum_required(VERSION 3.5)
project(swri_profiler_tools)

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

### QT ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(RCC_SRCS resources/images.qrc)
qt5_wrap_ui(SRC_FILES ${UI_FILES})

if ($ENV{ROS_VERSION} EQUAL 1)
  qt5_wrap_cpp(SRC_FILES ${MOC_HEADER_FILES})

  add_executable(profiler
    src/main.cpp
    ${SRC_FILES}
    ${RCC_SRCS}
    )

  set(BUILD_DEPS
    std_msgs
    swri_profiler_msgs
    roscpp)

  set(RUNTIME_DEPS
    std_msgs
    swri_profiler_msgs
    roscpp)

  ### Catkin ###
  find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

  include_directories(include
    ${catkin_INCLUDE_DIRS}
  )

  catkin_package(
    CATKIN_DEPENDS ${RUNTIME_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )

  target_link_libraries(profiler
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
  )
  set_target_properties(profiler PROPERTIES
    COMPILE_FLAGS "-std=c++11"
  )

  ### Install Test Node and Headers ###

  install(TARGETS
    profiler
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
else()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  qt5_wrap_cpp(SRC_FILES ${MOC_HEADER_FILES} OPTIONS "-DROS2_BUILD")

  add_executable(profiler
    src/main.cpp
    ${SRC_FILES}
    ${RCC_SRCS}
    )

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(swri_profiler_msgs REQUIRED)

  target_include_directories(profiler
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_definitions(profiler
    PUBLIC "-DROS2_BUILD"
  )
  target_link_libraries(profiler
    PRIVATE
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets
  )
  ament_target_dependencies(profiler
    PUBLIC
      ament_cmake
      rclcpp
      std_msgs
      swri_profiler_msgs
  )

  install(TARGETS
    profiler
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
  ament_package()
endif()
