cmake_minimum_required(VERSION 3.5)
project(swri_profiler)

find_package(Boost REQUIRED COMPONENTS thread)

if (ENV{ROS_VERSION} STREQUAL "1")
  set(BUILD_DEPS
    diagnostic_updater
    std_msgs
    swri_profiler_msgs
    roscpp
  )

  ### Catkin ###
  find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

  include_directories(include
    ${catkin_INCLUDE_DIRS})
  catkin_package(CATKIN_DEPENDS ${BUILD_DEPS} rospy
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )

  add_library(${PROJECT_NAME}
    src/profiler.cpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Boost::thread
  )
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "-std=c++11"
  )

  add_executable(basic_profiler_example_node src/nodes/basic_profiler_example_node.cpp)
  target_link_libraries(basic_profiler_example_node ${PROJECT_NAME})
  set_target_properties(basic_profiler_example_node PROPERTIES
    COMPILE_FLAGS "-std=c++11"
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
else()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(diagnostic_updater REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(swri_profiler_msgs REQUIRED)

  add_library(${PROJECT_NAME}
    src/profiler.cpp
  )
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC "-DROS2_BUILD"
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      Boost::thread
  )
  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
      diagnostic_updater
      rclcpp
      std_msgs
      swri_profiler_msgs
  )

  add_executable(basic_profiler_example_node src/nodes/basic_profiler_example_node.cpp)
  target_link_libraries(basic_profiler_example_node
    ${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py" PATTERN "*.launch"
  )
  install(TARGETS ${PROJECT_NAME} basic_profiler_example_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
  install(PROGRAMS scripts/profiler_server scripts/record_profiler_data
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_package()
endif()
