cmake_minimum_required(VERSION 3.8)
project(al_nav2_plugins)

if(NOT CMAKE_BUILD_TYPE)
  # The layer evaluates ~1e6 scorer calls once per map; -O0 would be needlessly slow.
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/landmarks_io.cpp
  src/localizability_layer.cpp
  src/localization_aware_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

pluginlib_export_plugin_description_file(nav2_costmap_2d layer_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES layer_plugin.xml planner_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(AL_FIXTURE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  # Pure-C++ core (scorer, yaw DP, cost field, occlusion): header-only, no ROS.
  foreach(t test_scorer test_yaw_dp test_cost_field)
    ament_add_gtest(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include test)
    target_compile_definitions(${t} PRIVATE AL_FIXTURE_DIR="${AL_FIXTURE_DIR}")
  endforeach()

  ament_add_gtest(test_landmarks_io test/test_landmarks_io.cpp)
  target_include_directories(test_landmarks_io PRIVATE include test)
  target_compile_definitions(test_landmarks_io PRIVATE AL_FIXTURE_DIR="${AL_FIXTURE_DIR}")
  target_link_libraries(test_landmarks_io ${PROJECT_NAME})

  # ROS-level: the planner wrapper on a synthetic costmap (inner Theta* loaded via pluginlib).
  ament_add_gtest(test_planner test/test_planner.cpp)
  target_include_directories(test_planner PRIVATE include test)
  target_compile_definitions(test_planner PRIVATE AL_FIXTURE_DIR="${AL_FIXTURE_DIR}")
  target_link_libraries(test_planner ${PROJECT_NAME})
  ament_target_dependencies(test_planner ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
