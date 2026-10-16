cmake_minimum_required(VERSION 3.14)
project(tf_to_fiducial)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rtabmap_msgs REQUIRED)

add_executable(tf_to_fiducial_node src/tf_to_fiducial_node.cpp)
target_compile_features(tf_to_fiducial_node PUBLIC cxx_std_17)
ament_target_dependencies(
  tf_to_fiducial_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  rtabmap_msgs
)

install(TARGETS tf_to_fiducial_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
