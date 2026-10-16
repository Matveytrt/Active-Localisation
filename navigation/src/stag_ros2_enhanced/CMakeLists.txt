cmake_minimum_required(VERSION 3.8)
project(stag_ros2_enhanced)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# STag library paths (adjust if needed)
set(STAG_INCLUDE_DIR "/usr/local/lib/stag/src" CACHE PATH "Path to STag headers")
set(STAG_LIBRARY "/usr/local/lib/stag/build/libstaglib.a" CACHE FILEPATH "Path to STag library")

# Check if STag library exists
if(NOT EXISTS ${STAG_LIBRARY})
    message(FATAL_ERROR "STag library not found at ${STAG_LIBRARY}. Please build STag first.")
endif()

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${STAG_INCLUDE_DIR}
)

# C++ executable - detector node
add_executable(stag_detector_node
  src/main.cpp
  src/detector_node.cpp
  src/detection_strategies.cpp
  src/pose_estimation.cpp
  src/marker_tracker.cpp
)

ament_target_dependencies(stag_detector_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(stag_detector_node
  ${OpenCV_LIBS}
  ${STAG_LIBRARY}
  pthread
)

# C++ executable - detection logger node
add_executable(stag_detection_logger_node
  src/detection_logger_main.cpp
  src/detection_logger_node.cpp
)

ament_target_dependencies(stag_detection_logger_node
  rclcpp
  visualization_msgs
)

# Install C++ executables
install(TARGETS
  stag_detector_node
  stag_detection_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (scripts)
install(PROGRAMS
  scripts/generate_markers.py
  scripts/generate_pdf.py
  scripts/pdf_batch_generation.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
