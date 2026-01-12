cmake_minimum_required(VERSION 3.8)
project(pavlov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 paketlerini bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(trajectory_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include dizinleri
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Controller node
add_executable(pavlov_controller_node
  src/pavlov_controller_node.cpp
)

add_executable(test_stance_node
  src/test_stance.cpp
)

add_executable(simple_step_node
  src/simple_step.cpp
)

ament_target_dependencies(simple_step_node
  rclcpp
  std_msgs
  trajectory_msgs
  gazebo_msgs
  sensor_msgs
)

ament_target_dependencies(pavlov_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(test_stance_node
  rclcpp
  std_msgs
)

target_link_libraries(pavlov_controller_node
  Eigen3::Eigen
)

target_link_libraries(test_stance_node
  Eigen3::Eigen
)

# Install
install(TARGETS
  pavlov_controller_node
  test_stance_node
  simple_step_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/gait_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()