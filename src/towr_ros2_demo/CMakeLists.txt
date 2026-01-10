cmake_minimum_required(VERSION 3.8)
project(towr_ros2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# TOWR'u bulalım
find_package(ifopt 2.0 REQUIRED)
find_package(towr 1.3 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Executable oluşturalım
add_executable(towr_node src/towr_node.cpp)

# Include directories
target_include_directories(towr_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# C++14 standardı kullanalım
target_compile_features(towr_node PUBLIC c_std_99 cxx_std_14)

# ROS2 dependencies
ament_target_dependencies(towr_node
  rclcpp
  std_msgs
)

# TOWR'u link edelim
target_link_libraries(towr_node 
  towr::towr
  ifopt::ifopt_ipopt
)

# Install
install(TARGETS towr_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()