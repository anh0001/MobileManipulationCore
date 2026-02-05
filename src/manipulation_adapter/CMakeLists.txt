cmake_minimum_required(VERSION 3.8)
project(manipulation_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(manipulation_msgs REQUIRED)

# Adapter node
add_executable(adapter_node src/adapter_node.cpp)
ament_target_dependencies(adapter_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  nav2_msgs
  moveit_msgs
  shape_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  manipulation_msgs
)

install(TARGETS
  adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Add unit tests here
  # ament_add_gtest(test_adapter test/test_adapter.cpp)
  # target_link_libraries(test_adapter adapter_node)
endif()

ament_package()
