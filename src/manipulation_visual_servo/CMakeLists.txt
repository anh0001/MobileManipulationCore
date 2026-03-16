cmake_minimum_required(VERSION 3.8)
project(manipulation_visual_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(manipulation_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video)

add_library(visual_servo_utils
  src/visual_servo_utils.cpp
)

target_include_directories(visual_servo_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(visual_servo_utils
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(visual_servo_utils
  sensor_msgs
)

add_executable(visual_servo_node
  src/visual_servo_node.cpp
)

target_include_directories(visual_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(visual_servo_node
  visual_servo_utils
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(visual_servo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  manipulation_msgs
)

install(TARGETS visual_servo_node
  visual_servo_utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_visual_servo_utils
    test/test_visual_servo_utils.cpp
  )
  if(TARGET test_visual_servo_utils)
    target_link_libraries(test_visual_servo_utils
      visual_servo_utils
      ${OpenCV_LIBRARIES}
    )
    target_include_directories(test_visual_servo_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OpenCV_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_visual_servo_utils
      sensor_msgs
    )
  endif()
endif()

ament_package()
