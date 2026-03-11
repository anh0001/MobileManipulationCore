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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(manipulation_msgs REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_visual_features visp_vs visp_me visp_detection)

set(_opencv_video_candidates
  /lib/${CMAKE_LIBRARY_ARCHITECTURE}/libopencv_video.so.4.5d
  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libopencv_video.so.4.5d
  /lib/aarch64-linux-gnu/libopencv_video.so.4.5d
  /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5d
  /lib/x86_64-linux-gnu/libopencv_video.so.4.5d
  /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5d
)

foreach(_opencv_video_candidate IN LISTS _opencv_video_candidates)
  if(EXISTS "${_opencv_video_candidate}")
    set(OPENCV_VIDEO_ROS_LIBRARY "${_opencv_video_candidate}")
    break()
  endif()
endforeach()

if(NOT OPENCV_VIDEO_ROS_LIBRARY)
  message(FATAL_ERROR "Could not find ROS-compatible OpenCV video library (4.5d)")
endif()

add_executable(visual_servo_node
  src/visual_servo_node.cpp
)

target_include_directories(visual_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${VISP_INCLUDE_DIRS}
)

target_link_libraries(visual_servo_node
  ${VISP_LIBRARIES}
  ${OPENCV_VIDEO_ROS_LIBRARY}
)

ament_target_dependencies(visual_servo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  manipulation_msgs
)

install(TARGETS visual_servo_node
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
