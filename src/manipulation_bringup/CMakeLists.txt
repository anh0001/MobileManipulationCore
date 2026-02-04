cmake_minimum_required(VERSION 3.8)
project(manipulation_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install top-level configuration and simulation assets
get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. REALPATH)
install(DIRECTORY ${REPO_ROOT}/config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY ${REPO_ROOT}/sim
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
