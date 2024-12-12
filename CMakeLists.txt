cmake_minimum_required(VERSION 3.8)
project(walker)

# Enable compile-time warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile_commands.json for linting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include the header files located in the 'include' directory
include_directories(
  include
)

# Add executable and specify its source file
add_executable(walker_node src/walker.cpp)

# Link dependencies to walker_node
ament_target_dependencies(walker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Install the executable, headers, and launch files
install(DIRECTORY
  launch
  include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS walker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package setup
ament_package()
