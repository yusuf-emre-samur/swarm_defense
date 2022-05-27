cmake_minimum_required(VERSION 3.8)
project(sd_world_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# gazebo
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
# rcl
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
# interfaces
find_package(std_msgs REQUIRED)
find_package(sd_interfaces REQUIRED)


# gazebo
link_directories(${gazebo_dev_LIBRARY_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# world plugin
add_library(sd_world_plugin SHARED
  src/sd_world_plugin.cpp
)
target_include_directories(sd_world_plugin PUBLIC include)
ament_target_dependencies(
  sd_world_plugin
  gazebo_dev
  gazebo_ros
  rclcpp

  std_msgs
  sd_interfaces
)

install(
  TARGETS
  sd_world_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
