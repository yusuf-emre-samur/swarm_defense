cmake_minimum_required(VERSION 3.8)
project(sd_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sd_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sd_drone_controller REQUIRED)


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


# sd drone controller
add_executable(sd_communication 
  src/sd_communication.cpp
)

target_include_directories(sd_communication PUBLIC include)
include_directories(${sd_drone_controller_INCLUDE_DIRS})

ament_target_dependencies(
  sd_communication
  rclcpp
  sd_interfaces
  Eigen3
  sd_drone_controller
)

install(
  TARGETS
  sd_communication
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()