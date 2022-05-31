cmake_minimum_required(VERSION 3.8)
project(sd_drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sd_interfaces REQUIRED)

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
add_executable(sd_drone_controller 
  src/sd_drone_controller.cpp
)

target_include_directories(sd_drone_controller PUBLIC include)
ament_target_dependencies(
  sd_drone_controller
  rclcpp
  sd_interfaces
)

install(
  TARGETS
  sd_drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
