cmake_minimum_required(VERSION 3.8)
project(gazebo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(main_node src/main.cpp src/pid_control.cpp)

# ament_target_dependencies(pid_node
#   rclcpp
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   )
  ament_target_dependencies(main_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  )
  install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
