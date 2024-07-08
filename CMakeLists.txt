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

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/gazebo_control
)
install(
  DIRECTORY description/launch
  DESTINATION share/gazebo_control
)
# install(TARGETS gazebo_control
#   EXPORT export_gazebo_control
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

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


ament_package()
