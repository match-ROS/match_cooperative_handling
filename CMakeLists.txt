cmake_minimum_required(VERSION 3.8)
project(match_cooperative_handling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(virtual_object_state_node src/virtual_object_state_node.cpp)
ament_target_dependencies(virtual_object_state_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(virtual_object_tcp_transform_node src/virtual_object_tcp_transform_node.cpp)
ament_target_dependencies(virtual_object_tcp_transform_node
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  virtual_object_state_node
  virtual_object_tcp_transform_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cooperative_handling_gui.py
  scripts/set_virtual_object_from_tcp.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
