cmake_minimum_required(VERSION 3.8)
project(match_cooperative_handling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(match_cooperative_handling)

add_executable(virtual_object_state_node src/virtual_object_state_node.cpp)
target_link_libraries(virtual_object_state_node
  ${geometry_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
)

add_executable(virtual_object_demo_runner src/virtual_object_demo_runner.cpp)
target_link_libraries(virtual_object_demo_runner
  ${geometry_msgs_TARGETS}
  ${rclcpp_TARGETS}
)

add_executable(virtual_object_tcp_transform_node src/virtual_object_tcp_transform_node.cpp)
target_link_libraries(virtual_object_tcp_transform_node
  ${geometry_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
)

install(TARGETS
  virtual_object_demo_runner
  virtual_object_state_node
  virtual_object_tcp_transform_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cooperative_handling_gui.py
  scripts/ensure_ur_ready.py
  scripts/log_cooperative_tracking.py
  scripts/move_arm_to_named_pose.py
  scripts/set_relative_pose_from_current_object.py
  scripts/set_virtual_object_from_manipulators.py
  scripts/set_virtual_object_from_tcps.py
  scripts/set_virtual_object_from_tcp.py
  scripts/virtual_object_demo_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cooperative_gui test/test_cooperative_gui.py)
  ament_add_pytest_test(test_top_view test/test_top_view.py)
  ament_add_pytest_test(test_mocap_tab test/test_mocap_tab.py)
  ament_add_pytest_test(test_object_pose_math test/test_object_pose_math.py)
  ament_add_pytest_test(test_virtual_object_ui test/test_virtual_object_ui.py)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
