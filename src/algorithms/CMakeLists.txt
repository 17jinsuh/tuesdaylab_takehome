cmake_minimum_required(VERSION 3.8)
project(algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(interfaces REQUIRED)

add_executable(motion_controller src/motion_controller.cpp)
ament_target_dependencies(motion_controller rclcpp geometry_msgs turtlesim tf2 interfaces)

add_executable(guicli_pose_issuer src/guicli_pose_issuer.cpp)
ament_target_dependencies(guicli_pose_issuer rclcpp geometry_msgs tf2)

add_executable(clock_pose_issuer src/clock_pose_issuer.cpp)
ament_target_dependencies(clock_pose_issuer rclcpp geometry_msgs tf2)

add_executable(pose_manager src/pose_manager.cpp)
ament_target_dependencies(pose_manager rclcpp geometry_msgs interfaces)

add_executable(get_target_pose_srv_test src/get_target_pose_srv_test.cpp)
ament_target_dependencies(get_target_pose_srv_test rclcpp geometry_msgs interfaces)

install( TARGETS
  motion_controller
  guicli_pose_issuer
  clock_pose_issuer
  pose_manager
  get_target_pose_srv_test
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
