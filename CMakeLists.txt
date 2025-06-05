cmake_minimum_required(VERSION 3.8)
project(moveit_trajectory_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add your executable (example: test_trajectory)
add_executable(test_trajectory src/test_trajectory.cpp)
add_executable(live_pose src/live_pose.cpp)

# Add dependencies for the test_trajectory executable
ament_target_dependencies(
  test_trajectory
  rclcpp
  moveit_ros_planning_interface
  yaml-cpp
)

# Add dependencies for the live_pose executable
ament_target_dependencies(live_pose 
  rclcpp 
  moveit_core 
  moveit_ros_planning_interface 
  moveit_ros_planning
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(test_trajectory yaml-cpp)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install the executable
install(TARGETS
  test_trajectory
  live_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
