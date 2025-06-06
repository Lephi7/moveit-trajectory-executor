cmake_minimum_required(VERSION 3.8)
project(moveit_trajectory_executor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_trajectory_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find yaml-cpp using pkg-config
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Create executables
add_executable(test_trajectory src/test_trajectory.cpp)
add_executable(live_pose src/live_pose.cpp)
add_executable(trajectory_action_server src/trajectory_action_server.cpp)
add_executable(trajectory_action_client src/trajectory_action_client.cpp)
add_executable(pose_recorder src/pose_recorder.cpp)

# Link dependencies for test_trajectory
ament_target_dependencies(test_trajectory
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  tf2
  tf2_ros
  ament_index_cpp
)

# Link yaml-cpp manually for test_trajectory
target_link_libraries(test_trajectory ${YAML_CPP_LIBRARIES})
target_include_directories(test_trajectory PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Link dependencies for live_pose
ament_target_dependencies(live_pose
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  tf2
  tf2_ros
  ament_index_cpp
)

# Link dependencies for trajectory_action_server
ament_target_dependencies(trajectory_action_server
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  tf2
  tf2_ros
  ament_index_cpp
  moveit_trajectory_msgs
)

# Link yaml-cpp manually for trajectory_action_server
target_link_libraries(trajectory_action_server ${YAML_CPP_LIBRARIES})
target_include_directories(trajectory_action_server PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Link dependencies for trajectory_action_client
ament_target_dependencies(trajectory_action_client
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_trajectory_msgs
)

# Link dependencies for pose_recorder
ament_target_dependencies(pose_recorder
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  ament_index_cpp
)

# Link yaml-cpp
target_link_libraries(pose_recorder ${YAML_CPP_LIBRARIES})
target_include_directories(pose_recorder PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Install executables
install(TARGETS
  test_trajectory
  live_pose
  trajectory_action_server
  trajectory_action_client
  pose_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()