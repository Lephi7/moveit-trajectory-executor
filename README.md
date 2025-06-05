# MoveIt2 Trajectory Executor

A ROS2 package for executing MoveIt2 trajectories and monitoring robot poses in real-time.

## Overview

This package provides utilities for:
- Executing predefined trajectories using MoveIt2
- Real-time pose monitoring of robot end-effector
- Kinematics parameter loading from configuration files

## Package Structure

```
moveit_trajectory_executor/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies
├── README.md                   # This file
├── config/
│   └── kinematics.yaml        # Kinematics solver configuration
├── include/
│   └── moveit2_scripts/       # Header files (if any)
├── launch/
│   └── test_trajectory.launch.py  # Launch file for trajectory test
└── src/
    ├── live_pose.cpp          # Real-time pose monitoring
    └── test_trajectory.cpp    # Trajectory execution example
```

## Dependencies

- `rclcpp` - ROS2 C++ client library
- `moveit_core` - MoveIt2 core functionality
- `moveit_ros_planning_interface` - MoveIt2 planning interface
- `moveit_ros_planning` - MoveIt2 planning components
- `geometry_msgs` - Geometry message types
- `tf2` & `tf2_ros` - Transform library
- `yaml-cpp` - YAML configuration parsing

## Executables

### 1. test_trajectory

Executes a predefined trajectory with two target poses:
- First pose: `[0.25, 0.25, 0.25]` with orientation `[0, 0, 0, 0.25]`
- Second pose: Offset by `[+0.2, +0.2, 0]` from the first pose

**Features:**
- Automatic kinematics parameter loading from `config/kinematics.yaml`
- Sequential execution of two planned trajectories
- Planning success/failure feedback

### 2. live_pose

Real-time monitoring tool that continuously displays the robot's end-effector pose.

**Features:**
- 50ms update rate (20 Hz)
- Compact pose display format
- Position and orientation monitoring
- Thread-safe execution with SingleThreadedExecutor

## Configuration

### Kinematics Configuration (`config/kinematics.yaml`)

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_attempts: 10
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.5
```

## Building

```bash
# From your ROS2 workspace root
colcon build --packages-select moveit_trajectory_executor
```

## Usage

### Running the Trajectory Test

```bash
# Option 1: Using launch file
ros2 launch moveit_trajectory_executor test_trajectory.launch.py

# Option 2: Direct execution
ros2 run moveit_trajectory_executor test_trajectory
```

### Running Live Pose Monitor

```bash
ros2 run moveit_trajectory_executor live_pose
```

## Output Examples

### test_trajectory Output
```
[INFO] [test_trajectory]: ✔ Loaded kinematics.yaml parameters into node
[INFO] [test_trajectory]: 🎯 Planning successful. Executing trajectory...
[INFO] [test_trajectory]: 🎯 Second planning successful. Executing second trajectory...
```

### live_pose Output
```
[INFO] [live_pose]: Pose: [0.12345, 0.67890, 0.54321] | Rot: [0.000, 0.000, 0.000, 1.000]
[INFO] [live_pose]: Pose: [0.12346, 0.67891, 0.54322] | Rot: [0.001, 0.000, 0.000, 1.000]
```

## Notes

- Ensure your robot's MoveIt2 configuration is properly set up
- The `manipulator` planning group must be defined in your robot's configuration
- Kinematics parameters are automatically loaded from the config file
- Both executables use the same planning group name: `"manipulator"`

## Troubleshooting

- **Planning failures**: Check if target poses are within the robot's workspace
- **Kinematics errors**: Verify the kinematics.yaml configuration matches your robot
- **Connection issues**: Ensure MoveIt2 move_group node is running