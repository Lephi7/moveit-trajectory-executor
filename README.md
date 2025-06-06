# MoveIt2 Trajectory Executor

A comprehensive ROS2 package for executing MoveIt2 trajectories with action-based interface, real-time monitoring, and predefined pose management.

## Overview

This package provides utilities for:
- **Action-based trajectory execution** with live feedback and cancellation support
- **Named pose management** for common robot positions (home, ready, pick, place, etc.)
- **Real-time pose monitoring** of robot end-effector with configurable update rates
- **Comprehensive parameter management** via YAML configuration files
- **Flexible launch file system** with runtime parameter overrides
- **Kinematics parameter loading** from configuration files
- **Legacy trajectory execution** for simple use cases

## Package Structure

```
moveit_trajectory_executor/
├── CMakeLists.txt                      # Build configuration
├── package.xml                         # Package dependencies
├── README.md                           # This documentation
├── config/
│   ├── kinematics.yaml                # Kinematics solver configuration
│   ├── live_pose_config.yaml          # Live pose monitoring parameters
│   └── named_poses.yaml               # Predefined robot poses
├── include/
│   └── moveit2_scripts/               # Header files (if any)
├── launch/
│   ├── test_trajectory.launch.py      # Launch file for trajectory test
│   ├── live_pose_config.launch.py     # Launch file for live pose with config
│   ├── action_server.launch.py        # Launch file for action server
│   └── action_demo.launch.py          # Demo launch file with sequence
└── src/
    ├── live_pose.cpp                  # Configurable real-time pose monitoring
    ├── test_trajectory.cpp            # Legacy trajectory execution with YAML loading
    ├── trajectory_action_server.cpp   # Action server for trajectory execution
    └── trajectory_action_client.cpp   # Action client for sending goals
```

## Installation

### Prerequisites

Ensure you have ROS2 Humble installed. If not, follow the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### Installing Dependencies

#### 1. Core ROS2 Development Tools
```bash
# Update package lists
sudo apt update

# Install ROS2 development essentials
sudo apt install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
```

#### 2. MoveIt2 Dependencies
```bash
# Install MoveIt2 packages
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-core \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo
```

#### 3. Additional ROS2 Packages
```bash
# Install additional required packages
sudo apt install -y \
    ros-humble-geometry-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-ament-index-cpp \
    ros-humble-ament-index-python \
    ros-humble-rclcpp-action
```

#### 4. Build System Dependencies
```bash
# Install CMake and build tools
sudo apt install -y \
    build-essential \
    cmake \
    python3-ament-package \
    python3-ament-cmake \
    ros-humble-ament-cmake-core
```

#### 5. YAML and Additional Libraries
```bash
# Install YAML processing library
sudo apt install -y \
    libyaml-cpp-dev \
    yaml-cpp \
    pkg-config

# Install Python YAML support (if needed)
pip3 install pyyaml
```

#### 6. Optional: Development Tools
```bash
# Install useful development tools
sudo apt install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui
```

### Initialize rosdep (First-time setup)
```bash
# Initialize rosdep if not already done
sudo rosdep init
rosdep update
```

### Install Package-Specific Dependencies
```bash
# Navigate to your workspace
cd /path/to/your/workspace

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

## Dependencies

### Core Dependencies
- `rclcpp` - ROS2 C++ client library
- `rclcpp_action` - ROS2 Action client/server library
- `moveit_core` - MoveIt2 core functionality
- `moveit_ros_planning_interface` - MoveIt2 planning interface
- `moveit_ros_planning` - MoveIt2 planning components
- `moveit_msgs` - MoveIt2 message types
- `geometry_msgs` - Geometry message types
- `tf2` & `tf2_ros` - Transform library
- `ament_index_cpp` - Package resource location
- `yaml-cpp` - YAML configuration parsing
- `launch` & `launch_ros` - Launch system support
- `ament_index_python` - Python package utilities

### Custom Message Dependencies
- **`moveit_trajectory_msgs`** - Custom action message package (to be published separately)
  - Contains the `MoveToPose.action` definition
  - Required for action-based trajectory execution
  - Install from: *[Package location will be provided when published]*

## Executables

### 1. trajectory_action_server ⭐ **PRIMARY**

**Action server for trajectory execution with named pose support and live feedback.**

**Features:**
- **Action-based interface** with goal/feedback/result pattern
- **Named pose support** (home, ready, pick, place, inspect, custom poses)
- **Live feedback** during planning and execution with progress updates
- **Cancellation support** - stop trajectories mid-execution
- **Error handling** with detailed error messages
- **Configurable parameters** (planning timeout, feedback rate, etc.)
- **YAML configuration loading** for kinematics and named poses
- **Real-time pose feedback** during trajectory execution

**Goal Parameters:**
- `target_pose` - Custom pose coordinates
- `named_pose` - Predefined pose name (e.g., "home", "ready")
- `use_named_pose` - Boolean to use named pose instead of coordinates
- `planning_group` - MoveIt planning group (default: "manipulator")
- `planning_timeout` - Planning timeout in seconds
- `execute_immediately` - Execute trajectory immediately after planning

### 2. trajectory_action_client

**Client for sending trajectory goals to the action server.**

**Features:**
- **Named pose goals** - Send robot to predefined positions
- **Custom pose goals** - Send robot to specific coordinates
- **Live feedback display** - Real-time progress and pose updates
- **Command line arguments** - Specify target pose via arguments
- **Error handling** - Graceful handling of planning/execution failures

**Usage:**
```bash
# Go to named pose
ros2 run moveit_trajectory_executor trajectory_action_client home
ros2 run moveit_trajectory_executor trajectory_action_client ready

# Default (goes to "home")
ros2 run moveit_trajectory_executor trajectory_action_client
```

### 3. live_pose

**Real-time monitoring tool for robot end-effector pose.**

**Features:**
- **Configurable update rates** (default: 50ms = 20Hz)
- **Adjustable precision** for position and orientation display
- **Optional joint values display**
- **Compact or detailed output formats**
- **Error handling** with configurable retry limits
- **Parameter loading** from YAML files or command line
- **Runtime parameter changes** via ROS2 parameter system

### 4. test_trajectory (Legacy)

**Simple trajectory execution for testing purposes.**

**Features:**
- Executes predefined trajectory with two target poses
- First pose: `[0.25, 0.25, 0.25]` with orientation `[0, 0, 0, 0.25]`
- Second pose: Offset by `[+0.2, +0.2, 0]` from the first pose
- Automatic kinematics parameter loading from `config/kinematics.yaml`
- Sequential execution of two planned trajectories
- Planning success/failure feedback

## Configuration Files

### Named Poses Configuration (`config/named_poses.yaml`)

```yaml
named_poses:
  home:
    position:
      x: 0.0
      y: 0.0
      z: 0.5
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

### Kinematics Configuration (`config/kinematics.yaml`)

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_attempts: 10
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.5
```

### Live Pose Configuration (`config/live_pose_config.yaml`)

```yaml
live_pose:
  ros__parameters:
    # Timer and rate parameters (in milliseconds)
    pose_update_rate_ms: 50        # 50ms = 20Hz update rate
    initialization_delay_ms: 1000  # Wait 1 second for MoveIt initialization
    
    # MoveIt configuration
    planning_group: "manipulator"   # Name of the planning group
    
    # Display parameters
    position_precision: 5           # Number of decimal places for position (%.5f)
    orientation_precision: 3        # Number of decimal places for orientation (%.3f)
    show_joint_values: false        # Whether to display joint values
    compact_output: true            # Use compact or detailed output format
    
    # Error handling
    max_consecutive_errors: 10      # Max errors before shutdown
    error_recovery_delay_ms: 500    # Delay after error before retry
```

## Building

### Build Order

This package depends on the custom action messages, so build in the correct order:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd /path/to/your/ros2_workspace

# Install dependencies (if not done already)
rosdep install --from-paths src --ignore-src -r -y

# Build the custom message package first
colcon build --packages-select moveit_trajectory_msgs
source install/setup.bash

# Build the main package
colcon build --packages-select moveit_trajectory_executor
source install/setup.bash
```

### Clean Build
```bash
# If you encounter build issues
rm -rf build/ install/ log/
colcon build --packages-select moveit_trajectory_msgs moveit_trajectory_executor
source install/setup.bash
```

## Usage

### Action-Based Trajectory Execution (Recommended)

#### Start the Action Server
```bash
# Terminal 1: Start the action server
ros2 run moveit_trajectory_executor trajectory_action_server

# Expected output:
# [INFO] [trajectory_action_server]: ✔ Loaded kinematics.yaml parameters
# [INFO] [trajectory_action_server]: ✔ Loaded 5 named poses
# [INFO] [trajectory_action_server]: 📍 Available named poses:
# [INFO] [trajectory_action_server]:   • home: [0.000, 0.000, 0.500]
# [INFO] [trajectory_action_server]:   • ready: [0.300, 0.000, 0.300]
# [INFO] [trajectory_action_server]:   • pick: [0.400, 0.200, 0.150]
# [INFO] [trajectory_action_server]:   • place: [0.200, -0.300, 0.250]
# [INFO] [trajectory_action_server]:   • inspect: [0.500, 0.000, 0.400]
# [INFO] [trajectory_action_server]: 🚀 Trajectory Action Server ready!
```

#### Send Goals with Named Poses
```bash
# Terminal 2: Send goals to named poses
ros2 run moveit_trajectory_executor trajectory_action_client home
ros2 run moveit_trajectory_executor trajectory_action_client ready
ros2 run moveit_trajectory_executor trajectory_action_client pick
ros2 run moveit_trajectory_executor trajectory_action_client place
ros2 run moveit_trajectory_executor trajectory_action_client inspect

# Expected output:
# [INFO] [trajectory_action_client]: 🚀 Sending goal to named pose 'home'...
# [INFO] [trajectory_action_client]: ✅ Goal accepted by server, waiting for result
# [INFO] [trajectory_action_client]: 📊 Planning trajectory to 'home'... | Progress: 10.0% | Pose: [0.123, 0.456, 0.789]
# [INFO] [trajectory_action_client]: 📊 Executing trajectory... | Progress: 50.0% | Pose: [0.100, 0.200, 0.600]
# [INFO] [trajectory_action_client]: 📊 Moving to 'home'... | Progress: 75.0% | Pose: [0.050, 0.100, 0.550]
# [INFO] [trajectory_action_client]: 🎉 Goal succeeded! Execution time: 2.34 seconds
```

#### Send Goals with Command Line
```bash
# Named pose goal
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose "{
  use_named_pose: true,
  named_pose: 'ready',
  planning_group: 'manipulator',
  execute_immediately: true
}" --feedback

# Custom pose goal
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose "{
  use_named_pose: false,
  target_pose: {
    position: {x: 0.3, y: 0.2, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  planning_group: 'manipulator',
  execute_immediately: true
}" --feedback
```

### Running Live Pose Monitor

```bash
# Option 1: Using launch file with automatic config loading
ros2 launch moveit_trajectory_executor live_pose_config.launch.py

# Option 2: Direct execution with default parameters
ros2 run moveit_trajectory_executor live_pose

# Option 3: Direct execution with parameter file
ros2 run moveit_trajectory_executor live_pose --ros-args --params-file install/moveit_trajectory_executor/share/moveit_trajectory_executor/config/live_pose_config.yaml

# Option 4: Override specific parameters via command line
ros2 run moveit_trajectory_executor live_pose --ros-args -p pose_update_rate_ms:=100 -p show_joint_values:=true

# Option 5: Launch file with parameter overrides
ros2 launch moveit_trajectory_executor live_pose_config.launch.py pose_update_rate_ms:=25 show_joint_values:=true
```

### Legacy Trajectory Execution

```bash
# Option 1: Using launch file
ros2 launch moveit_trajectory_executor test_trajectory.launch.py

# Option 2: Direct execution
ros2 run moveit_trajectory_executor test_trajectory
```

### Runtime Parameter Changes

```bash
# Change update rate while running
ros2 param set /live_pose pose_update_rate_ms 200

# Enable joint values display
ros2 param set /live_pose show_joint_values true

# List all available parameters
ros2 param list /live_pose
```

## Action Interface

### Action Definition (`MoveToPose.action`)

```yaml
# Goal - The target pose to reach
geometry_msgs/Pose target_pose
string planning_group
float64 planning_timeout
bool execute_immediately
string named_pose
bool use_named_pose

---

# Result - Success status and final pose
bool success
string error_message
geometry_msgs/Pose final_pose
float64 execution_time

---

# Feedback - Live position updates during execution
geometry_msgs/Pose current_pose
string status
float64 progress_percentage
```

### Action Topics

```bash
# Action server topics
/move_to_pose/_action/send_goal          # Send goals
/move_to_pose/_action/cancel_goal        # Cancel goals
/move_to_pose/_action/get_result         # Get results
/move_to_pose/_action/feedback           # Live feedback
/move_to_pose/_action/status             # Goal status

# Monitor action activity
ros2 topic echo /move_to_pose/_action/feedback
ros2 topic echo /move_to_pose/_action/status
```

## Output Examples

### Action Server Startup
```
[INFO] [trajectory_action_server]: ✔ Loaded kinematics.yaml parameters
[INFO] [trajectory_action_server]: ✔ Loaded 5 named poses
[INFO] [trajectory_action_server]: 📍 Available named poses:
[INFO] [trajectory_action_server]:   • home: [0.000, 0.000, 0.500]
                                ◦◦◦
[INFO] [trajectory_action_server]:   • inspect: [0.500, 0.000, 0.400]
[INFO] [trajectory_action_server]: 🚀 Trajectory Action Server ready!
```

### Action Execution with Feedback
```
[INFO] [trajectory_action_client]: 🚀 Sending goal to named pose 'pick'...
[INFO] [trajectory_action_client]: ✅ Goal accepted by server, waiting for result
[INFO] [trajectory_action_client]: 📊 Planning trajectory to 'pick'... | Progress: 10.0% | Pose: [0.123, 0.456, 0.789]
[INFO] [trajectory_action_client]: 📊 Executing trajectory... | Progress: 50.0% | Pose: [0.250, 0.300, 0.400]
[INFO] [trajectory_action_client]: 📊 Moving to 'pick'... | Progress: 75.0% | Pose: [0.350, 0.180, 0.200]
[INFO] [trajectory_action_client]: 📊 Moving to 'pick'... | Progress: 90.0% | Pose: [0.390, 0.195, 0.160]
[INFO] [trajectory_action_client]: 🎉 Goal succeeded! Execution time: 3.45 seconds
[INFO] [trajectory_action_client]: Final pose: [0.400, 0.200, 0.150]
```

### live_pose Output (Compact Mode)
```
[INFO] [live_pose]: === Live Pose Monitor Configuration ===
[INFO] [live_pose]: Pose update rate: 50 ms (20.0 Hz)
[INFO] [live_pose]: Initialization delay: 1000 ms
[INFO] [live_pose]: Planning group: manipulator
[INFO] [live_pose]: Position precision: 5 decimals
[INFO] [live_pose]: Orientation precision: 3 decimals
[INFO] [live_pose]: Show joint values: false
[INFO] [live_pose]: ======================================
[INFO] [live_pose]: Pose: [0.12345, 0.67890, 0.54321] | Rot: [0.000, 0.000, 0.000, 1.000]
[INFO] [live_pose]: Pose: [0.12346, 0.67891, 0.54322] | Rot: [0.001, 0.000, 0.000, 1.000]
```

### test_trajectory Output (Legacy)
```
[INFO] [test_trajectory]: ✔ Loaded kinematics.yaml parameters into node
[INFO] [test_trajectory]: 🎯 Planning successful. Executing trajectory...
[INFO] [test_trajectory]: 🎯 Second planning successful. Executing second trajectory...
```

## Parameter Reference

### Action Server Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `default_planning_group` | string | "manipulator" | Default MoveIt planning group |
| `feedback_rate_hz` | double | 10.0 | Feedback update rate in Hz |
| `position_tolerance` | double | 0.01 | Position tolerance for goal achievement |

### Live Pose Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pose_update_rate_ms` | int | 50 | Update rate in milliseconds (20Hz) |
| `initialization_delay_ms` | int | 1000 | MoveIt initialization delay |
| `planning_group` | string | "manipulator" | MoveIt planning group name |
| `position_precision` | int | 5 | Decimal places for position values |
| `orientation_precision` | int | 3 | Decimal places for orientation values |
| `show_joint_values` | bool | false | Display joint angles |
| `compact_output` | bool | true | Use compact display format |
| `max_consecutive_errors` | int | 10 | Max errors before shutdown |
| `error_recovery_delay_ms` | int | 500 | Delay after error before retry |

## Advanced Usage

### Pick and Place Sequence
```bash
# Automated pick and place sequence
ros2 run moveit_trajectory_executor trajectory_action_client home && \
ros2 run moveit_trajectory_executor trajectory_action_client ready && \
ros2 run moveit_trajectory_executor trajectory_action_client pick && \
ros2 run moveit_trajectory_executor trajectory_action_client place && \
ros2 run moveit_trajectory_executor trajectory_action_client home
```

### Custom Named Poses

Add your own poses to `config/named_poses.yaml`:

### Action Monitoring

```bash
# List available actions
ros2 action list

# Get action info
ros2 action info /move_to_pose

# Show action interface
ros2 interface show moveit_trajectory_msgs/action/MoveToPose

# Monitor all action activity
ros2 topic echo /move_to_pose/_action/feedback
```

### Goal Cancellation

```bash
# Send a goal and cancel it
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose "{use_named_pose: true, named_pose: 'inspect', planning_group: 'manipulator', execute_immediately: true}" --feedback &
# Wait a moment, then cancel
ros2 action send_goal --cancel-all /move_to_pose
```

## Troubleshooting

### Common Installation Issues

**Missing dependencies:**
```bash
# If you get "package not found" errors during build
rosdep install --from-paths src --ignore-src -r -y

# If yaml-cpp is not found
sudo apt install libyaml-cpp-dev

# If ament packages are missing
sudo apt install python3-ament-package ros-humble-ament-cmake-core

# If action messages are missing
sudo apt install ros-humble-rclcpp-action
```

**Build errors:**
```bash
# Clean build if you encounter weird errors
rm -rf build/ install/ log/
colcon build --packages-select moveit_trajectory_msgs moveit_trajectory_executor
```

**Action message not found:**
```bash
# Make sure to build moveit_trajectory_msgs first
colcon build --packages-select moveit_trajectory_msgs
source install/setup.bash
colcon build --packages-select moveit_trajectory_executor
```

### Runtime Issues

**Action server not available:**
- Ensure `trajectory_action_server` is running
- Check that `moveit_trajectory_msgs` package is properly installed
- Verify action server initialization completed successfully

**Planning failures:**
- Check if target poses are within the robot's workspace
- Verify the kinematics.yaml configuration matches your robot
- Ensure MoveIt2 move_group node is running

**Named pose not found:**
- Check `named_poses.yaml` file exists and is properly formatted
- Verify pose names match exactly (case-sensitive)
- Check server startup logs for loading errors

**Connection issues:**
- Ensure MoveIt2 move_group node is running
- Check that planning group name matches your robot configuration
- Verify robot description is loaded

**Parameter file not found:**
- Check that the package is properly built and installed
- Verify config files are in the correct location
- Use absolute paths if relative paths fail

**High CPU usage (live_pose):**
- Increase `pose_update_rate_ms` to reduce update frequency
- Disable joint values display if not needed
- Use compact output format

### Environment Setup

Make sure to source the ROS2 environment in every new terminal:

```bash
# Add this to your ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc

# Or source manually each time
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash
```

### Debug Commands

```bash
# Check action server status
ros2 node info /trajectory_action_server

# Monitor action feedback in real-time
ros2 topic echo /move_to_pose/_action/feedback

# Check parameter values
ros2 param list /trajectory_action_server
ros2 param get /trajectory_action_server default_planning_group

# Verify named poses are loaded
ros2 run moveit_trajectory_executor trajectory_action_server | grep "Available named poses" -A 10
```

## Notes

- **This package requires the separate `moveit_trajectory_msgs` package** for action message definitions
- Ensure your robot's MoveIt2 configuration is properly set up
- The `manipulator` planning group must be defined in your robot's configuration
- Configuration files are automatically installed and can be found in the package share directory
- Action-based interface is the recommended approach for new applications
- Legacy executables (`test_trajectory`) are maintained for backward compatibility
- Launch files provide the most convenient way to manage complex parameter sets
- This package requires ROS2 Humble or later
- Named poses can be easily customized by editing `config/named_poses.yaml`
- The action server provides comprehensive error handling and live feedback
- All executables support graceful shutdown with Ctrl+C