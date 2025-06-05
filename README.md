# MoveIt2 Trajectory Executor

A configurable ROS2 package for executing MoveIt2 trajectories and monitoring robot poses in real-time with comprehensive parameter management.

## Overview

This package provides utilities for:
- Executing predefined trajectories using MoveIt2
- Real-time pose monitoring of robot end-effector with configurable update rates
- Comprehensive parameter management via YAML configuration files
- Flexible launch file system with runtime parameter overrides
- Kinematics parameter loading from configuration files

## Package Structure

```
moveit_trajectory_executor/
├── CMakeLists.txt                      # Build configuration
├── package.xml                         # Package dependencies
├── README.md                           # This file
├── config/
│   ├── kinematics.yaml                # Kinematics solver configuration
│   └── live_pose_config.yaml          # Live pose monitoring parameters
├── include/
│   └── moveit2_scripts/               # Header files (if any)
├── launch/
│   ├── test_trajectory.launch.py      # Launch file for trajectory test
│   └── live_pose_config.launch.py     # Launch file for live pose with config
└── src/
    ├── live_pose.cpp                  # Configurable real-time pose monitoring
    └── test_trajectory.cpp            # Trajectory execution with YAML loading
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
    ros-humble-ament-index-python
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

- `rclcpp` - ROS2 C++ client library
- `moveit_core` - MoveIt2 core functionality
- `moveit_ros_planning_interface` - MoveIt2 planning interface
- `moveit_ros_planning` - MoveIt2 planning components
- `geometry_msgs` - Geometry message types
- `tf2` & `tf2_ros` - Transform library
- `ament_index_cpp` - Package resource location
- `yaml-cpp` - YAML configuration parsing
- `launch` & `launch_ros` - Launch system support
- `ament_index_python` - Python package utilities

## Executables

### 1. test_trajectory

Executes a predefined trajectory with two target poses:
- First pose: `[0.25, 0.25, 0.25]` with orientation `[0, 0, 0, 0.25]`
- Second pose: Offset by `[+0.2, +0.2, 0]` from the first pose

**Features:**
- Automatic kinematics parameter loading from `config/kinematics.yaml`
- Sequential execution of two planned trajectories
- Planning success/failure feedback
- YAML configuration file integration

### 2. live_pose

Real-time monitoring tool that continuously displays the robot's end-effector pose with extensive configurability.

**Features:**
- **Configurable update rates** (default: 50ms = 20Hz)
- **Adjustable precision** for position and orientation display
- **Optional joint values display**
- **Compact or detailed output formats**
- **Error handling** with configurable retry limits
- **Parameter loading** from YAML files or command line
- **Runtime parameter changes** via ROS2 parameter system

## Configuration Files

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

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd /path/to/your/ros2_workspace

# Install dependencies (if not done already)
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select moveit_trajectory_executor

# Source the workspace
source install/setup.bash
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

### Runtime Parameter Changes

```bash
# Change update rate while running
ros2 param set /live_pose pose_update_rate_ms 200

# Enable joint values display
ros2 param set /live_pose show_joint_values true

# List all available parameters
ros2 param list /live_pose
```

## Output Examples

### test_trajectory Output
```
[INFO] [test_trajectory]: ✔ Loaded kinematics.yaml parameters into node
[INFO] [test_trajectory]: 🎯 Planning successful. Executing trajectory...
[INFO] [test_trajectory]: 🎯 Second planning successful. Executing second trajectory...
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

### live_pose Output (Detailed Mode with Joint Values)
```
[INFO] [live_pose]: Frame: panda_link0 | Position: [x=0.12345, y=0.67890, z=0.54321] | Orientation: [x=0.000, y=0.000, z=0.000, w=1.000]
[INFO] [live_pose]: Joints: panda_joint1=0.12 panda_joint2=-0.45 panda_joint3=0.78 panda_joint4=-1.23 panda_joint5=0.56 panda_joint6=1.89 panda_joint7=0.34
```

## Parameter Reference

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
```

**Build errors:**
```bash
# Clean build if you encounter weird errors
rm -rf build/ install/ log/
colcon build --packages-select moveit_trajectory_executor
```

### Runtime Issues

- **Planning failures**: Check if target poses are within the robot's workspace
- **Kinematics errors**: Verify the kinematics.yaml configuration matches your robot
- **Connection issues**: Ensure MoveIt2 move_group node is running
- **Parameter file not found**: Check that the package is properly built and installed
- **High CPU usage**: Increase `pose_update_rate_ms` to reduce update frequency
- **Missing launch dependencies**: Ensure all ROS2 packages are installed as shown above

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

## Notes

- Ensure your robot's MoveIt2 configuration is properly set up
- The `manipulator` planning group must be defined in your robot's configuration
- Configuration files are automatically installed and can be found in the package share directory
- Both executables support the same planning group parameter system
- Launch files provide the most convenient way to manage complex parameter sets
- This package requires ROS2 Humble or later