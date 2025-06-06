# MoveIt2 Trajectory Executor

A comprehensive ROS2 package for robot trajectory execution with action-based interface, named poses, and live feedback.

## 🚀 Quick Start

```bash
# Clone and build
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --packages-select moveit_trajectory_executor
source install/setup.bash
```

## 📦 Installation

### Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-planners
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
sudo apt install libyaml-cpp-dev

# Install workspace dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build Custom Messages

```bash
# First build the custom message package
colcon build --packages-select moveit_trajectory_msgs

# Then build this package
colcon build --packages-select moveit_trajectory_executor
source install/setup.bash
```

## 🎯 Core Executables

### 1. `trajectory_action_server` ⭐ **PRIMARY**
Action server for trajectory execution with named poses and live feedback.

```bash
# Start the action server
ros2 run moveit_trajectory_executor trajectory_action_server

# With custom parameters
ros2 run moveit_trajectory_executor trajectory_action_server \
  --ros-args -p planning_group:=arm -p planning_timeout:=10.0
```

### 2. `trajectory_action_client` 
Client for sending trajectory goals to the action server.

```bash
# Go to named pose
ros2 run moveit_trajectory_executor trajectory_action_client home

# Go to custom pose coordinates
ros2 run moveit_trajectory_executor trajectory_action_client \
  --ros-args -p target_x:=0.4 -p target_y:=0.0 -p target_z:=0.3

# Available named poses: home, ready, pick, place, inspect
```

### 3. `pose_recorder` 
Record current robot pose and save to named poses configuration.

```bash
# Record current pose as "my_pose"
ros2 run moveit_trajectory_executor pose_recorder my_pose

# Record with custom delay
ros2 run moveit_trajectory_executor pose_recorder workspace_center \
  --ros-args -p initialization_delay_ms:=2000
```

### 4. `live_pose`
Display real-time robot pose information.

```bash
# Show live pose updates
ros2 run moveit_trajectory_executor live_pose

# Custom update rate (500ms)
ros2 run moveit_trajectory_executor live_pose \
  --ros-args -p pose_update_rate_ms:=500
```

## 📁 Configuration

### Named Poses (`config/named_poses.yaml`)
Predefined robot positions for common tasks:

```yaml
named_poses:
  home:
    position: {x: 0.0, y: 0.0, z: 0.5}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  ready:
    position: {x: 0.3, y: 0.0, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

### Kinematics Configuration (`config/kinematics.yaml`)
MoveIt planning group settings - configure for your robot.

## 🔄 Typical Workflow

### 1. Record New Poses
```bash
# Move robot to desired position manually, then:
ros2 run moveit_trajectory_executor pose_recorder pick_position
ros2 run moveit_trajectory_executor pose_recorder place_position
ros2 run moveit_trajectory_executor pose_recorder inspection_point
```

### 2. Start Action Server
```bash
# Terminal 1: Start the action server
ros2 run moveit_trajectory_executor trajectory_action_server
```

### 3. Execute Trajectories
```bash
# Terminal 2: Send trajectory commands
ros2 run moveit_trajectory_executor trajectory_action_client home
ros2 run moveit_trajectory_executor trajectory_action_client pick_position
ros2 run moveit_trajectory_executor trajectory_action_client place_position
```

### 4. Monitor Robot State
```bash
# Terminal 3: Monitor live pose
ros2 run moveit_trajectory_executor live_pose
```

## 🛠️ Action Interface

### Send Goals via Command Line
```bash
# Using ros2 action CLI
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose \
  "{use_named_pose: true, named_pose: 'home', execute_immediately: true}"

# Custom pose
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose \
  "{use_named_pose: false, target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {w: 1.0}}, execute_immediately: true}"
```

### Monitor Action Progress
```bash
# List active actions
ros2 action list

# Monitor feedback
ros2 topic echo /move_to_pose/_action/feedback

# Check action status
ros2 topic echo /move_to_pose/_action/status
```

## 🔍 Debugging & Troubleshooting

### Check System Status
```bash
# Verify nodes are running
ros2 node list

# Check topics
ros2 topic list | grep -E "(joint_states|move_to_pose)"

# Check parameters
ros2 param list | grep trajectory_action_server

# Check action server status
ros2 action list -t
```

### Common Issues

**1. "Failed to fetch current robot state"**
```bash
# Check joint states
ros2 topic echo /joint_states

# Start joint state publisher if missing
ros2 run joint_state_publisher joint_state_publisher
```

**2. "No kinematics plugins defined"**
```bash
# Check kinematics configuration exists
ls install/moveit_trajectory_executor/share/moveit_trajectory_executor/config/

# Update kinematics.yaml for your robot
```

**3. "Planning failed"**
```bash
# Check MoveIt configuration
ros2 param get trajectory_action_server planning_group
ros2 param get trajectory_action_server planning_timeout

# Increase planning timeout
ros2 param set trajectory_action_server planning_timeout 15.0
```

## 📊 Runtime Parameter Changes

```bash
# Change planning timeout
ros2 param set trajectory_action_server planning_timeout 15.0

# Change feedback rate
ros2 param set trajectory_action_server feedback_rate_hz 5.0

# Update pose update rate for live_pose
ros2 param set live_pose pose_update_rate_ms 1000
```

## 🎛️ Advanced Usage

### Pick and Place Sequence
```bash
# Automated sequence
ros2 run moveit_trajectory_executor trajectory_action_client home && \
ros2 run moveit_trajectory_executor trajectory_action_client ready && \
ros2 run moveit_trajectory_executor trajectory_action_client pick && \
ros2 run moveit_trajectory_executor trajectory_action_client place && \
ros2 run moveit_trajectory_executor trajectory_action_client home
```

### Cancel Active Goals
```bash
# Find active goal ID
ros2 topic echo /move_to_pose/_action/status

# Cancel specific goal
ros2 action send_goal /move_to_pose moveit_trajectory_msgs/action/MoveToPose \
  "{}" --cancel-goal <goal_id>
```

### Custom Launch Files
```bash
# Launch everything at once
ros2 launch moveit_trajectory_executor trajectory_system.launch.py

# With custom parameters
ros2 launch moveit_trajectory_executor trajectory_system.launch.py \
  planning_group:=arm planning_timeout:=15.0
```

## 📋 Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `planning_group` | "manipulator" | MoveIt planning group |
| `planning_timeout` | 5.0 | Planning timeout (seconds) |
| `feedback_rate_hz` | 2.0 | Action feedback rate |
| `execute_immediately` | true | Auto-execute after planning |
| `poses_file` | auto | Named poses YAML file path |
| `initialization_delay_ms` | 1000 | MoveIt init delay |

## 🚦 System Requirements

- **ROS2 Humble** or later
- **MoveIt2** with planning plugins
- **yaml-cpp** library
- **Custom message package**: `moveit_trajectory_msgs`

## 📝 Notes

- All executables support graceful shutdown with `Ctrl+C`
- Named poses are automatically reloaded when configuration file changes
- Action server provides real-time feedback during trajectory execution
- Compatible with standard MoveIt2 robot configurations

---

For more examples and advanced configuration, see the source code and configuration files in the package.

## TODO

- Get the trajectory generated between two pooint and stock the minstead of generathging them each time