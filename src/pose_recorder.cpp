// src/pose_recorder.cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "\n🤖 Robot Pose Recorder Tool\n" << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  ros2 run moveit_trajectory_executor pose_recorder <pose_name>" << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  ros2 run moveit_trajectory_executor pose_recorder home" << std::endl;
        std::cout << "  ros2 run moveit_trajectory_executor pose_recorder workspace_center" << std::endl;
        std::cout << "  ros2 run moveit_trajectory_executor pose_recorder pick_position" << std::endl;
        return 1;
    }

    std::string pose_name = argv[1];

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Declare Node with parameter auto-declaration
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("pose_recorder",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Declare parameters with default values (same as live_pose)
    node->declare_parameter("initialization_delay_ms", 1000);
    node->declare_parameter("planning_group", std::string("manipulator"));
    node->declare_parameter("poses_file", std::string(""));

    // Get parameter values
    int initialization_delay_ms = node->get_parameter("initialization_delay_ms").as_int();
    std::string planning_group = node->get_parameter("planning_group").as_string();
    std::string poses_file = node->get_parameter("poses_file").as_string();

    // Determine poses file path
    if (poses_file.empty()) {
        std::string package_path = ament_index_cpp::get_package_share_directory("moveit_trajectory_executor");
        poses_file = package_path + "/config/named_poses.yaml";
    }

    // Log configuration
    RCLCPP_INFO(node->get_logger(), "📝 Pose Recorder Configuration");
    RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
    RCLCPP_INFO(node->get_logger(), "Poses file: %s", poses_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Recording pose: '%s'", pose_name.c_str());

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state (same as live_pose)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // Wait for initialization (using parameter)
    RCLCPP_INFO(node->get_logger(), "⏳ Waiting %d ms for MoveIt initialization...", initialization_delay_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(initialization_delay_ms));

    try {
        // Create the MoveIt MoveGroup Interface (same as live_pose)
        moveit::planning_interface::MoveGroupInterface move_group_interface =
            moveit::planning_interface::MoveGroupInterface(node, planning_group);

        RCLCPP_INFO(node->get_logger(), "✅ MoveGroupInterface initialized");

        // Get current pose (same approach as live_pose)
        RCLCPP_INFO(node->get_logger(), "📍 Capturing current robot pose...");
        
        geometry_msgs::msg::PoseStamped pose_stamped = move_group_interface.getCurrentPose();
        geometry_msgs::msg::Pose current_pose = pose_stamped.pose;

        // Log the captured pose
        RCLCPP_INFO(node->get_logger(), "📸 Captured pose:");
        RCLCPP_INFO(node->get_logger(), "   Position: [%.5f, %.5f, %.5f]",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "   Orientation: [%.5f, %.5f, %.5f, %.5f]",
                   current_pose.orientation.x, current_pose.orientation.y,
                   current_pose.orientation.z, current_pose.orientation.w);

        // Save to YAML file
        YAML::Node poses_config;
        
        // Try to load existing file
        try {
            poses_config = YAML::LoadFile(poses_file);
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "Creating new poses file (couldn't load existing): %s", e.what());
            poses_config["named_poses"] = YAML::Node(YAML::NodeType::Map);
        }

        // Ensure named_poses section exists
        if (!poses_config["named_poses"]) {
            poses_config["named_poses"] = YAML::Node(YAML::NodeType::Map);
        }

        // Add or update the pose
        YAML::Node pose_node;
        pose_node["position"]["x"] = current_pose.position.x;
        pose_node["position"]["y"] = current_pose.position.y;
        pose_node["position"]["z"] = current_pose.position.z;
        pose_node["orientation"]["x"] = current_pose.orientation.x;
        pose_node["orientation"]["y"] = current_pose.orientation.y;
        pose_node["orientation"]["z"] = current_pose.orientation.z;
        pose_node["orientation"]["w"] = current_pose.orientation.w;

        poses_config["named_poses"][pose_name] = pose_node;

        // Write back to file with header
        std::ofstream fout(poses_file);
        fout << "# Named poses for robot trajectory execution\n";
        fout << "# Generated by pose_recorder tool\n";
        fout << "# Format: pose_name -> position{x,y,z} + orientation{x,y,z,w}\n\n";
        fout << poses_config;
        fout.close();

        RCLCPP_INFO(node->get_logger(), "✅ Successfully saved pose '%s' to %s", pose_name.c_str(), poses_file.c_str());
        RCLCPP_INFO(node->get_logger(), "🎯 Pose is now available for trajectory_action_server!");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to record pose: %s", e.what());
        
        // Shutdown ROS
        executor.cancel();
        if (spinner.joinable()) {
            spinner.join();
        }
        rclcpp::shutdown();
        return 1;
    }

    // Shutdown ROS
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}