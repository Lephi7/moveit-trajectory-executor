#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Declare Node with parameter auto-declaration
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("live_pose",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Declare parameters with default values
    node->declare_parameter("pose_update_rate_ms", 50);
    node->declare_parameter("initialization_delay_ms", 1000);
    node->declare_parameter("planning_group", std::string("manipulator"));
    node->declare_parameter("position_precision", 5);
    node->declare_parameter("orientation_precision", 3);
    node->declare_parameter("show_joint_values", false);

    // Get parameter values
    int pose_update_rate_ms = node->get_parameter("pose_update_rate_ms").as_int();
    int initialization_delay_ms = node->get_parameter("initialization_delay_ms").as_int();
    std::string planning_group = node->get_parameter("planning_group").as_string();
    int position_precision = node->get_parameter("position_precision").as_int();
    int orientation_precision = node->get_parameter("orientation_precision").as_int();
    bool show_joint_values = node->get_parameter("show_joint_values").as_bool();

    // Log the parameter values
    RCLCPP_INFO(node->get_logger(), "=== Live Pose Monitor Configuration ===");
    RCLCPP_INFO(node->get_logger(), "Pose update rate: %d ms (%.1f Hz)", 
                pose_update_rate_ms, 1000.0 / pose_update_rate_ms);
    RCLCPP_INFO(node->get_logger(), "Initialization delay: %d ms", initialization_delay_ms);
    RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
    RCLCPP_INFO(node->get_logger(), "Position precision: %d decimals", position_precision);
    RCLCPP_INFO(node->get_logger(), "Orientation precision: %d decimals", orientation_precision);
    RCLCPP_INFO(node->get_logger(), "Show joint values: %s", show_joint_values ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "======================================");

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // Wait for initialization (using parameter)
    RCLCPP_INFO(node->get_logger(), "Waiting %d ms for MoveIt initialization...", initialization_delay_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(initialization_delay_ms));

    // Create the MoveIt MoveGroup Interface (using parameter)
    moveit::planning_interface::MoveGroupInterface move_group_interface =
        moveit::planning_interface::MoveGroupInterface(node, planning_group);

    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized. Starting pose monitoring every %d ms...", pose_update_rate_ms);

    // Print current pose using configurable rate
    while (rclcpp::ok()) {
        try {
            // Get current pose
            geometry_msgs::msg::PoseStamped pose_stamped = move_group_interface.getCurrentPose();
            geometry_msgs::msg::Pose current_pose = pose_stamped.pose;

            // Print the current pose using configurable precision
            RCLCPP_INFO(node->get_logger(), 
                "Pose: [%.*f, %.*f, %.*f] | Rot: [%.*f, %.*f, %.*f, %.*f]",
                position_precision, current_pose.position.x,
                position_precision, current_pose.position.y,
                position_precision, current_pose.position.z,
                orientation_precision, current_pose.orientation.x,
                orientation_precision, current_pose.orientation.y,
                orientation_precision, current_pose.orientation.z,
                orientation_precision, current_pose.orientation.w);

            // Show joint values if enabled via parameter
            if (show_joint_values) {
                auto joint_names = move_group_interface.getActiveJoints();
                auto current_state = move_group_interface.getCurrentState();
                if (current_state) {
                    std::vector<double> joint_values;
                    current_state->copyJointGroupPositions(planning_group, joint_values);
                    
                    std::string joint_info = "Joints: ";
                    for (size_t j = 0; j < joint_values.size() && j < joint_names.size(); ++j) {
                        joint_info += joint_names[j] + "=" + std::to_string(joint_values[j]).substr(0, position_precision + 2) + " ";
                    }
                    RCLCPP_INFO(node->get_logger(), "%s", joint_info.c_str());
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Error getting current pose: %s", e.what());
        }

        // Wait using configurable rate (using parameter)
        std::this_thread::sleep_for(std::chrono::milliseconds(pose_update_rate_ms));
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    
    // Shutdown ROS
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}