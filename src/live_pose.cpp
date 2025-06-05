#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Declare Node
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("live_pose",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // Wait a bit for the executor to start
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface =
        moveit::planning_interface::MoveGroupInterface(node, "manipulator");

    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized. Starting pose monitoring every 0.05 seconds...");

    // Print current pose every 0.05 seconds indefinitely
    while (rclcpp::ok()) {
        try {
            // Get current pose
            geometry_msgs::msg::PoseStamped pose_stamped = move_group_interface.getCurrentPose();
            geometry_msgs::msg::Pose current_pose = pose_stamped.pose;

            // Print the current pose of the end effector - compact format
            RCLCPP_INFO(node->get_logger(), 
                "Pose: [%.5f, %.5f, %5f] | Rot: [%.3f, %.3f, %.3f, %.3f]",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);

            // Optional: Also print joint values for debugging - commented out for cleaner output
            /*
            auto joint_names = move_group_interface.getActiveJoints();
            auto current_state = move_group_interface.getCurrentState();
            if (current_state) {
                std::vector<double> joint_values;
                current_state->copyJointGroupPositions("manipulator", joint_values);
                
                std::string joint_info = "Joints: ";
                for (size_t j = 0; j < joint_values.size() && j < joint_names.size(); ++j) {
                    joint_info += joint_names[j] + "=" + std::to_string(joint_values[j]).substr(0,5) + " ";
                }
                RCLCPP_INFO(node->get_logger(), "%s", joint_info.c_str());
            }
            */

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Error getting current pose: %s", e.what());
        }

        // Wait 0.05 seconds (50 milliseconds) before next reading
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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