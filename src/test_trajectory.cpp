#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>

void loadKinematicsParameters(std::shared_ptr<rclcpp::Node> node)
{
    std::string package_path = ament_index_cpp::get_package_share_directory("moveit_trajectory_executor");
    std::string kinematics_file = package_path + "/config/kinematics.yaml";

    YAML::Node kinematics = YAML::LoadFile(kinematics_file);
    for (const auto& group : kinematics) {
        std::string group_name = group.first.as<std::string>();
        for (const auto& param : group.second) {
            std::string param_name = "robot_description_kinematics." + group_name + "." + param.first.as<std::string>();
            if (param.second.IsScalar()) {
                node->declare_parameter(param_name, param.second.as<std::string>());
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "✔ Loaded kinematics.yaml parameters into node");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_trajectory");

    // Charger les paramètres cinématiques manuellement
    loadKinematicsParameters(node);

    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 0.25;
    target_pose.position.x = 0.25;
    target_pose.position.y = 0.25;
    target_pose.position.z = 0.25;  

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "🎯 Planning successful. Executing trajectory...");
        move_group.move();
    } else {
        RCLCPP_WARN(node->get_logger(), "❌ Planning failed.");
    }

    // Plan and execute a second target pose after the first one, regardless of first success
    geometry_msgs::msg::Pose second_pose = target_pose;
    second_pose.position.x += 0.2;  // Move 10cm further in x
    second_pose.position.y += 0.2;  // Move 10cm further in y

    move_group.setPoseTarget(second_pose);

    moveit::planning_interface::MoveGroupInterface::Plan second_plan;
    bool second_success = (move_group.plan(second_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (second_success) {
        RCLCPP_INFO(node->get_logger(), "🎯 Second planning successful. Executing second trajectory...");
        move_group.move();
    } else {
        RCLCPP_WARN(node->get_logger(), "❌ Second planning failed.");
    }



    rclcpp::shutdown();
    return 0;
}
