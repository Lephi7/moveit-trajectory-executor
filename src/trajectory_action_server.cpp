#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_trajectory_msgs/action/move_to_pose.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <chrono>
#include <future>
#include <map>

class TrajectoryActionServer : public rclcpp::Node
{
public:
    using MoveToPose = moveit_trajectory_msgs::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    TrajectoryActionServer() : Node("trajectory_action_server")
    {
        // Declare parameters
        this->declare_parameter("default_planning_group", std::string("manipulator"));
        this->declare_parameter("feedback_rate_hz", 10.0);
        this->declare_parameter("position_tolerance", 0.01);
        
        // Load configuration files
        loadKinematicsParameters();
        loadNamedPoses();
        
        // Create action server
        action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&TrajectoryActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Log available named poses
        logAvailableNamedPoses();
        RCLCPP_INFO(this->get_logger(), "🚀 Trajectory Action Server ready!");
    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    std::map<std::string, geometry_msgs::msg::Pose> named_poses_;
    
    void loadKinematicsParameters()
    {
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory("moveit_trajectory_executor");
            std::string kinematics_file = package_path + "/config/kinematics.yaml";

            YAML::Node kinematics = YAML::LoadFile(kinematics_file);
            for (const auto& group : kinematics) {
                std::string group_name = group.first.as<std::string>();
                for (const auto& param : group.second) {
                    std::string param_name = "robot_description_kinematics." + group_name + "." + param.first.as<std::string>();
                    if (param.second.IsScalar()) {
                        this->declare_parameter(param_name, param.second.as<std::string>());
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "✔ Loaded kinematics.yaml parameters");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not load kinematics parameters: %s", e.what());
        }
    }

    void loadNamedPoses()
    {
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory("moveit_trajectory_executor");
            std::string poses_file = package_path + "/config/named_poses.yaml";

            YAML::Node poses_config = YAML::LoadFile(poses_file);
            
            if (poses_config["named_poses"]) {
                for (const auto& pose_entry : poses_config["named_poses"]) {
                    std::string pose_name = pose_entry.first.as<std::string>();
                    const YAML::Node& pose_data = pose_entry.second;
                    
                    geometry_msgs::msg::Pose pose;
                    
                    // Load position
                    if (pose_data["position"]) {
                        pose.position.x = pose_data["position"]["x"].as<double>();
                        pose.position.y = pose_data["position"]["y"].as<double>();
                        pose.position.z = pose_data["position"]["z"].as<double>();
                    }
                    
                    // Load orientation
                    if (pose_data["orientation"]) {
                        pose.orientation.x = pose_data["orientation"]["x"].as<double>();
                        pose.orientation.y = pose_data["orientation"]["y"].as<double>();
                        pose.orientation.z = pose_data["orientation"]["z"].as<double>();
                        pose.orientation.w = pose_data["orientation"]["w"].as<double>();
                    }
                    
                    named_poses_[pose_name] = pose;
                }
                
                RCLCPP_INFO(this->get_logger(), "✔ Loaded %zu named poses", named_poses_.size());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not load named poses: %s", e.what());
        }
    }

    void logAvailableNamedPoses()
    {
        if (!named_poses_.empty()) {
            RCLCPP_INFO(this->get_logger(), "📍 Available named poses:");
            for (const auto& [name, pose] : named_poses_) {
                RCLCPP_INFO(this->get_logger(), "  • %s: [%.3f, %.3f, %.3f]", 
                           name.c_str(), pose.position.x, pose.position.y, pose.position.z);
            }
        }
    }

    geometry_msgs::msg::Pose getNamedPose(const std::string& pose_name)
    {
        auto it = named_poses_.find(pose_name);
        if (it != named_poses_.end()) {
            return it->second;
        } else {
            throw std::runtime_error("Named pose '" + pose_name + "' not found");
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToPose::Goal> goal)
    {
        (void)uuid;
        
        if (goal->use_named_pose) {
            RCLCPP_INFO(this->get_logger(), "📥 Received goal request for named pose: '%s'", 
                        goal->named_pose.c_str());
            
            // Check if named pose exists
            if (named_poses_.find(goal->named_pose) == named_poses_.end()) {
                RCLCPP_WARN(this->get_logger(), "❌ Named pose '%s' not found", goal->named_pose.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "📥 Received goal request for custom pose [%.3f, %.3f, %.3f]", 
                        goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
        }
        
        // Validate the goal
        if (goal->planning_group.empty()) {
            RCLCPP_WARN(this->get_logger(), "❌ Empty planning group in goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{std::bind(&TrajectoryActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "🎯 Executing trajectory...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();
        
        auto start_time = std::chrono::steady_clock::now();
        double feedback_rate = this->get_parameter("feedback_rate_hz").as_double();
        auto feedback_period = std::chrono::milliseconds(static_cast<int>(1000.0 / feedback_rate));
        
        try {
            // Determine target pose (named or custom)
            geometry_msgs::msg::Pose target_pose;
            if (goal->use_named_pose) {
                target_pose = getNamedPose(goal->named_pose);
                RCLCPP_INFO(this->get_logger(), "🎯 Using named pose '%s': [%.3f, %.3f, %.3f]",
                           goal->named_pose.c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);
            } else {
                target_pose = goal->target_pose;
            }
            
            // Create MoveGroup interface
            std::string planning_group = goal->planning_group.empty() ? 
                this->get_parameter("default_planning_group").as_string() : goal->planning_group;
                
            moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), planning_group);
            
            // Set planning timeout if specified
            if (goal->planning_timeout > 0.0) {
                move_group.setPlanningTime(goal->planning_timeout);
            }
            
            // Set target pose
            move_group.setPoseTarget(target_pose);
            
            // Update feedback - planning phase
            feedback->status = goal->use_named_pose ? 
                ("Planning trajectory to '" + goal->named_pose + "'...") : 
                "Planning trajectory to custom pose...";
            feedback->progress_percentage = 10.0;
            feedback->current_pose = move_group.getCurrentPose().pose;
            goal_handle->publish_feedback(feedback);
            
            // Plan the trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool planning_success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (!planning_success) {
                result->success = false;
                result->error_message = "Planning failed";
                result->final_pose = move_group.getCurrentPose().pose;
                goal_handle->abort(result);
                return;
            }
            
            // Update feedback - execution phase
            feedback->status = "Executing trajectory...";
            feedback->progress_percentage = 50.0;
            goal_handle->publish_feedback(feedback);
            
            if (goal->execute_immediately) {
                // Execute with live feedback
                auto execute_future = std::async(std::launch::async, [&move_group]() {
                    return move_group.move();
                });
                
                // Provide live feedback during execution
                while (execute_future.wait_for(feedback_period) == std::future_status::timeout) {
                    if (goal_handle->is_canceling()) {
                        move_group.stop();
                        result->success = false;
                        result->error_message = "Execution canceled";
                        result->final_pose = move_group.getCurrentPose().pose;
                        goal_handle->canceled(result);
                        return;
                    }
                    
                    // Update feedback with current pose
                    feedback->current_pose = move_group.getCurrentPose().pose;
                    feedback->status = goal->use_named_pose ? 
                        ("Moving to '" + goal->named_pose + "'...") : 
                        "Moving to target...";
                    feedback->progress_percentage = 75.0;
                    goal_handle->publish_feedback(feedback);
                }
                
                // Get execution result
                auto execution_result = execute_future.get();
                bool execution_success = (execution_result == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (!execution_success) {
                    result->success = false;
                    result->error_message = "Execution failed";
                    result->final_pose = move_group.getCurrentPose().pose;
                    goal_handle->abort(result);
                    return;
                }
            }
            
            // Success!
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            result->success = true;
            result->error_message = "";
            result->final_pose = move_group.getCurrentPose().pose;
            result->execution_time = duration.count() / 1000.0;
            
            // Final feedback
            feedback->status = goal->use_named_pose ? 
                ("Reached '" + goal->named_pose + "' successfully!") : 
                "Trajectory completed successfully!";
            feedback->progress_percentage = 100.0;
            feedback->current_pose = result->final_pose;
            goal_handle->publish_feedback(feedback);
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "✅ Trajectory completed successfully in %.2f seconds", 
                        result->execution_time);
            
        } catch (const std::exception& e) {
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "❌ Exception during execution: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<TrajectoryActionServer>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(action_server->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}