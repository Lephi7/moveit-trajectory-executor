#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_trajectory_msgs/action/move_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>

class TrajectoryActionClient : public rclcpp::Node
{
public:
    using MoveToPose = moveit_trajectory_msgs::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        action_client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
    }

    void send_named_goal(const std::string& pose_name)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = MoveToPose::Goal();
        goal_msg.named_pose = pose_name;
        goal_msg.use_named_pose = true;
        goal_msg.planning_group = "manipulator";
        goal_msg.planning_timeout = 5.0;
        goal_msg.execute_immediately = true;

        send_goal(goal_msg, "named pose '" + pose_name + "'");
    }

    void send_custom_goal(double x, double y, double z)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = MoveToPose::Goal();
        goal_msg.target_pose.position.x = x;
        goal_msg.target_pose.position.y = y;
        goal_msg.target_pose.position.z = z;
        goal_msg.target_pose.orientation.w = 1.0;
        goal_msg.use_named_pose = false;
        goal_msg.planning_group = "manipulator";
        goal_msg.planning_timeout = 5.0;
        goal_msg.execute_immediately = true;

        send_goal(goal_msg, "custom pose [" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]");
    }

private:
    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;

    void send_goal(const MoveToPose::Goal& goal_msg, const std::string& description)
    {
        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&TrajectoryActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&TrajectoryActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&TrajectoryActionClient::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "🚀 Sending goal to %s...", description.c_str());
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleMoveToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleMoveToPose::SharedPtr,
        const std::shared_ptr<const MoveToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "📊 %s | Progress: %.1f%% | Pose: [%.3f, %.3f, %.3f]",
            feedback->status.c_str(),
            feedback->progress_percentage,
            feedback->current_pose.position.x,
            feedback->current_pose.position.y,
            feedback->current_pose.position.z);
    }

    void result_callback(const GoalHandleMoveToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "🎉 Goal succeeded! Execution time: %.2f seconds", 
                           result.result->execution_time);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "❌ Goal was aborted: %s", 
                            result.result->error_message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "🛑 Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "❓ Unknown result code");
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), "Final pose: [%.3f, %.3f, %.3f]",
                   result.result->final_pose.position.x,
                   result.result->final_pose.position.y,
                   result.result->final_pose.position.z);
        
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<TrajectoryActionClient>();
    
    // Example: Send robot to "home" position
    if (argc > 1) {
        std::string pose_name = argv[1];
        action_client->send_named_goal(pose_name);
    } else {
        // Default: go to home position
        action_client->send_named_goal("home");
    }
    
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}