#include <thread>
#include <rclcpp/rclcpp.hpp> //Include the ros C++ library
#include <rclcpp_action/rclcpp_action.hpp> //Include the ROS C++ action library
#include <rclcpp_components/register_node_macro.hpp> //Include the ROS C++ components register node macro
#include <bumperbot_msgs/action/fibonacci.hpp> //Include the action message type


namespace bumperbot_cpp_examples
{
class SimpleActionServer : public rclcpp::Node
{
public:
    SimpleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("simple_action_server", options)
    {
        // Action server initialization code would go here
        action_server_ = rclcpp_action::create_server<bumperbot_msgs::action::Fibonacci>(
            this,
            "fibonacci",
            std::bind(&SimpleActionServer::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SimpleActionServer::cancelCallback, this, std::placeholders::_1),
            std::bind(&SimpleActionServer::acceptedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(get_logger(), "Simple Action Server initialized");
    }
private:
    rclcpp_action::Server<bumperbot_msgs::action::Fibonacci>::SharedPtr action_server_; // Replace YourActionType with the actual action type
    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal request with order: %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal and start executing it
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Goal accepted, starting execution of thread");
        // Here you would start the action execution logic in a different thread
        std::thread{std::bind(&SimpleActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle
    )
    {
        RCLCPP_INFO(get_logger(), "Executing Goal");
        rclcpp::Rate loop_rate(1); // 1 Hz
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<bumperbot_msgs::action::Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<bumperbot_msgs::action::Fibonacci::Result>();

        for (int i = 0; (i < goal->order) && rclcpp::ok; ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }
            sequence.push_back(sequence[i] + sequence[i-1]);
            feedback->partial_sequence = sequence;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(get_logger(), "Publishing feedback");
            loop_rate.sleep();
        }
        if (rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Recieved request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT; // Accept the cancel request
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(bumperbot_cpp_examples::SimpleActionServer) // Register the node with the component manager