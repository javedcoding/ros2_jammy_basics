#include <rclcpp/rclcpp.hpp> //Include the ros C++ library
#include <rclcpp_action/rclcpp_action.hpp> //Include the ROS C++ action library
#include <rclcpp_components/register_node_macro.hpp> //Include the ROS C++ components register node macro
#include <bumperbot_msgs/action/fibonacci.hpp> //Include the action message type


namespace bumperbot_cpp_examples
{
class SimpleActionClient : public rclcpp::Node
{
public:
    explicit SimpleActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("simple_action_client", options)
    {
        // Action Client initialization code would go here
        action_client_ = rclcpp_action::create_client<bumperbot_msgs::action::Fibonacci>(this, "fibonacci");
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SimpleActionClient::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Simple Action Client initialized");
    }
private:
    rclcpp_action::Client<bumperbot_msgs::action::Fibonacci>::SharedPtr action_client_; // Replace YourActionType with the actual action type
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback()
    {
        timer_->cancel(); // Cancel the timer to prevent it from running again
        if (!action_client_->wait_for_action_server()) {
            RCLCPP_WARN(get_logger(), "Action server not available after waiting");
            return;
        }
        
        auto goal_msg = bumperbot_msgs::action::Fibonacci::Goal();
        goal_msg.order = 10; // Set the order for the Fibonacci sequence
        RCLCPP_INFO(get_logger(), "Sending Goal");

        auto send_goal_options = rclcpp_action::Client<bumperbot_msgs::action::Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalCallback(const rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server");
            return;
        }
        RCLCPP_INFO(get_logger(), "Goal accepted by the action server");
    }

    void feedbackCallback(
        const rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::SharedPtr,
        const std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in Fibonacci sequence: ";
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(get_logger(), ss.str().c_str());
    }
    
    void resultCallback(const rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::WrappedResult result)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            break;
            
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal aborted");
            return;
            
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal canceled");
            return;
            
            default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(get_logger(), ss.str().c_str());
        rclcpp::shutdown(); // Shutdown the node after receiving the result
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(bumperbot_cpp_examples::SimpleActionClient) // Register the node with the component manager