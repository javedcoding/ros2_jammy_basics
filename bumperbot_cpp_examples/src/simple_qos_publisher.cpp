#include <rclcpp/rclcpp.hpp> //Include the ros C++ library
#include <std_msgs/msg/string.hpp> //Include the standard message type string

#include <chrono>

using namespace std::chrono_literals;

class SimpleQoSPublisher : public rclcpp::Node
{
public:
    SimpleQoSPublisher() : Node("simple_qos_publisher"), qos_profile_pub_(10), counter_(0)
    {
        declare_parameter<std::string>("reliability", "system_default"); //initiation of parameter delcaration
        declare_parameter<std::string>("durability", "system_default");

        const auto reliability = get_parameter("reliability").as_string(); //variable setting of parameter
        const auto durability = get_parameter("durability").as_string();

        //setting up reliability as per defined
        if(reliability == "best_effort"){
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", reliability.c_str());
        }
        else if(reliability == "reliable"){
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", reliability.c_str());
        }
        else if(reliability == "system_default"){
            qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", reliability.c_str());
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Selected Reliability QoS: " << reliability << " doesn't exists!");
        }

        //setting up durability as per defined
        if(durability == "volatile"){
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", durability.c_str());
        }
        else if(durability == "transient_local"){
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", durability.c_str());
        }
        else if(durability== "system_default"){
            qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
            RCLCPP_INFO(get_logger(), "[Reliability] : %s", durability.c_str());
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Selected Reliability QoS: " << durability << " doesn't exists!");
        }


        pub_ = create_publisher<std_msgs::msg::String>("chatter", qos_profile_pub_);
        timer_ = create_wall_timer(1s, std::bind(&SimpleQoSPublisher::timerCallback, this));

        RCLCPP_INFO(get_logger(), "publishing at 1 Hz");
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_profile_pub_;

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++);

        pub_->publish(message);
    }
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleQoSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}