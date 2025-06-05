import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String #Library for Message type which will be published

class SimpleQoSPublisher(Node):
    def __init__(self):
        super().__init__("simple_qos_publisher")

        self.qos_profile_pub = QoSProfile(depth=10) #Make a quality of service profile

        self.declare_parameter("reliability", "system_default") #Here reliability parameter is set to system default
        self.declare_parameter("durability", "system_default") #Here durability parameter is set to system default

        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        durability = self.get_parameter("durability").get_parameter_value().string_value

        if reliability == "best_effort":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info(f"[Reliability] : {reliability}")
        elif reliability == "reliable":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info(f"[Reliability] : {reliability}")
        elif reliability == "system_default":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info(f"[Reliability] : {reliability}")
        else:
            self.get_logger().error(f"Selected Reliability QoS \"{reliability}\" doesn't exist")
            return
        
        if durability == "volatile":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info(f"[Durability] : {durability}")
        elif durability == "transient_local":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info(f"[Durability] : {durability}")
        elif durability == "system_default":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info(f"[Durability] : {durability}")
        else:
            self.get_logger().error(f"Selected Durability QoS \"{durability}\" doesn't exist")
            return

        


        self.pub_ = self.create_publisher(String, "chatter", self.qos_profile_pub)

        self.counter_ = 0
        self.frequency_ = 1.0

        self.get_logger().info(f"Publishing at {self.frequency_} Hz")
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback) #function which sets the frequency of the publisher or subscriber messages

    def timerCallback(self):
        msg = String()
        msg.data = f"Hello ROS2 - counter: {self.counter_}" #this sets up the string message data to be published

        self.pub_.publish(msg) #this call will publish the message
        self.counter_ += 1

    
def main():
    rclpy.init()
    simple_qos_publisher = SimpleQoSPublisher()
    rclpy.spin(simple_qos_publisher) #this makes the thread continuously run the publisher class object
    simple_qos_publisher.destroy_node() #this will destroy the publisher node smoothly when Ctrl+C pressed
    rclpy.shutdown()


if __name__ == '__main__':
    main()
