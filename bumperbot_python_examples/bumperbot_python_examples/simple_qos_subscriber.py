import rclpy #ros2 library
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String #Library for Message type which will be recieved

class SimpleQoSSubscriber(Node):
    def __init__(self):
        super().__init__("simple_qos_subscriber")

        self.qos_profile_sub = QoSProfile(depth=10) #Make a quality of service profile

        self.declare_parameter("reliability", "system_default") #Here reliability parameter is set to system default
        self.declare_parameter("durability", "system_default") #Here durability parameter is set to system default

        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        durability = self.get_parameter("durability").get_parameter_value().string_value

        if reliability == "best_effort":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info(f"[Reliability] : {reliability}")
        elif reliability == "reliable":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info(f"[Reliability] : {reliability}")
        elif reliability == "system_default":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info(f"[Reliability] : {reliability}")
        else:
            self.get_logger().error(f"Selected Reliability QoS \"{reliability}\" doesn't exist")
            return
        
        if durability == "volatile":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info(f"[Durability] : {durability}")
        elif durability == "transient_local":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info(f"[Durability] : {durability}")
        elif durability == "system_default":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info(f"[Durability] : {durability}")
        else:
            self.get_logger().error(f"Selected Durability QoS \"{durability}\" doesn't exist")
            return

        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, self.qos_profile_sub)

    def msgCallback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    simple_qos_subscriber = SimpleQoSSubscriber()
    rclpy.spin(simple_qos_subscriber)
    SimpleQoSSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()