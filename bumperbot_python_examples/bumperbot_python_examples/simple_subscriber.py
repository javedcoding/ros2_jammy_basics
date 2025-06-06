import rclpy #ros2 library
from rclpy.node import Node
from std_msgs.msg import String #Library for Message type which will be recieved

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)

    def msgCallback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    SimpleSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()