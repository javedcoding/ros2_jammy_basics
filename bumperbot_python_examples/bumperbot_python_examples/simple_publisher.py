import rclpy #ros2 library
from rclpy.node import Node
from std_msgs.msg import String #Library for Message type which will be published

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        self.pub_ = self.create_publisher(String, "chatter", 10)

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
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) #this makes the thread continuously run the publisher class object
    simple_publisher.destroy_node() #this will destroy the publisher node smoothly when Ctrl+C pressed
    rclpy.shutdown()


if __name__ == '__main__':
    main()