import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bumperbot_msgs.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        
        self.action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )
        self.action_client.wait_for_server() #this line waits for the action server to be available
        self.get_logger().info("Starting Action Server...")
        self.goal = Fibonacci.Goal()
        self.goal.order = 10  # Set the order for the Fibonacci sequence
        self.future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.resultCallback)

    def resultCallback(self, future):
        result = future.result().result
        if result is None:
            self.get_logger().error("Result is None")
            return
        self.get_logger().info(f"Result: {result.sequence}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msgs):
        self.get_logger().info(f"Received Feedback: {feedback_msgs.feedback.partial_sequence}")
        # You can process the feedback here if needed
    

def main():
    rclpy.init()
    simple_action_client = SimpleActionClient()
    rclpy.spin(simple_action_client)
    simple_action_client.destroy_node()
    rclpy.shutdown()    


if __name__ == '__main__':
    main()