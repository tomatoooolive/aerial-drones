import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LeaderCommandPublisher(Node):
    def __init__(self):
        super().__init__('leader_cmd_publisher')
        self.publisher_ = self.create_publisher(String, '/leader/command', 10)
        self.timer = self.create_timer(3.0, self.send_command)

    def send_command(self):
        msg = String()
        msg.data = "START"
        self.publisher_.publish(msg)
        self.get_logger().info('Sent START command from leader.')

def main(args=None):
    rclpy.init(args=args)
    node = LeaderCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
