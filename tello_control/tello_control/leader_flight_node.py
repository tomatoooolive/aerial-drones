import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LeaderFlightNode(Node):
    def __init__(self):
        super().__init__('leader_flight_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.step = 0
        self.get_logger().info('Leader flight node started.')
        self.timer = self.create_timer(0.5, self.fly_sequence)

    def fly_sequence(self):
        msg = Twist()
        if self.step < 10:
            msg.linear.z = 2.0  # Ascend
            self.get_logger().info(f'Step {self.step}: ascending')
        elif self.step < 60:
            msg.linear.x = 5.0  # Move forward
            self.get_logger().info(f'Step {self.step}: flying forward')
        else:
            msg.linear.x = 0.0
            msg.linear.z = 0.0
            self.get_logger().info(f'Step {self.step}: hovering')

        self.cmd_vel_pub.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = LeaderFlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

