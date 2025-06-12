import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

class FollowerListener(Node):
    def __init__(self):
        super().__init__('follower_cmd_listener')
        self.subscription = self.create_subscription(
            String, '/leader/command', self.listener_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)

        self.takeoff_cli = self.create_client(TelloAction, '/follower/tello_action')
        self.takeoff_cli_ready = False
        self.ensure_takeoff_service()

        self.moving = False
        self.stop_timer = None

    def ensure_takeoff_service(self):
        """Wait asynchronously for the takeoff service, then mark ready."""
        if not self.takeoff_cli.service_is_ready():
            self.get_logger().info('Waiting for /follower/tello_action service...')
            self.takeoff_cli.wait_for_service()
        self.takeoff_cli_ready = True
        self.get_logger().info('/follower/tello_action service is ready.')

    def listener_callback(self, msg):
        if msg.data == "START" and not self.moving:
            self.moving = True
            self.get_logger().info('Received START command')

            # 1) Take off, only if service is ready
            if not self.takeoff_cli_ready:
                self.get_logger().error('Takeoff service not available!')
            else:
                req = TelloAction.Request()
                req.cmd = 'takeoff'
                future = self.takeoff_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info(f'Takeoff response: {response}')
                if future.result() is None or not future.result().success:
                    self.get_logger().error(f'Takeoff failed: {future.result()}')
                else:
                    self.get_logger().info('Takeoff command succeeded.')

            # 2) Start moving forward
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)

            # 3) Schedule a stop timer
            self.stop_timer = self.create_timer(2.0, self.stop_movement)

    def stop_movement(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Follower: Stopped after 2 seconds.')

        self.moving = False
        if self.stop_timer:
            self.stop_timer.cancel()
            self.destroy_timer(self.stop_timer)
            self.stop_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = FollowerListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
