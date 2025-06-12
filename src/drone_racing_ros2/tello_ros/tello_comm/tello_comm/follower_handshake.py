#!/usr/bin/env python3

import rclpy                                    # ROS 2 Python client library
from rclpy.node import Node                    # Base class for ROS 2 nodes
from std_msgs.msg import String, Float32       # Standard messages for commands & distances
from geometry_msgs.msg import Twist            # Twist for velocity commands
from tello_msgs.srv import TelloAction         # Tello takeoff/land service


class FollowerHandshake(Node):
    def __init__(self):
        super().__init__('follower_handshake')    # Initialize node

        # 1) Subscribe to the leader's takeoff command
        self.create_subscription(
            String,
            '/leader/follow_cmd',
            self.on_takeoff_cmd,
            10
        )

        # 2) Subscribe to the leader's move_distance
        self.create_subscription(
            Float32,
            '/leader/move_distance',
            self.on_move_distance,
            10
        )

        # 3) Publisher to send ack_takeoff back to the leader
        self.status_pub = self.create_publisher(
            String,
            '/follower/status',
            10
        )

        # 4) Publisher for this follower's own cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/follower/cmd_vel',
            10
        )

        # 5) Client for the follower's takeoff service
        self.takeoff_cli = self.create_client(
            TelloAction,
            '/follower/tello_action'
        )
        self.get_logger().info('Follower: waiting for /follower/tello_action...')
        self.takeoff_cli.wait_for_service()
        self.get_logger().info('/follower/tello_action ready')

        # Internal state
        self.took_off    = False
        self.move_dist   = None
        self.move_timer  = None
        self.move_count  = 0

    def on_takeoff_cmd(self, msg: String):
        # Only act on the first "takeoff"
        if msg.data != 'takeoff' or self.took_off:
            return
        self.took_off = True
        self.get_logger().info('Follower: TAKEOFF command received')

        # Acknowledge immediately
        self.get_logger().info('Follower → leader: ack_takeoff')
        self.status_pub.publish(String(data='ack_takeoff'))

        # Then call the takeoff service
        req = TelloAction.Request(cmd='takeoff')
        fut = self.takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        rc = fut.result().rc if fut.result() else None
        self.get_logger().info(f'Follower: takeoff returned rc={rc}')

    def on_move_distance(self, msg: Float32):
        # Only handle the first distance message

        self.get_logger().info(f'Follower: Recieved move command')

        if self.move_dist is not None:
            return
        self.move_dist = msg.data
        self.get_logger().info(f'Follower: move_distance={self.move_dist} received')

        # Prepare a timer to move at 0.5 m/s for dist/speed seconds at 10 Hz
        speed = 0.5
        total_steps = int(self.move_dist / speed * 10)  # number of 0.1 s steps
        self.move_count = 0

        # Store and start the non-blocking timer
        self.move_timer = self.create_timer(0.1, lambda: self.perform_move(total_steps))
        self.get_logger().info(f'Follower: scheduled {total_steps} move steps at 10 Hz')

    def perform_move(self, total_steps):
        # Publish one Twist per timer call
        if self.move_count < total_steps:
            twist = Twist(linear=Twist().linear.__class__(x=0., y=0.0, z=0.0))
            self.cmd_vel_pub.publish(twist)
            self.move_count += 1
            self.get_logger().info(f'Follower: forward step {self.move_count}/{total_steps}')
        else:
            # Stop motion and clean up timer
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Follower: reached target distance')

            self.move_timer.cancel()
            self.destroy_timer(self.move_timer)
            self.move_timer = None


def main(args=None):
    rclpy.init(args=args)                         # Initialize ROS 2
    node = FollowerHandshake()                     # Create node
    rclpy.spin(node)                               # Spin to process callbacks
    node.destroy_node()                            # Clean up
    rclpy.shutdown()                               # Shutdown ROS 2


if __name__ == '__main__':
    main()                                         # Entry point
