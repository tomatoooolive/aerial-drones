#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction


class LeaderHandshake(Node):
    def __init__(self):
        super().__init__('leader_handshake')

        # — Publishers —
        # 1) send "takeoff" to follower
        self.cmd_pub     = self.create_publisher(String,  '/leader/follow_cmd',   10)
        # 2) send numeric distance to follower
        self.dist_pub    = self.create_publisher(Float32, '/leader/move_distance',10)
        # 3) velocity commands for leader itself
        self.cmd_vel_pub = self.create_publisher(Twist,   '/leader/cmd_vel',      10)

        # — Subscriber —
        # listen for follower's ack_takeoff
        self.create_subscription(String, '/follower/status', self.on_follower_ack, 10)

        # — Service client —
        self.leader_cli = self.create_client(TelloAction, '/leader/tello_action')
        self.get_logger().info('Leader: waiting for /leader/tello_action...')
        self.leader_cli.wait_for_service()
        self.get_logger().info('/leader/tello_action is ready')

        # — State & timers —
        self.ack_received = False
        self.takeoff_timer = None
        self.move_timer    = None
        self.move_count    = 0

        # Start the loop that ensures follower hears "takeoff"
        # at 5 Hz until ack arrives.
        self.takeoff_timer = self.create_timer(0.2, self.publish_takeoff)

        # Also kick off leader's own takeoff right away
        self.call_own_takeoff()


    def publish_takeoff(self):
        if self.ack_received:
            # follower has acked, stop sending "takeoff"
            self.takeoff_timer.cancel()
            self.destroy_timer(self.takeoff_timer)
            self.takeoff_timer = None
            return

        self.get_logger().info('Leader → follower: takeoff')
        self.cmd_pub.publish(String(data='takeoff'))


    def call_own_takeoff(self):
        self.get_logger().info('Leader: calling own takeoff service')
        req = TelloAction.Request(cmd='takeoff')
        fut = self.leader_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        rc = fut.result().rc if fut.result() else None
        self.get_logger().info(f'Leader: takeoff returned rc={rc}')


    def on_follower_ack(self, msg: String):
        # Only handle the first ack
        if msg.data != 'ack_takeoff' or self.ack_received:
            return

        self.ack_received = True
        self.get_logger().info('Leader: ACK received from follower → scheduling 10 m move')

        # Start non-blocking forward move at 10 Hz
        self.move_count = 0
        self.move_timer = self.create_timer(0.1, self.perform_move)


    def perform_move(self):
        # Publish 20 steps @10 Hz → 2 s @0.5 m/s = 10 m
        if self.move_count < 20:
            twist = Twist()
            twist.linear.y = 0.5
            self.cmd_vel_pub.publish(twist)
            self.move_count += 1
            self.get_logger().info(f'Leader: forward step {self.move_count}/20')
        else:
            # stop
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Leader: reached +10 m')

            # cleanup
            self.move_timer.cancel()
            self.destroy_timer(self.move_timer)
            self.move_timer = None

            # tell follower
            self.get_logger().info('Leader → follower: move_distance=10.0')
            self.dist_pub.publish(Float32(data=10.0))


def main(args=None):
    rclpy.init(args=args)
    node = LeaderHandshake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
