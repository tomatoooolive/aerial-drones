# leader_flight_node.py - ROS 2 drone controller for Gazebo simulation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
import math
import random

class LeaderFlightNode(Node):
    def __init__(self):
        super().__init__('leader_flight_node')

        # ROS interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Publishes velocity
        self.odom_sub = self.create_subscription(Odometry, 'ground_truth', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)  # Lidar input (optional)
        self.land_cli = self.create_client(Empty, 'tello/land')  # Landing service

        # Drone state
        self.position = None
        self.obstacle_detected = False
        self.avoidance_direction = 1
        self.current_index = 0
        self.landing_triggered = False

        # Waypoints: each is (x, y, z)
        self.waypoints = [
            (0.0, 0.0, 10.0),
            (30.0, 0.0, 12.0),
            (30.0, 30.0, 15.0),
            (0.0, 30.0, 10.0),
            (0.0, 0.0, 5.0),  # Final landing point
        ]

        self.timer = self.create_timer(0.1, self.navigate)  # Run control loop at 10Hz
        self.get_logger().info('LeaderFlightNode initialized and ready.')

    def odom_callback(self, msg):
        """Update current position from ground_truth topic."""
        self.position = msg.pose.pose.position

    def scan_callback(self, msg):
        """Detect obstacles from Lidar scan."""
        center = msg.ranges[len(msg.ranges)//3: 2*len(msg.ranges)//3]
        self.obstacle_detected = any(r < 3.0 for r in center if r > 0.01)
        if self.obstacle_detected:
            self.avoidance_direction = random.choice([-1, 1])

    def navigate(self):
        """Main control loop for navigation."""
        if self.position is None or self.current_index >= len(self.waypoints):
            return

        # Calculate distance to current waypoint
        tx, ty, tz = self.waypoints[self.current_index]
        dx, dy, dz = tx - self.position.x, ty - self.position.y, tz - self.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # If close enough, move to next waypoint
        if dist < 1.5:
            self.get_logger().info(f'Reached waypoint {self.current_index + 1}')
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.land()
            return

        # Set movement command
        twist = Twist()
        if self.obstacle_detected:
            twist.linear.y = self.avoidance_direction * 2.0
            twist.linear.z = 1.0
            self.get_logger().warn('Obstacle detected — performing sidestep')
        else:
            # Move toward waypoint
            vx, vy, vz = dx / dist, dy / dist, dz / dist
            twist.linear.x = vx * 3.0
            twist.linear.y = vy * 3.0
            twist.linear.z = vz * 3.0

        self.cmd_vel_pub.publish(twist)

    def land(self):
        """Stop movement and attempt landing."""
        if self.landing_triggered:
            return
        self.get_logger().info('Landing sequence started.')
        self.cmd_vel_pub.publish(Twist())
        self.landing_triggered = True

        if self.land_cli.service_is_ready():
            self.land_cli.call_async(Empty.Request())
        else:
            self.get_logger().warn('Landing service unavailable, drone will hover.')

def main(args=None):
    rclpy.init(args=args)
    node = LeaderFlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

