#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2, pi
import tf_transformations


class PatrolFSM(Node):
    def __init__(self):
        super().__init__('patrol_fsm')

        self.state = 'Navigating'
        self.station_index = 0
        self.checkpoints = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
        self.dock_position = (0.0, 0.0)

        # Battery warning flags
        self.warn_50 = self.warn_15 = self.warn_5 = False

        # Robot state
        self.current_position = (0.0, 0.0)
        self.yaw = 0.0
        self.rotate_start_time = None

        # Obstacle detection flags
        self.obstacle_detected = False
        self.prev_obstacle_state = False

        # Timer setup
        self.patrol_timer = 120.0
        self.last_tick = time.time()

        # ROS interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.run_fsm)

        # Logging
        self.log_file = open('/tmp/fsm_log.csv', 'w')
        self.log_file.write('timestamp,old_state,new_state\n')
        self.get_logger().info('FSM started in Navigating state.')

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.prev_obstacle_state = self.obstacle_detected
        self.obstacle_detected = False

        for dist in msg.ranges:
            if 0.05 < dist < 0.5:
                self.obstacle_detected = True
                break

        if self.obstacle_detected and not self.prev_obstacle_state:
            red_text = '\033[91m Obstacle detected.\033[0m'
            self.get_logger().warn(red_text)

    def log_state_change(self, old, new):
        timestamp = datetime.now().isoformat()
        self.log_file.write(f'{timestamp},{old},{new}\n')
        self.log_file.flush()
        green_new = f'\033[92m{new}\033[0m'
        self.get_logger().info(f'FSM transition: {old} → {green_new}')

    def publish_velocity(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def angle_to_target(self, target):
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        return atan2(dy, dx)

    def distance_to(self, target):
        x, y = self.current_position
        tx, ty = target
        return sqrt((x - tx)**2 + (y - ty)**2)

    def angle_diff(self, a, b):
        d = a - b
        while d > pi:
            d -= 2 * pi
        while d < -pi:
            d += 2 * pi
        return d

    def run_fsm(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now

        # Battery warnings
        percent_remaining = (self.patrol_timer / 100.0) * 100

        if not self.warn_50 and percent_remaining <= 50:
            self.warn_50 = True
            self.get_logger().warn("Battery at 50% of patrol time remaining.")
        if not self.warn_15 and percent_remaining <= 15:
            self.warn_15 = True
            self.get_logger().warn("Battery at 15% of patrol time remaining!")
        if not self.warn_5 and percent_remaining <= 5:
            self.warn_5 = True
            self.get_logger().error("Battery critically low: 5% of patrol time remaining!")

        # Timer decrement logic
        if self.state in ['Navigating', 'Station-Inspect', 'Transmit Panorama']:
            self.patrol_timer -= dt
        elif self.state == 'Blocked':
            if not hasattr(self, 'last_blocked_log') or now - self.last_blocked_log > 1.0:
                self.last_blocked_log = now
                self.get_logger().info('Obstacle blocking path. Timer paused.')
                self.get_logger().info('Waiting for obstacle to clear...')

        # Battery depleted
        if self.patrol_timer <= 0 and self.state not in ['Return-Dock']:
            old = self.state
            self.state = 'Return-Dock'
            self.log_state_change(old, self.state)
            self.publish_velocity(0.0)
            return

        # Transition to Blocked state
        if self.obstacle_detected and self.state not in ['Blocked']:
            old = self.state
            self.state = 'Blocked'
            self.log_state_change(old, self.state)
            self.publish_velocity(0.0)
            return

        # FSM logic
        if self.state == 'Navigating':
            target = self.checkpoints[self.station_index]
            angle = self.angle_to_target(target)
            angle_error = self.angle_diff(angle, self.yaw)

            if abs(angle_error) > 0.1:
                self.publish_velocity(0.0, 0.3 if angle_error > 0 else -0.3)
            elif self.distance_to(target) > 0.3:
                self.publish_velocity(0.2, 0.0)
            else:
                self.publish_velocity(0.0)
                old = self.state
                self.state = 'Station-Inspect'
                self.rotate_start_time = time.time()
                self.log_state_change(old, self.state)

        elif self.state == 'Station-Inspect':
            self.publish_velocity(0.0, 0.6)
            if time.time() - self.rotate_start_time > 8.0:
                self.publish_velocity(0.0)
                old = self.state
                self.state = 'Return-Dock' if self.patrol_timer <= 0 else 'Transmit Panorama'
                self.log_state_change(old, self.state)

        elif self.state == 'Transmit Panorama':
            if not hasattr(self, 'transmit_start_time') or self.transmit_start_time is None:
                self.transmit_start_time = time.time()
                self.get_logger().info('Transmitting panorama...')
            elif time.time() - self.transmit_start_time >= 1.0:
                self.transmit_start_time = None
                self.station_index = (self.station_index + 1) % len(self.checkpoints)
                old = self.state
                self.state = 'Navigating'
                self.log_state_change(old, self.state)

        elif self.state == 'Blocked':
            if not self.obstacle_detected:
                old = self.state
                self.state = 'Navigating'
                self.log_state_change(old, self.state)

        elif self.state == 'Return-Dock':
            target = self.dock_position
            angle = self.angle_to_target(target)
            angle_error = self.angle_diff(angle, self.yaw)
            distance = self.distance_to(target)

            #self.get_logger().info(f"[ReturnDock] Distance: {distance:.2f}, Angle Error: {angle_error:.2f}")

            if abs(angle_error) > 0.1:
                self.publish_velocity(0.0, 0.3 if angle_error > 0 else -0.3)
            elif distance > 0.3:
                self.publish_velocity(0.2, 0.0)
            else:
                self.publish_velocity(0.0)
                self.get_logger().info('Docked. Resetting timer.')
                self.patrol_timer = 120.0
                self.warn_50 = self.warn_15 = self.warn_5 = False
                old = self.state
                self.state = 'Station-Inspect'
                self.rotate_start_time = time.time()
                self.log_state_change(old, self.state)

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
