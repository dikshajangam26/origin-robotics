#!/usr/bin/env python3
"""
Trajectory Controller Node
Makes robot follow trajectory using Pure Pursuit control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import math


class TrajectoryControllerNode(Node):
    """
    Subscribes to:
        /trajectory - desired trajectory
        /odom - robot odometry (position, velocity)
    Publishes to:
        /cmd_vel - velocity commands
    """
    
    def __init__(self):
        super().__init__('trajectory_controller_node')
        
        # Parameters
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.trajectory = []
        self.current_target_idx = 0
        
        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.sub_trajectory = self.create_subscription(
            PoseArray,
            '/trajectory',
            self.trajectory_callback,
            10
        )
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Timer for control loop (100 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('Trajectory Controller Node Started')
    
    def trajectory_callback(self, trajectory_msg):
        """Receive new trajectory"""
        self.trajectory = []
        for pose in trajectory_msg.poses:
            self.trajectory.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,  # timestamp
                'theta': 2 * math.atan2(pose.orientation.z, pose.orientation.w)
            })
        
        self.current_target_idx = 0
        self.get_logger().info(f'Received trajectory with {len(self.trajectory)} points')
    
    def odom_callback(self, odom_msg):
        """Update robot state from odometry"""
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y
        
        # Extract theta from quaternion
        q = odom_msg.pose.pose.orientation
        self.robot_theta = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def pure_pursuit_control(self):
        """
        Pure Pursuit control law
        Finds lookahead point and steers toward it
        """
        if len(self.trajectory) == 0:
            return 0.0, 0.0
        
        # Find closest point on trajectory
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(self.trajectory):
            dx = point['x'] - self.robot_x
            dy = point['y'] - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Lookahead point: point at distance lookahead_distance ahead
        target_idx = min(closest_idx + 5, len(self.trajectory) - 1)  # Look 5 points ahead
        target_point = self.trajectory[target_idx]
        
        # Calculate desired heading to lookahead point
        dx = target_point['x'] - self.robot_x
        dy = target_point['y'] - self.robot_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        desired_theta = math.atan2(dy, dx)
        
        # Heading error
        heading_error = self.normalize_angle(desired_theta - self.robot_theta)
        
        # Control commands
        if distance_to_target < 0.1:  # Close to goal
            linear_v = 0.0
        else:
            linear_v = self.max_linear * min(1.0, distance_to_target)
        
        angular_v = self.kp_angular * heading_error
        
        # Saturate commands
        angular_v = max(-self.max_angular, min(self.max_angular, angular_v))
        
        return linear_v, angular_v
    
    def control_loop(self):
        """Main control loop"""
        linear_v, angular_v = self.pure_pursuit_control()
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_v
        cmd.angular.z = angular_v
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()