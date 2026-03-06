#!/usr/bin/env python3
"""
Path Smoothing Node - Converts discrete waypoints to smooth trajectory
Uses Catmull-Rom spline interpolation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import math


class PathSmoothingNode(Node):
    """
    Subscribes to: /waypoints (array of waypoints)
    Publishes to: /smooth_path (smooth continuous path)
    """
    
    def __init__(self):
        super().__init__('path_smoothing_node')
        
        self.declare_parameter('samples_per_segment', 50)
        self.samples = self.get_parameter('samples_per_segment').value
        
        # Publishers
        self.pub_smooth = self.create_publisher(
            PoseArray, 
            '/smooth_path', 
            10
        )
        
        self.get_logger().info('Path Smoothing Node Started')
    
    def catmull_rom_spline(self, p0, p1, p2, p3, t):
        """
        Catmull-Rom spline interpolation
        t: parameter in [0, 1]
        Returns: interpolated point
        """
        # Basis functions
        t2 = t * t
        t3 = t2 * t
        
        q = 0.5 * np.array([
            2.0 * p1,
            -p0 + p2,
            2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3,
            -p0 + 3.0 * p1 - 3.0 * p2 + p3
        ])
        
        return q[0] + q[1] * t + q[2] * t2 + q[3] * t3
    
    def smooth_path(self, waypoints):
        """
        Input: List of waypoints [[x0, y0], [x1, y1], ...]
        Output: Smooth path as list of points
        """
        if len(waypoints) < 2:
            return waypoints
        
        waypoints = np.array(waypoints)
        smooth_path = []
        
        # Handle each segment
        for i in range(len(waypoints) - 1):
            # Control points for Catmull-Rom
            p0 = waypoints[i - 1] if i > 0 else waypoints[i]
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            p3 = waypoints[i + 2] if i + 2 < len(waypoints) else waypoints[i + 1]
            
            # Interpolate between p1 and p2
            for j in range(self.samples):
                t = j / float(self.samples)
                point = self.catmull_rom_spline(p0, p1, p2, p3, t)
                smooth_path.append(point)
        
        # Add final point
        smooth_path.append(waypoints[-1])
        return np.array(smooth_path)
    
    def smooth_waypoints_callback(self, waypoints):
        """
        Process waypoints and publish smooth path
        """
        try:
            # Extract waypoint coordinates
            wp_list = []
            for pose in waypoints.poses:
                wp_list.append([pose.position.x, pose.position.y])
            
            # Generate smooth path
            smooth = self.smooth_path(wp_list)
            
            # Publish as PoseArray
            pose_array = PoseArray()
            pose_array.header = Header(frame_id='world')
            
            for point in smooth:
                pose = Pose()
                pose.position.x = float(point[0])
                pose.position.y = float(point[1])
                pose.position.z = 0.0
                pose_array.poses.append(pose)
            
            self.pub_smooth.publish(pose_array)
            self.get_logger().info(f'Published smooth path: {len(smooth)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoothingNode()
    
    # TEST: Create some waypoints for testing
    test_waypoints = [
        [0.0, 0.0],
        [2.0, 3.0],
        [5.0, 2.0],
        [8.0, 4.0],
        [10.0, 1.0]
    ]
    
    smooth = node.smooth_path(test_waypoints)
    
    print("\n=== PATH SMOOTHING TEST ===")
    print(f"Input waypoints: {len(test_waypoints)}")
    print(f"Output smooth path: {len(smooth)} points")
    print("\nFirst 5 smooth points:")
    for i, p in enumerate(smooth[:5]):
        print(f"  {i}: ({p[0]:.3f}, {p[1]:.3f})")
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()