#!/usr/bin/env python3
"""
Trajectory Generation Node
Converts smooth path to time-stamped trajectory with velocity profile
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import math


class TrajectoryPoint:
    """Data structure for trajectory point"""
    def __init__(self, x, y, timestamp, velocity, theta=0.0):
        self.x = x
        self.y = y
        self.timestamp = timestamp
        self.velocity = velocity
        self.theta = theta


class TrajectoryGeneratorNode(Node):
    """
    Subscribes to: /smooth_path (smooth path from path smoother)
    Publishes to: /trajectory (time-stamped trajectory)
    """
    
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # Parameters
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.2)
        
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        
        # Publishers
        self.pub_trajectory = self.create_publisher(
            PoseArray,
            '/trajectory',
            10
        )
        
        self.get_logger().info('Trajectory Generator Node Started')
        self.trajectory = []
    
    def calculate_path_distances(self, smooth_path):
        """Calculate cumulative distance along path"""
        distances = [0.0]
        
        for i in range(1, len(smooth_path)):
            dx = smooth_path[i][0] - smooth_path[i-1][0]
            dy = smooth_path[i][1] - smooth_path[i-1][1]
            distance = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + distance)
        
        return np.array(distances)
    
    def generate_trapezoidal_profile(self, total_distance):
        """
        Generate trapezoidal velocity profile
        
        Phases:
        1. Acceleration: 0 -> max_velocity
        2. Cruise: constant max_velocity
        3. Deceleration: max_velocity -> 0
        
        Returns: (accel_distance, cruise_distance, decel_distance, total_time)
        """
        # Distance needed to accelerate to max velocity
        accel_distance = (self.max_velocity ** 2) / (2 * self.max_acceleration)
        accel_time = self.max_velocity / self.max_acceleration
        
        # Deceleration is symmetric
        decel_distance = accel_distance
        decel_time = accel_time
        
        # Remaining distance at cruise velocity
        cruise_distance = total_distance - accel_distance - decel_distance
        
        if cruise_distance < 0:
            # Distance too short for full profile, peak at reduced velocity
            self.max_velocity = math.sqrt(total_distance * self.max_acceleration)
            accel_distance = (self.max_velocity ** 2) / (2 * self.max_acceleration)
            decel_distance = accel_distance
            cruise_distance = 0
            accel_time = self.max_velocity / self.max_acceleration
            decel_time = accel_time
        
        cruise_time = cruise_distance / self.max_velocity if self.max_velocity > 0 else 0
        total_time = accel_time + cruise_time + decel_time
        
        return accel_distance, cruise_distance, decel_distance, total_time
    
    def velocity_at_distance(self, distance, accel_d, cruise_d, decel_d, total_d):
        """Calculate desired velocity at given distance along path"""
        
        if distance <= accel_d:
            # Acceleration phase: v^2 = 2*a*s
            velocity = math.sqrt(2 * self.max_acceleration * distance)
        
        elif distance <= (accel_d + cruise_d):
            # Cruise phase
            velocity = self.max_velocity
        
        else:
            # Deceleration phase
            remaining = total_d - distance
            velocity = math.sqrt(2 * self.max_acceleration * remaining)
        
        return max(0.0, velocity)
    
    def time_at_distance(self, distance, accel_d, cruise_d, decel_d, total_d):
        """Calculate time to reach given distance along path"""
        
        accel_time = self.max_velocity / self.max_acceleration
        
        if distance <= accel_d:
            # t = sqrt(2*s/a)
            time = math.sqrt(2 * distance / self.max_acceleration)
        
        elif distance <= (accel_d + cruise_d):
            # Time in cruise phase
            cruise_time_to_point = (distance - accel_d) / self.max_velocity
            time = accel_time + cruise_time_to_point
        
        else:
            # Time in deceleration phase
            decel_time = self.max_velocity / self.max_acceleration
            cruise_time = cruise_d / self.max_velocity if self.max_velocity > 0 else 0
            
            remaining = total_d - distance
            time_in_decel = math.sqrt(2 * remaining / self.max_acceleration)
            
            time = accel_time + cruise_time + decel_time - time_in_decel
        
        return max(0.0, time)
    
    def generate_trajectory(self, smooth_path):
        """
        Input: Smooth path (list of [x, y] points)
        Output: Trajectory with timestamps and velocities
        """
        if len(smooth_path) < 2:
            return []
        
        smooth_path = np.array(smooth_path)
        distances = self.calculate_path_distances(smooth_path)
        total_distance = distances[-1]
        
        # Get velocity profile parameters
        accel_d, cruise_d, decel_d, total_time = self.generate_trapezoidal_profile(total_distance)
        
        self.get_logger().info(
            f'Profile: accel_dist={accel_d:.2f}m, cruise_dist={cruise_d:.2f}m, '
            f'decel_dist={decel_d:.2f}m, total_time={total_time:.2f}s'
        )
        
        trajectory = []
        
        # For each point in smooth path, calculate timestamp and velocity
        for i, point in enumerate(smooth_path):
            distance = distances[i]
            
            # Velocity at this distance
            velocity = self.velocity_at_distance(distance, accel_d, cruise_d, decel_d, total_distance)
            
            # Time to reach this distance
            timestamp = self.time_at_distance(distance, accel_d, cruise_d, decel_d, total_distance)
            
            # Heading angle (toward next point)
            if i < len(smooth_path) - 1:
                dx = smooth_path[i+1][0] - smooth_path[i][0]
                dy = smooth_path[i+1][1] - smooth_path[i][1]
                theta = math.atan2(dy, dx)
            else:
                theta = 0.0
            
            traj_point = TrajectoryPoint(
                x=float(point[0]),
                y=float(point[1]),
                timestamp=timestamp,
                velocity=velocity,
                theta=theta
            )
            trajectory.append(traj_point)
        
        return trajectory
    
    def trajectory_to_pose_array(self, trajectory):
        """Convert trajectory to ROS2 PoseArray for visualization"""
        pose_array = PoseArray()
        pose_array.header = Header(frame_id='world')
        
        for traj_pt in trajectory:
            pose = Pose()
            pose.position.x = traj_pt.x
            pose.position.y = traj_pt.y
            pose.position.z = traj_pt.timestamp  # Use Z for timestamp (for visualization)
            
            # Orientation from heading angle
            from geometry_msgs.msg import Quaternion
            q = Quaternion()
            q.z = math.sin(traj_pt.theta / 2)
            q.w = math.cos(traj_pt.theta / 2)
            pose.orientation = q
            
            pose_array.poses.append(pose)
        
        return pose_array
    
    def smooth_path_callback(self, smooth_path):
        """Process smooth path and generate trajectory"""
        try:
            # Extract path coordinates
            path_list = []
            for pose in smooth_path.poses:
                path_list.append([pose.position.x, pose.position.y])
            
            # Generate trajectory
            self.trajectory = self.generate_trajectory(path_list)
            
            # Publish trajectory
            pose_array = self.trajectory_to_pose_array(self.trajectory)
            self.pub_trajectory.publish(pose_array)
            
            self.get_logger().info(
                f'Generated trajectory: {len(self.trajectory)} points, '
                f'duration: {self.trajectory[-1].timestamp:.2f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error generating trajectory: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    
    # TEST: Create sample trajectory
    smooth_path = [
        [0.0, 0.0],
        [2.0, 3.0],
        [5.0, 2.0],
        [8.0, 4.0],
        [10.0, 1.0]
    ]
    
    trajectory = node.generate_trajectory(smooth_path)
    
    print("\n=== TRAJECTORY GENERATION TEST ===")
    print(f"Path points: {len(smooth_path)}")
    print(f"Trajectory points: {len(trajectory)}")
    print("\nTrajectory Sample:")
    print("Time(s)\t X\t Y\t Velocity")
    for i in range(0, min(len(trajectory), 10), max(1, len(trajectory)//5)):
        tp = trajectory[i]
        print(f"{tp.timestamp:.2f}\t {tp.x:.2f}\t {tp.y:.2f}\t {tp.velocity:.3f}")
    
    # Plot velocity profile
    import matplotlib.pyplot as plt
    times = [tp.timestamp for tp in trajectory]
    velocities = [tp.velocity for tp in trajectory]
    
    plt.figure(figsize=(10, 5))
    plt.plot(times, velocities, 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Trapezoidal Velocity Profile')
    plt.grid(True)
    plt.savefig('velocity_profile.png', dpi=150)
    print("\nVelocity profile saved to 'velocity_profile.png'")
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()