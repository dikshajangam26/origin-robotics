#!/usr/bin/env python3
"""
Modified Webots Controller with ROS2 Integration
"""

from controller import Supervisor
import math
import numpy as np
import csv
import os
import subprocess
import threading  # For running ROS2 in background

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Your existing imports
# ... (keep all your original imports)

# ============================================================
# PART 0: ROS2 BRIDGE NODE
# ============================================================
class WebotsBridgeNode(Node):
    """
    ROS2 node that connects Webots to ROS2 ecosystem
    
    Subscribes to:
        - /trajectory (desired trajectory)
        - /cmd_vel (velocity commands)
    
    Publishes to:
        - /odom (robot odometry)
        - /waypoints (if needed)
    """
    
    def __init__(self, robot_controller):
        super().__init__('webots_bridge_node')
        
        self.robot = robot_controller  # Reference to Webots controller
        self.trajectory = []
        self.cmd_vel = [0.0, 0.0]  # [linear_v, angular_v]
        
        # Subscribers
        self.sub_trajectory = self.create_subscription(
            PoseArray,
            '/trajectory',
            self.trajectory_callback,
            10
        )
        
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.pub_odom = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.get_logger().info('Webots Bridge Node Started')
    
    def trajectory_callback(self, msg):
        """Receive trajectory from ROS2"""
        self.trajectory = []
        for pose in msg.poses:
            self.trajectory.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'timestamp': pose.position.z,
            })
        self.get_logger().info(f'Received trajectory: {len(self.trajectory)} points')
    
    def cmd_vel_callback(self, msg):
        """Receive velocity commands from ROS2"""
        self.cmd_vel = [msg.linear.x, msg.angular.z]
    
    def publish_odometry(self):
        """Publish robot state as odometry"""
        odom = Odometry()
        odom.header = Header(frame_id='odom')
        
        # Position
        odom.pose.pose.position.x = self.robot.current_x
        odom.pose.pose.position.y = self.robot.current_y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (from theta)
        theta = self.robot.current_theta
        odom.pose.pose.orientation.z = math.sin(theta / 2)
        odom.pose.pose.orientation.w = math.cos(theta / 2)
        
        self.pub_odom.publish(odom)


# ============================================================
# PART 1: YOUR ORIGINAL MAPPER CLASS (unchanged)
# ============================================================
class OccupancyGridMapper:
    """Your original mapper - KEEP UNCHANGED"""
    # ... (paste your original OccupancyGridMapper class here)
    pass


# ============================================================
# PART 2: MODIFIED ROBOT CONTROLLER
# ============================================================
class KinematicRobot(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.robot_node = self.getSelf()
        self.SPEED = 0.3
        
        # ... (keep all your original __init__ code)
        
        # NEW: ROS2 Bridge
        self.ros2_node = None
        self.use_ros2_trajectory = False
        self.target_trajectory = []
        self.target_idx = 0
    
    def init_ros2(self):
        # Inside __init__
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        
        # Set position to infinity to enable velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Set initial velocity to 0
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        """Initialize ROS2 bridge in separate thread"""
        def ros2_thread():
            rclpy.init()
            self.ros2_node = WebotsBridgeNode(self)
            try:
                rclpy.spin(self.ros2_node)
            except KeyboardInterrupt:
                pass
            finally:
                rclpy.shutdown()
             
        
        # Start ROS2 in background thread
        thread = threading.Thread(target=ros2_thread, daemon=True)
        thread.start()
        print("ROS2 Bridge started in background")
    
    def run(self):
        """Main control loop"""
        print("Robot Controller Starting .......")
        
        # Initialize ROS2
        self.init_ros2()

        # Inside the run() loop, before you use the node
        if self.ros2_node is None:
            continue # Wait for the bridge to finish starting
                
        self.imu_offset = None
        self.is_worker_detected = False
        
        # Force alignment at start
        self.current_theta = 0.0
        
        while self.step(self.time_step) != -1:
            dt = self.time_step / 1000.0
            self.timer += dt
            self.state_timer += dt
            self.step_count += 1
            # Grab the values from the bridge node
            linear_v = self.ros2_node.cmd_vel[0]  # Forward speed
            angular_v = self.ros2_node.cmd_vel[1] # Turning speed
            
            # --- 1. SENSOR FUSION (Your original code) ---
            imu_rpy = self.imu.getRollPitchYaw()
            raw_theta = imu_rpy[2]
            if self.imu_offset is None:
                self.imu_offset = raw_theta
            fused_theta = raw_theta - self.imu_offset
            
            # --- 2. VISION (Your original code) ---
            if self.step_count % 10 == 0:
                self.is_worker_detected = self.detect_worker()
            
            # ... (keep all your original sensor processing code)
            
            # --- NEW: TRAJECTORY FOLLOWING ---
            linear_v = 0
            angular_v = 0
            
            # Check if we have ROS2 trajectory
            if (self.ros2_node is not None and 
                hasattr(self.ros2_node, 'trajectory') and 
                len(self.ros2_node.trajectory) > 0):
                
                # Use ROS2 trajectory
                linear_v, angular_v = self.follow_ros2_trajectory()
                self.use_ros2_trajectory = True
            
            # Check for ROS2 velocity commands
            elif (self.ros2_node is not None and 
                  hasattr(self.ros2_node, 'cmd_vel')):
                
                linear_v = self.ros2_node.cmd_vel[0]
                angular_v = self.ros2_node.cmd_vel[1]
            
            # Fall back to your original logic
            else:
                # YOUR ORIGINAL STATE MACHINE HERE
                # (all your worker detection, emergency avoidance, etc)
                pass
            
            # --- UPDATE ROBOT STATE ---
            self.current_theta += angular_v * dt
            self.current_x += linear_v * math.cos(self.current_theta) * dt
            self.current_y += linear_v * math.sin(self.current_theta) * dt
            
            # --- PUBLISH ODOMETRY ---
            if self.ros2_node is not None:
                self.ros2_node.publish_odometry()
            
            # --- MAPPING AND VISUALIZATION (Your original code) ---
            self.mapper.update_map(...)
            self.trans_field.setSFVec3f([...])
            self.rot_field.setSFRotation([...])
            
            # --- CSV LOGGING (Your original code) ---
            # ... (keep your CSV logging)
    
    def follow_ros2_trajectory(self):
        """
        Follow trajectory from ROS2
        Pure Pursuit control
        """
        if len(self.ros2_node.trajectory) == 0:
            return 0.0, 0.0
        
        trajectory = self.ros2_node.trajectory
        
        # Find closest point
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(trajectory):
            dx = point['x'] - self.current_x
            dy = point['y'] - self.current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Lookahead point
        target_idx = min(closest_idx + 5, len(trajectory) - 1)
        target = trajectory[target_idx]
        
        # Pure Pursuit
        dx = target['x'] - self.current_x
        dy = target['y'] - self.current_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        desired_theta = math.atan2(dy, dx)
        heading_error = desired_theta - self.current_theta
        
        # Normalize angle
        while heading_error > math.pi:
            heading_error -= 2*math.pi
        while heading_error < -math.pi:
            heading_error += 2*math.pi
        
        # Control commands
        linear_v = 0.3 * min(1.0, distance_to_target)
        angular_v = 0.5 * heading_error
        
        return linear_v, angular_v
    
    def detect_worker(self):
        """Your original detect_worker method"""
        # ... (keep your original code)
        pass


# --- EXECUTION ---
if __name__ == '__main__':
    controller = KinematicRobot()
    controller.run()
