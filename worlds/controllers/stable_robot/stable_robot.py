#!/usr/bin/env python3
# Copyright 1996-2023 Cyberbotics Ltd.
# OPTIMIZED Controller: Stable Robot + ROS2 Bridge + Yellow Detection

from controller import Supervisor
import math
import numpy as np
import csv
import os 
import subprocess
import threading
import time
import cv2  # For HSV color detection

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseArray, Pose, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("WARNING: ROS2 not available. Running without ROS2 bridge.")

# ============================================================
# PART 0: ROS2 BRIDGE NODE
# ============================================================
if ROS2_AVAILABLE:
    class WebotsBridgeNode(Node):
        """Bridges Webots to ROS2"""
        
        def __init__(self, robot_controller):
            super().__init__('webots_bridge_node')
            
            self.robot = robot_controller
            self.trajectory = []
            self.cmd_vel = [0.0, 0.0]
            
            # Subscriber for trajectory
            self.sub_trajectory = self.create_subscription(
                PoseArray,
                '/trajectory',
                self.trajectory_callback,
                10
            )
            
            # Subscriber for cmd_vel
            self.sub_cmd = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
            
            # Publisher for odometry
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
            """Publish robot state"""
            odom = Odometry()
            odom.header = Header(frame_id='odom')
            
            odom.pose.pose.position.x = self.robot.current_x
            odom.pose.pose.position.y = self.robot.current_y
            odom.pose.pose.position.z = 0.0
            
            theta = self.robot.current_theta
            odom.pose.pose.orientation.z = math.sin(theta / 2)
            odom.pose.pose.orientation.w = math.cos(theta / 2)
            
            self.pub_odom.publish(odom)

# ============================================================
# PART 1: OPTIMIZED MAPPER
# ============================================================
class OccupancyGridMapper:
    def __init__(self, map_size_m, resolution_m, max_range_m):
        self.RESOLUTION = resolution_m
        self.SIZE_M = map_size_m
        self.SIZE_PIXELS = int(self.SIZE_M / self.RESOLUTION)
        self.CENTER_PIXEL = self.SIZE_PIXELS // 2
        self.MAX_RANGE = max_range_m 
        
        self.grid = np.full((self.SIZE_PIXELS, self.SIZE_PIXELS), 0.5, dtype=np.float32)

    def world_to_map(self, x_w, y_w):
        x_p = int(x_w / self.RESOLUTION)
        y_p = int(y_w / self.RESOLUTION)
        map_x = x_p + self.CENTER_PIXEL
        map_y = self.SIZE_PIXELS - (y_p + self.CENTER_PIXEL)
        
        if 0 <= map_x < self.SIZE_PIXELS and 0 <= map_y < self.SIZE_PIXELS:
            return map_x, map_y
        return None, None

    def log_odds_update(self, map_x, map_y, is_occupied):
        if map_x is None: return
        current_p = self.grid[map_y, map_x]
        
        if is_occupied: 
            self.grid[map_y, map_x] = min(1.0, current_p + 0.3) 
        else:           
            if current_p > 0.9: return 
            self.grid[map_y, map_x] = max(0.0, current_p - 0.05)

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return points

    def update_map(self, x_robot, y_robot, theta_robot, lidar_data):
        num_rays = len(lidar_data)
        if num_rays == 0: return
        
        step = 5  # OPTIMIZATION: Skip more rays (5 instead of 3)
        lidar_angle_min = -math.pi / 2 
        angle_increment = math.pi / num_rays 
        
        rob_x, rob_y = self.world_to_map(x_robot, y_robot)

        for i in range(0, num_rays, step):
            distance = lidar_data[i]
            if distance > self.MAX_RANGE: 
                real_distance = self.MAX_RANGE; is_hit = False
            else:
                real_distance = distance; is_hit = True

            ray_angle = theta_robot + (lidar_angle_min + i * angle_increment)
            
            x_wall = x_robot + real_distance * math.cos(ray_angle)
            y_wall = y_robot + real_distance * math.sin(ray_angle)
            wall_x, wall_y = self.world_to_map(x_wall, y_wall)
            
            free_dist = min(real_distance, 4.0) 
            x_free = x_robot + free_dist * math.cos(ray_angle)
            y_free = y_robot + free_dist * math.sin(ray_angle)
            free_end_x, free_end_y = self.world_to_map(x_free, y_free)

            if rob_x is not None and free_end_x is not None:
                line_points = self.bresenham_line(rob_x, rob_y, free_end_x, free_end_y)
                for (px, py) in line_points:
                    self.log_odds_update(px, py, is_occupied=False)

            if is_hit and wall_x is not None:
                self.log_odds_update(wall_x, wall_y, is_occupied=True)

    def visualize_map(self, display_device):
        output = np.full((self.SIZE_PIXELS, self.SIZE_PIXELS, 4), 128, dtype=np.uint8)
        output[:, :, 3] = 255 
        output[self.grid > 0.65, 0:3] = 0   
        output[self.grid < 0.35, 0:3] = 255
        ir = display_device.imageNew(output.tobytes(), display_device.RGBA, self.SIZE_PIXELS, self.SIZE_PIXELS)
        display_device.imagePaste(ir, 0, 0, False)
        display_device.imageDelete(ir)

# ============================================================
# PART 2: ROBOT CONTROLLER (OPTIMIZED)
# ============================================================
class KinematicRobot(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.robot_node = self.getSelf()
        self.SPEED = 0.3 
        
        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")
        
        start_pos = self.trans_field.getSFVec3f()
        start_rot = self.rot_field.getSFRotation()
        self.current_x = start_pos[0]
        self.current_y = start_pos[1]
        self.current_theta = start_rot[3] 
        self.LOCKED_Z = 0.11 
        
        self.lidar = self.getDevice("lidar")
        self.lidar.enable(self.time_step)
        self.lidar.enablePointCloud()
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        
        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(self.time_step)
        
        self.map_display = self.getDevice("map_display") 
        
        self.mapper = OccupancyGridMapper(
            map_size_m=50.0, 
            resolution_m=0.1, 
            max_range_m=self.lidar.getMaxRange()
        )

        self.state = "FORWARD"
        self.timer = 0
        self.state_timer = 0 
        self.turn_direction = 1
        self.step_count = 0
        
        # Cache for worker detection (avoid calling twice)
        self.cached_worker_level = 0
        self.last_worker_detection_step = -1

        # CSV LOGGING SETUP
        self.csv_writer = None
        self.csv_file = None

        try:
            print(f"CSV Logging to: {os.getcwd()}/robot_data.csv")
            self.csv_file = open('robot_data.csv', 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            
            self.csv_writer.writerow([
                'Time_s', 'Pos_X', 'Pos_Y', 'Theta_rad', 
                'State', 'Worker_Level',
                'Lidar_Front', 'Lidar_Left', 'Lidar_Right'
            ])

            self.csv_file.flush()
            print("CSV Header Written Successfully.")
            
        except PermissionError:
             print("ERROR: 'robot_data.csv' is open in Excel! Please close it.")
        except Exception as e:
            print(f"CSV Setup Error: {e}")

        # ROS2 INITIALIZATION
        self.ros2_node = None
        self.ros2_available = ROS2_AVAILABLE
        
        if self.ros2_available:
            self.init_ros2()

    def init_ros2(self):
        """Initialize ROS2 bridge in separate thread"""
        def ros2_thread():
            try:
                rclpy.init()
                self.ros2_node = WebotsBridgeNode(self)
                rclpy.spin(self.ros2_node)
            except Exception as e:
                print(f"ROS2 Error: {e}")
            finally:
                try:
                    rclpy.shutdown()
                except:
                    pass
        
        thread = threading.Thread(target=ros2_thread, daemon=True)
        thread.start()
        print("ROS2 Bridge started in background")
        time.sleep(1)

    def detect_worker(self):
        """
        OPTIMIZED: Advanced yellow clothing detection
        Only processes image when called, caches result
        """
        
        image = self.camera.getImage()
        if image is None: 
            return 0
        
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        img_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        
        # Convert BGRA to HSV
        bgr_array = img_array[:, :, [2, 1, 0]]
        hsv_array = cv2.cvtColor(bgr_array.astype(np.uint8), cv2.COLOR_BGR2HSV)
        
        # Yellow color range in HSV
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        
        yellow_mask = cv2.inRange(hsv_array, lower_yellow, upper_yellow)
        
        # Find contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return 0
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < 500:  # Min area threshold
            return 0
        
        x, y, w, h = cv2.boundingRect(largest_contour)
        roi = yellow_mask[y:y+h, x:x+w]
        
        if roi.size == 0:
            return 0
        
        yellow_pixels = np.count_nonzero(roi)
        yellow_density = yellow_pixels / roi.size
        
        if yellow_density < 0.3:
            return 0
        
        # Estimate distance
        max_possible_area = width * height * 0.3
        normalized_area = min(area / max_possible_area, 1.0)
        
        if normalized_area > 0.15:
            detection_level = 3
        elif normalized_area > 0.08:
            detection_level = 2
        elif normalized_area > 0.03:
            detection_level = 1
        else:
            return 0
        
        # Check position
        center_y = (y + h // 2) / height
        if center_y < 0.2:
            return 0
        
        # Log only occasionally
        if self.step_count % 50 == 0:
            print(f"👤 WORKER DETECTED | Level: {detection_level} | Area: {area:.0f}px | Density: {yellow_density:.1%}")
        
        return detection_level

    def run(self):
        print("Robot Controller Starting .......")
        self.imu_offset = None  
        
        while self.step(self.time_step) != -1:
            dt = self.time_step / 1000.0
            self.timer += dt
            self.state_timer += dt 
            self.step_count += 1
            
            # --- 1. SENSOR FUSION (Every iteration) ---
            imu_rpy = self.imu.getRollPitchYaw()
            raw_theta = imu_rpy[2] 
            if self.imu_offset is None: 
                self.imu_offset = raw_theta
            fused_theta = raw_theta - self.imu_offset

            # --- 2. VISION (OPTIMIZED - Every 20 steps instead of 10) ---
            worker_detection_level = 0
            if self.step_count % 20 == 0:  # OPTIMIZATION: Every 20 steps
                worker_detection_level = self.detect_worker()
                self.cached_worker_level = worker_detection_level
                self.last_worker_detection_step = self.step_count
            else:
                # Use cached value between detections
                worker_detection_level = self.cached_worker_level

            # --- 3. LIDAR CLEANING (OPTIMIZED) ---
            raw_lidar_data = self.lidar.getRangeImage()
            cleaned_data = []
            
            for dist in raw_lidar_data:
                if dist == float('inf') or dist < 0.05 or dist > 15.0:
                    cleaned_data.append(20.0)
                else:
                    cleaned_data.append(dist)
            
            if len(cleaned_data) > 0:
                third = len(cleaned_data) // 3
                min_right = min(cleaned_data[0:third])
                min_front = min(cleaned_data[third:2*third])
                min_left = min(cleaned_data[2*third:len(cleaned_data)])
            else:
                min_right, min_front, min_left = 20.0, 20.0, 20.0
                
            # --- STATE MACHINE WITH YELLOW WORKER DETECTION ---
            linear_v = 0.0
            angular_v = 0.0

            if self.state == "FORWARD":
                if worker_detection_level >= 2:
                    print(f"⚠️ WORKER DETECTED AT LEVEL {worker_detection_level}")
                    self.state = "WORKER_CAUTION"
                    self.state_timer = 0
                elif min_front < 0.35 or min_left < 0.35 or min_right < 0.35:
                    self.state = "EMERGENCY"
                    self.state_timer = 0
                elif min_front < 0.4:
                    self.state = "U_TURN"
                    self.state_timer = 0
                else:
                    linear_v = self.SPEED
                    angular_v = 0.0

            elif self.state == "WORKER_CAUTION":
                if worker_detection_level == 3:
                    print("🚨 WORKER CRITICAL DISTANCE - EMERGENCY STOP!")
                    self.state = "WORKER_STOP"
                    self.state_timer = 0
                elif worker_detection_level >= 2:
                    linear_v = self.SPEED * 0.3
                    angular_v = 0.0
                    
                    if self.state_timer > 3.0 and worker_detection_level < 2:
                        print("✅ Worker moved away, resuming...")
                        self.state = "FORWARD"
                else:
                    print("✓ Worker out of detection range")
                    self.state = "FORWARD"

            elif self.state == "WORKER_STOP":
                linear_v = 0.0
                angular_v = 0.0
                
                if self.state_timer > 5.0:
                    if worker_detection_level <= 1:
                        print("✅ Safe to resume - worker far away")
                        self.state = "FORWARD"
                    else:
                        self.state_timer = 0

            elif self.state == "EMERGENCY":
                linear_v = -0.15
                angular_v = 0.0
                
                if self.state_timer > 1.5 or min_front > 0.6:
                    self.state = "U_TURN"
                    self.state_timer = 0

            elif self.state == "U_TURN":
                linear_v = 0.0
                angular_v = 1.5 if min_left < min_right else -1.5
                
                if self.state_timer > 1.05:
                    self.state = "FORWARD"
            
            # --- UPDATE POSITION ---
            self.current_theta += angular_v * dt
            if self.current_theta > math.pi: 
                self.current_theta -= 2*math.pi
            if self.current_theta < -math.pi: 
                self.current_theta += 2*math.pi
            
            self.current_x += linear_v * math.cos(self.current_theta) * dt
            self.current_y += linear_v * math.sin(self.current_theta) * dt
    
            # --- MAPPING (OPTIMIZED - Every 50 steps) ---
            if self.step_count % 50 == 0: 
                self.mapper.update_map(self.current_x, self.current_y, self.current_theta, raw_lidar_data)
                self.mapper.visualize_map(self.map_display)
    
            # --- UPDATE WEBOTS ---
            self.trans_field.setSFVec3f([self.current_x, self.current_y, self.LOCKED_Z])
            self.rot_field.setSFRotation([0, 0, 1, self.current_theta])
    
            # --- PUBLISH TO ROS2 (OPTIMIZED - Every 10 steps) ---
            if self.ros2_node is not None and self.step_count % 10 == 0:
                self.ros2_node.publish_odometry()
    
            # --- CSV LOGGING (OPTIMIZED - Every 30 steps) ---
            if self.step_count % 30 == 0 and self.csv_writer is not None:
                try:
                    self.csv_writer.writerow([
                        f"{self.timer:.2f}",
                        f"{self.current_x:.3f}",
                        f"{self.current_y:.3f}",
                        f"{self.current_theta:.3f}",
                        self.state,
                        worker_detection_level,
                        f"{min_front:.2f}",
                        f"{min_left:.2f}",
                        f"{min_right:.2f}"
                    ])
                    self.csv_file.flush()
                except Exception:
                    pass

# --- EXECUTION ---
if __name__ == '__main__':
    controller = KinematicRobot()
    controller.run()