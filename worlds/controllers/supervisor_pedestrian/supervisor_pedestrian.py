from controller import Supervisor # Must import Supervisor to control other nodes
from math import atan2, pi         # Import math functions for rotation

print("--- STARTING PEDESTRIAN SUPERVISOR SCRIPT ---")

# --- 1. Initialization ---
supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())

# Get the Pedestrian node from the scene tree
# IMPORTANT: This must match the DEF name in your scene tree, which is "Human".
pedestrian_node = supervisor.getFromDevice("Human")

# --- 2. Movement Setup ---
if pedestrian_node is not None:
    
    # Define a simple path (x, y, z coordinates)
    # The current position is approx [0.0, 0.0, 1.27]
    waypoints = [
        [0.0, 0.0, 1.27],  # Start point
        [5.0, 0.0, 1.27],  # Walk forward +X 
        [5.0, 0.0, -3.0],  # Turn and walk -Z 
        [0.0, 0.0, -3.0]   # Return walk -X 
    ]
    
    waypoint_index = 0
    speed = 0.02 # Movement speed
    
    # --- 3. Main Loop: Movement and Rotation ---
    while supervisor.step(TIME_STEP) != -1:
        
        # Get the current position and target
        current_translation = pedestrian_node.getField('translation').getSFVec3f()
        target_waypoint = waypoints[waypoint_index]
        
        # Calculate the direction vector to the target
        direction_x = target_waypoint[0] - current_translation[0]
        direction_z = target_waypoint[2] - current_translation[2]
        
        # 2D distance calculation (ignoring y/height)
        distance = ((direction_x)**2 + (direction_z)**2)**0.5

        # Check if the waypoint is reached
        if distance < 0.1: 
            # Move to the next waypoint in the list (loops back to 0)
            waypoint_index = (waypoint_index + 1) % len(waypoints)
            continue # Start the next iteration to get the new target
        
        # --- Apply Movement and Rotation ---
        if distance > 0:
            # 1. Update Translation (Position)
            pedestrian_node.getField('translation').setSFVec3f([
                current_translation[0] + speed * direction_x / distance,
                current_translation[1], # Keep Y (height) constant
                current_translation[2] + speed * direction_z / distance
            ])
            
            # 2. Update Rotation (Orientation)
            # Calculate the angle (yaw) using atan2
            # The Webots Pedestrian model faces -Z (negative depth) when angle=0
            # We add pi/2 to align the calculated angle with the model's forward direction
            angle = pi/2.0 + atan2(direction_z, direction_x)
            
            # Set rotation using the Axis-Angle representation [Ax, Ay, Az, Angle]
            # Rotation is around the Y-axis: [0, 1, 0, angle]
            pedestrian_node.getField('rotation').setSFRotation([0, 1, 0, angle])