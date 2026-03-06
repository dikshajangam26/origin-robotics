# Copyright 1996-2020 Cyberbotics Ltd.
# Licensed under the Apache License, Version 2.0
"""Pedestrian class container."""
from controller import Supervisor
from controller import Keyboard

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO with Keyboard."""

    def __init__(self):
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        # Height 1.38 lifts the robot slightly so it doesn't hit the floor
        self.ROOT_HEIGHT = 1.38 
        self.CYCLE_TO_DISTANCE_RATIO = 0.05
        self.speed = 1.5
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]
        ]
        Supervisor.__init__(self)
        self.key_board = Keyboard()
        self.key_board.enable(16)
        self.animation_timer = 0.0

    def run(self):
        self.time_step = int(self.getBasicTimeStep())
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        
        # Safety Check: Verify we are running on the correct PROTO
        for name in self.joint_names:
            field = self.root_node_ref.getField(name)
            if field is None:
                print(f"ERROR: Field '{name}' not found. Please use the standard Pedestrian PROTO (Do not convert to Base Node).")
                return # Stop execution to prevent crash
            self.joints_position_field.append(field)

        print("Controller started successfully.")
        
        # Main Loop
        while self.step(self.time_step) != -1:
            # 1. Reset Physics to prevent momentum crashes
            self.root_node_ref.resetPhysics()
            
            # 2. Get Input
            key = self.key_board.getKey()
            
            dx = 0
            dz = 0
            is_walking = False
            rotation_angle = None
            
            if key == 315: # UP
                dx = 0
                dz = -1
                rotation_angle = 0 
                is_walking = True
            elif key == 317: # DOWN
                dx = 0
                dz = 1
                rotation_angle = 3.14159 
                is_walking = True
            elif key == 314: # LEFT
                dx = -1
                dz = 0
                rotation_angle = 1.57 
                is_walking = True
            elif key == 316: # RIGHT
                dx = 1
                dz = 0
                rotation_angle = -1.57 
                is_walking = True

            # 3. Move and Animate
            if is_walking:
                self.animation_timer += (self.time_step / 1000.0)
                
                # Calculate Position
                current_pos = self.root_translation_field.getSFVec3f()
                move_factor = self.speed * (self.time_step / 1000.0)
                new_x = current_pos[0] + (dx * move_factor)
                new_z = current_pos[2] + (dz * move_factor)
                
                # Calculate Animation Ratio
                ratio = (self.animation_timer * self.speed) / self.CYCLE_TO_DISTANCE_RATIO
                current_sequence = int(ratio % self.WALK_SEQUENCES_NUMBER)
                ratio = ratio - int(ratio) 

                # Calculate Height (Bobbing)
                self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                    self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                
                # Apply Position (with SAFE HEIGHT 1.38)
                self.root_translation_field.setSFVec3f([new_x, self.ROOT_HEIGHT + self.current_height_offset, new_z])
                
                # Apply Rotation
                if rotation_angle is not None:
                    self.root_rotation_field.setSFRotation([0, 1, 0, rotation_angle])

                # Move Limbs
                for i in range(0, self.BODY_PARTS_NUMBER):
                    current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                        self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                    self.joints_position_field[i].setSFFloat(current_angle)
            
            else:
                # Standing Pose
                stand_sequence = 0 
                for i in range(0, self.BODY_PARTS_NUMBER):
                     self.joints_position_field[i].setSFFloat(self.angles[i][stand_sequence])
                
                # Reset to Safe Height
                current_pos = self.root_translation_field.getSFVec3f()
                self.root_translation_field.setSFVec3f([current_pos[0], self.ROOT_HEIGHT, current_pos[2]])
                
                # Lock rotation upright
                current_rot = self.root_rotation_field.getSFRotation()
                self.root_rotation_field.setSFRotation([0, 1, 0, current_rot[3]])

controller = Pedestrian()
controller.run()