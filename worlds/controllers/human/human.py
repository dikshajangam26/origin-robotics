from controller import Robot, DistanceSensor
import random

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get distance sensors
front_sensor = robot.getDevice("front sensor")
left_sensor = robot.getDevice("left sensor")
right_sensor = robot.getDevice("right sensor")

front_sensor.enable(timestep)
left_sensor.enable(timestep)
right_sensor.enable(timestep)

# Movement parameters
speed = 1.0  # meters per second
direction = [speed, 0, 0]  # Initial velocity along X-axis
duration = 200  # Time steps before random direction change
counter = 0

# Main loop
while robot.step(timestep) != -1:
    # Read sensor values
    front = front_sensor.getValue()
    left = left_sensor.getValue()
    right = right_sensor.getValue()

    # Obstacle detection threshold
    threshold = 800
    obstacle = front < threshold or left < threshold or right < threshold

    if obstacle:
        # Avoid obstacle: pick a new random direction
        direction = [
            random.choice([-1, 1]) * speed * round(random.random(), 2),
            random.choice([-1, 1]) * speed * round(random.random(), 2),
            0
        ]
        counter = 0
    else:
        # Timed random direction change
        counter += 1
        if counter > duration:
            direction = [
                random.choice([-1, 1]) * speed,
                random.choice([-1, 1]) * speed,
                0
            ]
            counter = 0

    # Apply velocity (linear only, no angular)
    robot.getPhysics().setVelocity([direction[0], direction[1], direction[2], 0, 0, 0])