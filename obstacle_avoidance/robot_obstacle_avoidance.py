import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and robot (simple cube)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", basePosition=[0,0,0.1])

# Add some obstacles (cubes)
obstacle1 = p.loadURDF("cube.urdf", basePosition=[2, 0, 0.1], globalScaling=0.5)
obstacle2 = p.loadURDF("cube.urdf", basePosition=[1, 1, 0.1], globalScaling=0.5)

# Simulation parameters
p.setGravity(0,0,-9.8)
timeStep = 1./240.
p.setTimeStep(timeStep)

# Simple obstacle avoidance loop
for i in range(1000):
    robot_pos, _ = p.getBasePositionAndOrientation(robotId)
    
    # Move forward
    new_pos = [robot_pos[0] + 0.01, robot_pos[1], robot_pos[2]]
    
    # Simple collision detection using rayTest
    ray_start = robot_pos
    ray_end = [robot_pos[0] + 0.2, robot_pos[1], robot_pos[2]]
    ray = p.rayTest(ray_start, ray_end)
    
    # If no collision, move forward
    if ray[0][0] < 0:  # no hit
        p.resetBasePositionAndOrientation(robotId, new_pos, [0,0,0,1])
    else:
        # Turn right if obstacle detected
        new_pos = [robot_pos[0], robot_pos[1] + 0.05, robot_pos[2]]
        p.resetBasePositionAndOrientation(robotId, new_pos, [0,0,0,1])
    
    p.stepSimulation()
    time.sleep(timeStep)

# Disconnect
p.disconnect()
