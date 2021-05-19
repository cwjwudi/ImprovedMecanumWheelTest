"""my_pycon controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import env
import numpy as np

robot1 = env.Robot1()

timestep = robot1.timestep

robot1.init_arm_pos()

while robot1.robot.step(timestep) != -1:
    robot1.ctrl_key_scan()
    robot1.move()

    pass
