"""
Reference from: https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-1-13895a622b24
"""

import pybullet as p
import pybullet_data

from time import sleep

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)

# Sliders are added to our GUI view that allow us to dynamically
# input values to our program through the simulation. 
# The ID returned by addUserDebugParameter() is passed to 
# readUserDebugParameter() to retrieve the value on the slider. 
angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = p.addUserDebugParameter('Throttle', -5, 5, 0)

car = p.loadURDF('simplecar.urdf', [0, 0, 0.1])
number_of_joints = p.getNumJoints(car)
for joint_number in range(number_of_joints):
    # This prints out all the information returned by getJointInfo().
    # Try it out 
    info = p.getJointInfo(car, joint_number)
    print(info[0], ": ", info[1])

plane = p.loadURDF('plane.urdf')


wheel_indices = [1, 3, 4, 5]
hinge_indices = [0, 2]

while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        # This allows us to change the velocity, position, 
        # or apply a torque to a joint. 
        # This is the main method used to control robots. 
        # It takes both a robot ID and a joint ID
        p.setJointMotorControl2(car, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.POSITION_CONTROL, 
                                targetPosition=user_angle)
    p.stepSimulation()


p.disconnect(physicsClient)
