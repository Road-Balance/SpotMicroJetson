"""
Reference from : http://blog.naver.com/einsbon/221549883110
"""

import pybullet as p
import time
import pybullet_data

# A simulation is started using the p.connect() method. 
# Several connection modes are available, with 
# p.GUI allowing for visualization and debugging, 
# and p.DIRECT providing the fastest, non-visual connection. 
physicsClient = p.connect(p.GUI)

# The module pybullet_data provides many example Universal Robotic Description
# Format (URDF) files. 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# You can set how many seconds to update once 
# you update in a physical simulation.
p.setTimeStep(1/200)

# Note that the returned value is an integer, not a modifiable object. 
# This integer is the ID passed into other functions
# to query the state of a robot and perform actions on it. 
planeID = p.loadURDF("plane.urdf")

# Note that the returned value is an integer,
# not a modifiable object. 
# This integer is the ID passed into other functions 
# to query the state of a robot and perform actions on it.
cubeStartPos = [0, 0, 0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 3.14])

# The loadURDF function can also set the location and rotation state.
# Rotation status is changed from oiler angle to quarterian format on next line
# because it receives a quarterionic factor.
robotID = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1 / 200)

p.disconnect(physicsClient)