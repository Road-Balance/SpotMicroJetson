import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1/200)

planeID = p.loadURDF("plane.urdf")
carId = p.loadURDF("racecar/racecar.urdf", basePosition=[0,0,0])
position, orientation = p.getBasePositionAndOrientation(carId)

for _ in range(10000):
    pos, ori = p.getBasePositionAndOrientation(carId)
    p.applyExternalForce(carId, 0, [50, 0, 0], pos, p.WORLD_FRAME)
    p.stepSimulation()
    time.sleep(1 / 200)

p.disconnect(physicsClient)
