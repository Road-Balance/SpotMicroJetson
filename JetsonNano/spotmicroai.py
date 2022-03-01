"""
SpotMicroAI with Pybullet Simulation fully eliminated!!!
"""
# import pybullet as p

import sys
sys.path.append("..")

import time
import math
import numpy as np
from Kinematics.kinematics import Kinematic 
from enum import Enum

class RobotState(Enum):
    OFF = 0     # don't do anything
    READY = 1   # compact, legs together, waiting
    STAND = 2   # standing, feet to the ground
    TROTTING_GAIT=3 # legs away moving up/down 0/3,1/2 / 2 Step
    CRAWL = 4   # 4 Stepped, 1,2,3,0
    CRAWL2 = 5  #4 Stepped, Back first, 2,1,3,0

class Robot:

    def __init__(self,useFixedBase=False,useStairs=True,resetFunc=None):      

        # Simulation Configuration
        self.useMaximalCoordinates = False
        self.resetFunc=resetFunc
        self.useRealTime = True
        self.debugLidar=False
        self.rotateCamera=False
        self.debug=False
        self.fixedTimeStep = 1. / 550
        self.numSolverIterations = 200
        self.useFixedBase =useFixedBase
        self.useStairs=useStairs

        self.init_position=[0, 0, 0.3]

        self.reflection=False
        self.state=RobotState.OFF
        
        # Parameters for Servos - still wrong
        self.kp = 0.045#0.012
        self.kd = .4#.2
        self.maxForce = 25.0

        self.angles = []

        self.oldTextId=0
        self.textId=0
        self.oldDebugInfo=[]
        self.rot=(0,0,0)
        self.pos=(0,0,0)
        self.t=0

        self.IDkp = 0.045 #p.addUserDebugParameter("Kp", 0, 0.05, self.kp) # 0.05
        self.IDkd = 0.4 #p.addUserDebugParameter("Kd", 0, 1, self.kd) # 0.5
        self.IDmaxForce = 25.0 #p.addUserDebugParameter("MaxForce", 0, 50, 12.5)

        replaceLines=True
        self.numRays=360
        self.rayFrom=[]
        self.rayTo=[]
        self.rayIds=[]
        self.rayHitColor = [1,0,0]
        self.rayMissColor = [0,1,0]
        rayLen = 12
        rayStartLen=0.12
        
        self.W=75+5+40

        self.dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]
        self.roll=0

        self.Lp = np.array([[120, -100, self.W/2, 1], [120, -100, -self.W/2, 1],
        [-50, -100, self.W/2, 1], [-50, -100, -self.W/2, 1]])

        self.kin = Kinematic()
        self.ref_time = time.time()

        # Camera Settings
        fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100

        self.lastLidarTime=0
    
    def resetBody(self):
        self.ref_time=time.time()
        if len(self.oldDebugInfo)>0:
            for x in self.oldDebugInfo:
                p.removeUserDebugItem(x)        
        p.resetBasePositionAndOrientation(self.quadruped, self.init_position,[0,0,0,1])
        p.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
        if(self.resetFunc):
            self.resetFunc()

    def checkSimulationReset(self,bodyOrn):
        (xr, yr, _) = p.getEulerFromQuaternion(bodyOrn)

        # If our Body rotated more than pi/3: reset
        if(abs(xr) > math.pi/3 or abs(yr) > math.pi/3):
            self.resetBody()
            return True
        return False

    def bodyRotation(self,rot):
        self.rot=rot

    def bodyPosition(self,pos):
        self.pos=pos

    def feetPosition(self,Lp):
        self.Lp=Lp
  
    def getPos(self):
        bodyPos,_=p.getBasePositionAndOrientation(self.quadruped)
        return bodyPos

    def getAngle(self):
        return self.angles

    def getIMU(self):
        _, bodyOrn = p.getBasePositionAndOrientation(self.quadruped)
        linearVel, angularVel = p.getBaseVelocity(self.quadruped)
        return bodyOrn,linearVel,angularVel

    def step(self):

        if (self.useRealTime):
            self.t = time.time() - self.ref_time
        else:
            self.t = self.t + self.fixedTimeStep

        # print(self.t)

        kp=self.IDkp # p.readUserDebugParameter(self.IDkp)
        kd=self.IDkd #p.readUserDebugParameter(self.IDkd)
        maxForce=self.IDmaxForce #p.readUserDebugParameter(self.IDmaxForce)

        # Calculate Angles with the input of FeetPos,BodyRotation and BodyPosition
        self.angles = self.kin.calcIK(self.Lp, self.rot, self.pos)

        # print(self.Lp)
        # print(self.angles)
        # print([ (self.angles * 180 / 3.1415) for singleFootAngle in angles for angle in singleFootAngle ])

        if (self.useRealTime == False):
            # p.stepSimulation()
            time.sleep(self.fixedTimeStep)

    def getLp(self):
        return self.Lp
