'''
Includes Walking Mechanism for quadruped robot
'''

import time
import numpy as np
import math
import pybullet as p

class KinematicLegMotion:

    def __init__(self,LLp):
        self.rtime=time.time()
        self.running=False
        self.LLp=LLp

    def moveTo(self,newLLp,rtime,func=None):
        if self.running:
            # TODO: Queue the Requests
            print("Movement already running, please try again later.")
            return False
        self.startTime=time.time()
        self.startLLp=self.LLp
        self.func=func
        self.targetLLp=newLLp
        self.endTime=time.time()+rtime/1000
        self.running=True
        return True
    
    def update(self):
        diff=time.time()-self.startTime
        ldiff=self.targetLLp-self.startLLp
        tdiff=self.endTime-self.startTime
        ldiff/(tdiff*diff)
        p=1/tdiff*diff

        if time.time()>self.endTime and self.running:
            self.running=False
            p=1
        self.LLp=self.startLLp+ldiff*p
        if self.func:
            self.LLp=self.func(p,self.LLp)

    def step(self):
        if self.running:
            self.update()
        return self.LLp

class KinematicMotion:

    def __init__(self,Lp):
        self.Lp=Lp
        self.legs=[KinematicLegMotion(Lp[x]) for x in range(4)]

    def moveLegsTo(self,newLp,rtime):
        [self.legs[x].moveTo(newLp[x],rtime) for x in range(4)]

    def moveLegTo(self,leg,newLLp,rtime,func=None):
        return self.legs[leg].moveTo(newLLp,rtime,func)

    def step(self):
        return [x.step() for x in self.legs]


"""
This class will define the trotting-gait function
A complete cycle is tone in Tt
Each leg has the following "states"
0 - wait on ground for t0
1 - move on ground for steplength Sl for t1
2 - wait on ground for t2
3 - lift leg by Sh and Sl for t3 back to 0
"""
class TrottingGait:
    
    def __init__(self):
        self.maxSl=2
        self.bodyPos=(0,100,0)
        self.bodyRot=(0,0,0)
        self.t0=0 # senseless i guess
        self.t1=510
        self.t2=00
        self.t3=185
        self.Sl=0.0
        self.Sw=0
        self.Sh=60
        self.Sa=0
        self.Spf=87
        self.Spr=77

        self.IDspurFront= p.addUserDebugParameter("spur front", 20, 150, self.Spf)
        self.IDspurRear= p.addUserDebugParameter("spur rear", 20, 150, self.Spr)
        self.IDstepLength = p.addUserDebugParameter("step length", -150, 150, self.Sl)
        self.IDstepWidth = p.addUserDebugParameter("step width", -150, 150, self.Sw)
        self.IDstepHeight = p.addUserDebugParameter("step height", 0, 150, self.Sh)
        self.IDstepAlpha = p.addUserDebugParameter("step alpha", -90, 90, self.Sa)
        self.IDt0 = p.addUserDebugParameter("t0", 0, 1000, self.t0)
        self.IDt1 = p.addUserDebugParameter("t1", 0, 1000, self.t1)
        self.IDt2 = p.addUserDebugParameter("t2", 0, 1000, self.t2)
        self.IDt3 = p.addUserDebugParameter("t3", 0, 1000, self.t3)
        self.IDfrontOffset = p.addUserDebugParameter("front Offset", 0,200, 120)
        self.IDrearOffset = p.addUserDebugParameter("rear Offset", 0,200, 50)
        
        self.Rc=[-50,0,0,1] # rotation center


    """
    calculates the Lp - LegPosition for the configured gait for time t and original Lp of x,y,z
    """
    def calcLeg(self,t,x,y,z):
        startLp=np.array([x-self.Sl/2.0,y,z-self.Sw/2,1])
        endLp=np.array([x+self.Sl/2,y,z+self.Sw/2,1])
        
        if(t<self.t0): # stay on ground
            print("stay")
            return startLp
        elif(t<self.t0+self.t1): # drag foot over ground
            print("drag")
            td=t-self.t0
            tp=1/(self.t1/td)
            diffLp=endLp-startLp
            curLp=startLp+diffLp*tp

            psi=(math.pi/180*self.Sa)*tp
            Ry = np.array([[np.cos(psi),0,np.sin(psi),0],
                    [0,1,0,0],
                    [-np.sin(psi),0,np.cos(psi),0],[0,0,0,1]])
            curLp=Ry.dot(curLp)
            return curLp
        elif(t<self.t0+self.t1+self.t2): # stay on ground again
            print("stay")
            return endLp
        elif(t<self.t0+self.t1+self.t2+self.t3): # Lift foot
            print("lift")
            td=t-(self.t0+self.t1+self.t2)
            tp=1/(self.t3/td)
            diffLp=startLp-endLp
            curLp=endLp+diffLp*tp
            curLp[1]+=self.Sh*math.sin(math.pi*tp)
            return curLp
            
    def stepLength(self,len):
        self.Sl=len

    def positions(self,t,kb_offset={}):
        spf= p.readUserDebugParameter(self.IDspurFront)
        spr= p.readUserDebugParameter(self.IDspurRear)
        self.Sh=p.readUserDebugParameter(self.IDstepHeight)
        
        # Pybullet
        if list(kb_offset.values()) == [0.0, 0.0, 0.0]:
            self.Sl=p.readUserDebugParameter(self.IDstepLength)
            self.Sw=p.readUserDebugParameter(self.IDstepWidth)
            self.Sa=p.readUserDebugParameter(self.IDstepAlpha)
        else:
            self.Sl=kb_offset['IDstepLength']
            self.Sw=kb_offset['IDstepWidth']
            self.Sa=kb_offset['IDstepAlpha']
        self.t0=p.readUserDebugParameter(self.IDt0)
        self.t1=p.readUserDebugParameter(self.IDt1)
        self.t2=p.readUserDebugParameter(self.IDt2)
        self.t3=p.readUserDebugParameter(self.IDt3)

        Tt=(self.t0+self.t1+self.t2+self.t3)
        Tt2=Tt/2

        td=(t*1000)%Tt
        rtd=(t*1000-Tt2)%Tt
        
        Fx=p.readUserDebugParameter(self.IDfrontOffset)
        Rx=-p.readUserDebugParameter(self.IDrearOffset)

        Fy=-100
        Ry=-100

        # front left / front right / rear left / rear right
        r=np.array([self.calcLeg(td,Fx,Fy,spf),self.calcLeg(rtd,Fx,Fy,-spf),self.calcLeg(rtd,Rx,Ry,spr),self.calcLeg(td,Rx,Ry,-spr)])

        # Cat Trotting Example
        Tt4=Tt/4

        """
        How can implement cat Trotting??
        """

        return r







        # td=(t*1000)%Tt # front left
        # td2=(t*1000-Tt4*2)%Tt # front right
        # rtd=(t*1000-Tt4*1)%Tt # rear left
        # rtd2=(t*1000-Tt4*3)%Tt # rear right

        # r=np.array([self.calcLeg(td,Fx,Fy,spf),self.calcLeg(td2,Fx,Fy,-spf),self.calcLeg(rtd,Rx,Ry,spr),self.calcLeg(rtd2,Rx,Ry,-spr)])

