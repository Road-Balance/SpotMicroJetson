"""
Simulation of SpotMicroAI and it's Kinematics 
Use a keyboard to see how it works
Use keyboard-Button to switch betweek walk on static-mode
"""
from os import system, name 
import sys
sys.path.append("..")

import matplotlib.animation as animation
import numpy as np
import time 
import math
import datetime as dt
import keyboard
import random

from environment import environment
import pybullet as p
import pybullet_data

import spotmicroai

from multiprocessing import Process
from Common.multiprocess_kb import KeyInterrupt

import kinematics as kn
from kinematicMotion import KinematicMotion, TrottingGait

rtime=time.time()
env=environment()

def reset():
    global rtime
    rtime=time.time()    

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

# define our clear function 
def consoleClear():

    # for windows 
    if name == 'nt': 
        _ = system('cls') 
  
    # for mac and linux(here, os.name is 'posix') 
    else: 
        _ = system('clear') 

robot=spotmicroai.Robot(True,True,reset)

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132

IDheight = p.addUserDebugParameter("height", -40, 90, 20)

Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

resetPose()
trotting=TrottingGait()

def main(id, command_status):
    
    s=False

    while True:
        bodyPos=robot.getPos()
        bodyOrn,_,_=robot.getIMU()
        xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
        distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)

        if distance>50:
            robot.resetBody()
    
        ir=xr/(math.pi/180)
        
        d=time.time()-rtime
        height = p.readUserDebugParameter(IDheight)

        # calculate robot step command from keyboard inputs
        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)

        print(robot.getAngle())

        if result_dict['StartStepping']:
            robot.feetPosition(trotting.positions(d-3, result_dict))
        else:
            robot.feetPosition(Lp)

        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))

        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))

        robot.step()
        consoleClear()

if __name__ == "__main__":
    try:
        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process 
        main(2, KeyInputs.command_status)
        
        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()

    except Exception as e:
        print(e)
    finally:
        print("Done... :)")