"""
Simulation of SpotMicroAI and it's Kinematics 
Use a keyboard to see how it works
Use keyboard-Button to switch betweek walk on static-mode
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
import datetime as dt
import matplotlib.animation as animation
import random
from thinputs_keyboard import ThreadedInputsKeyBoard
import spotmicroai

from kinematicMotion import KinematicMotion, TrottingGait
from environment import environment

rtime=time.time()
env=environment()

def reset():
    global rtime
    rtime=time.time()    

robot=spotmicroai.Robot(False,False,reset)

# TODO: Needs refactoring
speed1=240
speed2=170
speed3=300

speed1=322
speed2=237
speed3=436

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepHeight = p.addUserDebugParameter("step height", 0, 150, stepHeight)


walk=False

# keyboard Initialisation
# Dictionary of game controller buttons we want to include.
keyboardInputs = {'ABS_X': 128, 'ABS_RZ': 127, 'ABS_Y': 126, 'ABS_Z': 125,
                 'BTN_TIGGER': 124, 'BTN_THUMB': 123, 'BTN_THUMB2': 122, 'BTN_TOP': 121, # right side of keyboard
                 'ABS_HAT0X': 120, 'ABS_HAT0Y': 119, # left
                 'BTN_TOP2': 118, 'BTN_BASE': 117, # left top
                 'BTN_PINKIE': 116, 'BTN_BASE2': 115 # right top
                 }

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

def handleKeyboard():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz, walk
    commandValue = keyboard.read()

    # keyboard button command filter
    if commandValue == 'w':
        joy_x += 1
    if commandValue == 'e':
        joy_y += 1
    if commandValue == 's':
        joy_x -= 1
    if commandValue == 'd':
        joy_y -= 1
    if commandValue == 'q':
        resetPose()

IDheight = p.addUserDebugParameter("height", -40, 90, 40)
# IDstepLength = p.addUserDebugParameter("step length", -150, 150, 0.0)

Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

motion=KinematicMotion(Lp)
resetPose()

# Initialise the keyboard object using the keyboard inputs Python package
# keyboard = ThreadedInputsKeyBoard()
# keyboard.start()

trotting=TrottingGait()

def main():
    s=False
    while True:

        bodyPos=robot.getPos()
        bodyOrn,_,_=robot.getIMU()
        xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
        distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
        if distance>50:
            robot.resetBody()
    
        ir=xr/(math.pi/180)

        # handleKeyboard()
        
        d=time.time()-rtime
        height = p.readUserDebugParameter(IDheight)

        # wait 3 seconds to start
        if d>3:
            print(trotting.positions(d-3))
            robot.feetPosition(trotting.positions(d-3))
        else:
            robot.feetPosition(Lp)
        #roll=-xr
        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))
        robot.step()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")