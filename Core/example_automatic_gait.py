"""
Simulation of SpotMicroAI and it's Kinematics 
Use a keyboard to see how it works
Use keyboard-Button to switch betweek walk on static-mode
"""
import matplotlib.animation as animation
import numpy as np
import time
import math
import datetime as dt
import random

import kinematics as kn
import spotmicroai
# import servo_controller

# from without_sim import spotmicroai

from thinputs_keyboard import ThreadedInputsKeyBoard
from kinematicMotion import KinematicMotion, TrottingGait

rtime=time.time()

def reset():
    global rtime
    rtime=time.time()    

robot=spotmicroai.Robot(False,False,reset)
# controller = servo_controller.Controllers()

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

# Initial End point X Value for Front legs 
iXf=120

# for animation
jointAngles = []

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

# robot height, 40 default
# IDheight = p.addUserDebugParameter("height", -40, 90, 40)

Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

motion=KinematicMotion(Lp)
resetPose()

# Initialise the keyboard object using the keyboard inputs Python package
# keyboard = ThreadedInputsKeyBoard()
# keyboard.start()

trotting=TrottingGait()

def main():

    s = False

    while True:
        xr = 0.0
        yr = 0.0

        # robot.resetBody()
    
        ir=xr/(math.pi/180)
        
        d=time.time()-rtime
        height = 40 # p.readUserDebugParameter(IDheight)

        # wait 3 seconds to start
        if d>3:
            # trotting.positions(d-3) - position of end points for each legs
            robot.feetPosition(trotting.positions(d-3))
        else:
            robot.feetPosition(Lp)
        #roll=-xr
        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))

        # Get current Angles for each motors 
        jointAngles = robot.getAngle()
        print(jointAngles)
        
        # First Step doesn't contains jointAngles
        if len(jointAngles):
            # Real Actuators
            pass
            # controller.servoRotate(jointAngles)
            
            # # Debugging
            # kn.initFK(jointAngles)
            # kn.plotKinematics()

        robot.step()
        # time.sleep(0.2)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")