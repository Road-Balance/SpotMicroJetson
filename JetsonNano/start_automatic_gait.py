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
import keyboard
# import servo_controller

from multiprocessing import Process, Queue
from kinematicMotion import KinematicMotion, TrottingGait



# keyboard Initialisation
# Dictionary of keyboard controller buttons we want to include.
kb_default = {'w': 0, 'a': 0, 's': 0, 'd': 0, 'q': 0, 'e': 0}
control_offset = {'IDstepLength': 0.0, 'IDstepWidth': 0.0, 'IDstepAlpha': 0.0}



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

walk=False

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

motion=KinematicMotion(Lp)
resetPose()


trotting=TrottingGait()

def main(id, command_status):

    jointAngles = []

    while True:
        xr = 0.0
        yr = 0.0

        # Reset when robot pose become strange
        # robot.resetBody()
    
        ir=xr/(math.pi/180)
        
        d=time.time()-rtime
        height = 40 # p.readUserDebugParameter(IDheight)

        # calculate robot step command from keyboard inputs
        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)

        # wait 3 seconds to start
        if d>3:
            # trotting.positions(d-3) values are end points for each legs
            robot.feetPosition(trotting.positions(d-3))
        else:
            robot.feetPosition(Lp)
        #roll=-xr
        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))

        # Get current Angles for each motor
        jointAngles = robot.getAngle()
        # print(jointAngles)
        
        # First Step doesn't contains jointAngles
        if len(jointAngles):
            # Real Actuators
            pass
            # controller.servoRotate(jointAngles)
            
            # # Plot Robot Pose into Matplotlib for Debugging
            # TODO: Matplotplib animation
            # kn.initFK(jointAngles)
            # kn.plotKinematics()

        robot.step()


def resetStatus(key_status):
    result_dict = key_status.get()
    key_status.put({'w': 0, 'a': 0, 's': 0, 'd': 0, 'q': 0, 'e': 0})

def keyCounter(key_status, character):
    result_dict = key_status.get()
    result_dict[character] += 1
    key_status.put(result_dict)

def calcRbStep(key_status, command_status):
    result_dict = key_status.get()
    command_dict = command_status.get()
    command_dict['IDstepLength'] = 10.0 * result_dict['s'] - 10.0 * result_dict['w']
    command_dict['IDstepWidth'] = 5.0 * result_dict['d'] - 5.0 * result_dict['a']
    command_dict['IDstepAlpha'] = 3.0 * result_dict['q'] - 3.0 * result_dict['e']

    key_status.put(result_dict)
    command_status.put(command_dict)


def keyInterrupt(id, key_status, command_status):
    was_pressed = False

    while True:
        if keyboard.is_pressed('w'):
            if not was_pressed:
                keyCounter(key_status, 'w')
                was_pressed = True
        elif keyboard.is_pressed('a'):
            if not was_pressed:
                keyCounter(key_status, 'a')
                was_pressed = True
        elif keyboard.is_pressed('s'):
            if not was_pressed:
                keyCounter(key_status, 's')
                was_pressed = True
        elif keyboard.is_pressed('d'):
            if not was_pressed:
                keyCounter(key_status, 'd')
                was_pressed = True
        elif keyboard.is_pressed('q'):
            if not was_pressed:
                keyCounter(key_status, 'q')
                was_pressed = True
        elif keyboard.is_pressed('e'):
            if not was_pressed:
                keyCounter(key_status, 'e')
                was_pressed = True
        elif keyboard.is_pressed('space'):
            if not was_pressed:
                resetStatus(key_status)
                was_pressed = True
        else:
            was_pressed = False

        calcRbStep(key_status, command_status)

if __name__ == "__main__":
    try:

        key_status = Queue()
        key_status.put(kb_default)

        command_status = Queue()
        command_status.put(control_offset)

        # Keyboard input Process
        KeyInterrupt = Process(target=keyInterrupt, args=(1, key_status, command_status))
        KeyInterrupt.start()

        main(1, command_status)
        
        print("terminate process")
        if KeyInterrupt.is_alive():
            KeyInterrupt.terminate()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")