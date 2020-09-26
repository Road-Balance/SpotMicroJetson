import kinematics as kn
import numpy as np
from adafruit_servokit import ServoKit
import board
import busio
import time

class Controllers:
    def __init__(self):

        #1 by 12 array
        self.MOTOR_LEG_FRONT = 0
        self.MOTOR_LEG_BACK = 6
        self.MOTOR_LEG_LEFT = 0
        self.MOTOR_LEG_RIGHT = 3
        self.MOTOR_LEG_SHOULDER = 2
        self.MOTOR_LEG_UPPER = 1
        self.MOTOR_LEG_LOWER = 0

        #4 by 3 matrix
        self.SIM_LEG_FRONT = 0
        self.SIM_LEG_BACK = 2
        self.SIM_LEG_LEFT = 0
        self.SIM_LEG_RIGHT = 1
        self.SIM_LEG_THETA1 = 0
        self.SIM_LEG_THETA2 = 1
        self.SIM_LEG_THETA3 = 2
        

        


if __name__=="__main__":
    legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    thetas = kn.initKinematics(legEndpoints) #radians
    print(thetas)
    kn.plotKinematics()


    print("Initializing Servos")
    i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
    print("Initializing ServoKit")
    kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x40)
    print("Done initializing")


    # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
    val_list = [0, 180, 90, 180, 0, 90,
                0, 180, 90, 180, 0, 90]


    for x in range(len(val_list)):
        kit.servo[x].angle = val_list[x]

    #kit.servo[num].angle=degree





