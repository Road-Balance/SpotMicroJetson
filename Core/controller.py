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
    thetas = kn.initIK(legEndpoints) #radians

    print("Initializing Servos")
    i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
    print("Initializing ServoKit")
    kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x40)
    kit_2 = ServoKit(channels=16, i2c=i2c_bus0, address=0x41)
    print("Done initializing")


    thetas *= 180/np.pi
    print(thetas)

    # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
    # centered position perpendicular to the ground
    val_list = [170, 60, 90, 10, 120, 90,
                170, 60, 90, 10, 120, 90]


    #FL Lower
    val_list[0] -= thetas[0][2]
    #FL Upper
    val_list[1] -= thetas[0][1]    
    #FL Shoulder
    val_list[2] += thetas[0][0]

    #FR Lower
    val_list[3] += thetas[1][2]
    #FR Upper
    val_list[4] += thetas[1][1]    
    #FR Shoulder
    val_list[5] -= thetas[1][0]


    #BL Lower. 
    val_list[6] -= thetas[2][2]
    #BL Upper
    val_list[7] -= thetas[2][1]    
    #BL Shoulder, Formula flipped from the front
    val_list[8] -= thetas[2][0]

    #BR Lower. 
    val_list[9] += thetas[3][2]
    #BR Upper
    val_list[10] += thetas[3][1]    
    #BR Shoulder, Formula flipped from the front
    val_list[11] += thetas[3][0]

    for x in range(len(val_list)):
        val_list[x] = int(val_list[x])
        print(val_list[x])
        if x < 6:
            kit_2.servo[x].angle = val_list[x]
        else:
            kit.servo[x].angle = val_list[x]

    #kit.servo[num].angle=degree


    #plot at the end
    kn.plotKinematics()




