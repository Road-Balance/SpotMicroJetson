import kinematics as kn
import numpy as np
from adafruit_servokit import ServoKit
import board
import busio
import time

class Controllers:
    def __init__(self):

        print("Initializing Servos")
        self._i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        print("Initializing ServoKit")
        self._kit = ServoKit(channels=16, i2c=_i2c_bus0, address=0x40)
        self._kit2 = ServoKit(channels=16, i2c=_i2c_bus0, address=0x41)
        print("Done initializing")

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

        # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
        # centered position perpendicular to the ground
        self._servo_offsets = [170, 60, 90, 10, 120, 90,
                    170, 60, 90, 10, 120, 90]

        self._val_list = [ x for x in range(12) ]

        # All Angles for Leg 3 * 4 = 12 length
        self._thetas = []

    def getRadianAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La

    # Angle mapping from radian to servo angles
    def angleToServo(self, La):
        self.getRadianAngles(La)

        #FL Lower
        self._val_list[0] -= self._thetas[0][2]
        #FL Upper
        self._val_list[1] -= self._thetas[0][1]    
        #FL Shoulder
        self._val_list[2] += self._thetas[0][0]

        #FR Lower
        self._val_list[3] += self._thetas[1][2]
        #FR Upper
        self._val_list[4] += self._thetas[1][1]    
        #FR Shoulder
        self._val_list[5] -= self._thetas[1][0]

        #BL Lower
        self._val_list[6] -= self._thetas[2][2]
        #BL Upper
        self._val_list[7] -= self._thetas[2][1]    
        #BL Shoulder, Formula flipped from the front
        self._val_list[8] -= self._thetas[2][0]

        #BR Lower. 
        self._val_list[9] += self._thetas[3][2]
        #BR Upper
        self._val_list[10] += self._thetas[3][1]    
        #BR Shoulder, Formula flipped from the front
        self._val_list[11] += self._thetas[3][0]     

    def getServoAngles(self):
        return self._val_list

    def servoRotate(self, thetas):
        self.angleToServo(thetas)

        for x in range(len(self._val_list)):
            if x < 6:
                self._kit2.servo[x].angle = self._val_list[x]
            else:
                self._kit.servo[x].angle = self._val_list[x]


if __name__=="__main__":
    legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    thetas = kn.initIK(legEndpoints) #radians
    
    controller = Controllers()

    # Get radian thetas, transform to integer servo angles
    # then, rotate servos
    controller.servoRotate(thetas)

    # Get AngleValues for Debugging
    svAngle = controller.getServoAngles()
    print(svAngle)

    # #plot at the end
    # kn.plotKinematics()