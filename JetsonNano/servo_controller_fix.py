"""

servo_controller_fix.py
====

본 파일은 이전 배포판에서 사용한 ServoKit 라이브러리를 PCA9685 라이브러리로 교체한 파일입니다.
PCA9685 라이브러리는 example/test_servos_*.py에서 사용된 것과 동일합니다.
PWM제어에 필요한 min_pulse, max_pulse, frequency를 직접 수정할 수 있으며,
이전 배포판보다 정확하게 서보모터 각도를 제어할 수 있습니다.

"""
import sys
sys.path.append("..")

import Kinematics.kinematics as kn
import numpy as np

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import time

class Controllers:
    def __init__(self):

        print("Initializing Servos")
        self._i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        print("Initializing ServoKit")
        self._pca_1 = PCA9685(self._i2c_bus0, address=0x40)
        self._pca_1.frequency = 60
        self._pca_2 = PCA9685(self._i2c_bus0, address=0x41)
        self._pca_2.frequency = 60

        self._servos = list()
        for i in range(0, 12):
            if i<6:
                self._servos.append(servo.Servo(self._pca_1.channels[i], min_pulse=460, max_pulse=2440))
            else:
                self._servos.append(servo.Servo(self._pca_2.channels[i], min_pulse=460, max_pulse=2440))

        print("Done initializing")

        # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
        # centered position perpendicular to the ground
        self._servo_offsets = [180, 90, 90, 1, 90, 90,
                    180, 90, 90, 1, 90, 90]

        self._val_list = [ x for x in range(12) ]

        # All Angles for Leg 3 * 4 = 12 length
        self._thetas = []

    def getDegreeAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La

    # Angle mapping from radian to servo angles
    def angleToServo(self, La):
        self.getDegreeAngles(La)

        #FL Lower
        self._val_list[0] = self._servo_offsets[0] - self._thetas[0][2]
        #FL Upper
        self._val_list[1] = self._servo_offsets[1] - self._thetas[0][1]    
        #FL Shoulder
        self._val_list[2] = self._servo_offsets[2] + self._thetas[0][0]

        #FR Lower
        self._val_list[3] = self._servo_offsets[3] + self._thetas[1][2]
        #FR Upper
        self._val_list[4] = self._servo_offsets[4] + self._thetas[1][1]    
        #FR Shoulder
        self._val_list[5] = self._servo_offsets[5] - self._thetas[1][0]

        #BL Lower
        self._val_list[6] = self._servo_offsets[6] - self._thetas[2][2]
        #BL Upper
        self._val_list[7] = self._servo_offsets[7] - self._thetas[2][1]    
        #BL Shoulder, Formula flipped from the front
        self._val_list[8] = self._servo_offsets[8] - self._thetas[2][0]

        #BR Lower. 
        self._val_list[9] = self._servo_offsets[9] + self._thetas[3][2]
        #BR Upper
        self._val_list[10] = self._servo_offsets[10] + self._thetas[3][1]    
        #BR Shoulder, Formula flipped from the front
        self._val_list[11] = self._servo_offsets[11] + self._thetas[3][0]     

    def getServoAngles(self):
        return self._val_list

    def servoRotate(self, thetas):
        self.angleToServo(thetas)

        for x in range(len(self._val_list)):
            
            if (self._val_list[x] > 180):
                print("Over 180!!")
                self._val_list[x] = 179
            if (self._val_list[x] <= 0):
                print("Under 0!!")
                self._val_list[x] = 1
                
            self._servos[x].angle = float(self._val_list[x])


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
    kn.plotKinematics()
