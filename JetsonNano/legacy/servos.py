"""
SpotMicroAI - Servos
"""

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import busio

class AcceleratedServo():
    
    def __init__(self,servo):
        self.servo=servo

    def angle(self,angle):
        self.servo.angle=angle

class Servos():

    def __init__(self):
        self.i2c = busio.I2C(SCL_1, SDA_1)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        self.servo = AcceleratedServo(servo.Servo(self.pca.channels[7],min_pulse=771,max_pulse=2740))

    def deinit(self):
        self.pca.deinit()

    def angle(self,servo,angle):
        self.servo.angle(angle)

if __name__ == "__main__":
    try:
        servos=Servos()
        # servos.setServoPositions([0 for _ in range(0,12)])
        servos.angle(0)

        # for i in range(0, 4):
        #     servos.angle(i, 0)
        #     time.sleep(0.5)

        # print(servos.getServoOffset(0))
        # print(servos.getServoOffset(1))

    except Exception as e:
        print(e)
    finally:
        print("Successfully Done...")
