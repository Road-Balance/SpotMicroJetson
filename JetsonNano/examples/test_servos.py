"""
Class to Control the Servos for all Legs
"""
import Adafruit_PCA9685
import board
import busio
import time
from adafruit_servokit import ServoKit

class Servo:
    def __init__(self,direction,servo):
        self.direction=direction
        self.offset=0
        self.min=0
        self.max=180
        self.servo=servo
        self.lastValue=0

    def setValue(self,value):
        self.lastValue=value
        realValue=self.offset+self.direction*value
        print("realValue : ", realValue)
        # I2C set calculated Value - current dummy is servo center
        self.servo.angle=realValue
        
        # sweep = range(current_value, aim_value, -1/+1)
        # for degree in sweep :
        #     kit.servo[0].angle=degree
        #     time.sleep(0.01)
        # current_value = aim_value
    
    def getLastValue(self):
        return self.lastValue

    def setOffset(self,offset):
        self.offset=offset
        self.setValue(self.getLastValue())

class AcceleratedServo(Servo):
    def __init__(self,direction,servo):
        Servo.__init__(self,direction,servo)

        # random values for now
        self.kp=0.2
        self.kd=0.2
        self.max_acceleration=14

    def moveTo(self,value,inTime):
        print("Moving servo to {} in {}ms".format(value,inTime))

class Servos:

    def __init__(self):
        print("Servos init")

        i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        # self.pca = Adafruit_PCA9685.PCA9685(i2c)
        # self.pca = Adafruit_PCA9685.PCA9685(address=0x40)
        kit = ServoKit(channels=16, i2c=i2c_bus0)

        # Front_left,Front_right,Back_left,Back_right
        # each leg has shoulder, leg, foot
        directions=[-1,1,1,1,1,1,-1,1,1,1,1,1]
        self.leg_servos=[AcceleratedServo(directions[x],kit.servo[x]) for x in range(0,12)]

    def setServoPositions(self,positions):
        [self.leg_servos[x].moveTo(positions[x],100) for x in range(0,12)]
    
    def getServoOffset(self,servo):
        return self.leg_servos[servo].offset

    def setServoOffset(self,servo,offset):
        self.leg_servos[servo].setOffset(offset)


if __name__ == "__main__":
    try:
        servos=Servos()
        servos.setServoPositions([0 for _ in range(0,12)])
        for i in range(0, 4):
            servos.setServoOffset(i, 0)
            time.sleep(0.5)

        # print(servos.getServoOffset(0))
        # print(servos.getServoOffset(1))

    except Exception as e:
        print(e)
    finally:
        print("Successfully Done...")
