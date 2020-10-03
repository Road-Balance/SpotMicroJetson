# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import time


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0)

# kit[0] is the bottom servo
# kit[1] is the top servo
print("Done initializing")

# sweep = range(0,30,)
# for degree in sweep:
#     kit.servo[0].angle=0
#     time.sleep(0.01)

# sweep = range(30,0,-1)
# for degree in sweep:
#     kit.servo[0].angle=0
#     time.sleep(0.01)

while 1:
    kit.servo[5].angle=110
    # sweep = range(110,110)
    # for degree in sweep:
    #     kit.servo[5].angle=degree
    #     time.sleep(0.02)

    # time.sleep(0.01)

    # sweep = range(110, 110, -1)
    # for degree in sweep :
    #     kit.servo[5].angle=degree
    #     time.sleep(0.02)
