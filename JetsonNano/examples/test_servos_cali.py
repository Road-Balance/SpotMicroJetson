
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
kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x40)
# kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x41)
# kit[0] is the bottom servo
# kit[1] is the top servo
print("Done initializing")

# [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
val_list = [0, 180, 90, 180, 0, 90, \
            0, 180, 90, 180, 0, 90]


for x in range(len(val_list)):
    kit.servo[x].angle = val_list[x]


while True:
    # num은 각도를 바꿀 모터 번호이다.
    num=int(input("Enter Servo to calibrate (0-12)"))
    
    # print("Current Offset is {}".format(self.servos.getServoOffset(servo)))
    
    # offset은 위에서 정한 모터번호에 부여할 각도이다.
    offset=int(input("Enter new Offset:"))
     
    prev_offset = val_list[num]
    
    # prev_offset(현재각도)부터 offset(목표각도)까지 1도씩 증가 혹은 감소 
    sweep = range(prev_offset, offset, 1) if (prev_offset < offset) else range(prev_offset, offset, -1)
    for degree in sweep:
        kit.servo[num].angle=degree
        # kit.servo[1].angle=degree
        time.sleep(0.01)

    val_list[num] = offset