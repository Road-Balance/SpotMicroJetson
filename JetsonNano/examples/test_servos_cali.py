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
kit2 = ServoKit(channels=16, i2c=i2c_bus0, address=0x41)

# kit[0] is the front servos
# kit[1] is the rear servos
print("Done initializing")

# [0]~[2] : FL // [3]~[5] : FR // [6]~[8] : RL // [9]~[11] : RR
val_list = [90]
prev_angle = None
cur_angle = None
# val_list = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]


if __name__ == '__main__':

    while True:
        # motro_num is index of motor to rotate
        motro_num=int(input("Enter Servo to rotate (0-11): "))
        
        # new angle to be written on selected motor
        cur_angle=int(input("Enter new angles (0-180): "))
        
        # increase(decrease) prev_angle to angle by 1 degree
        if prev_angle:
            sweep = range(prev_angle, cur_angle, 1) if (prev_angle < cur_angle) else range(prev_angle, cur_angle, -1)

            for degree in sweep:
                if motro_num < 6:
                    kit.servo[int(motro_num%6)].angle = cur_angle
                else:
                    kit2.servo[int(motro_num%6)].angle = cur_angle
                time.sleep(0.01)
        else:
            if motro_num < 6:
                kit.servo[int(motro_num%6)].angle = cur_angle
            else:
                kit2.servo[int(motro_num%6)].angle = cur_angle


        prev_angle = val_list[motro_num]