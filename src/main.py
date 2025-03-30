from motor import Motor
from arm import Arm
from vector import Vector, Joints

# Define motor pins
dir_1 = 27 
puls_1 = 13

dir_2 = 12
puls_2 = 19

dir_3 = 5
puls_3 = 16

# initialize motors
motor_1 = Motor(pulse_pin=puls_1, dir_pin=dir_1, limit_pin=..., step_per_rev=1600, min_delay=0.001, max_delay=0.01, gear_ratio=1)
motor_2 = Motor(pulse_pin=puls_2, dir_pin=dir_2, limit_pin=..., step_per_rev=1600, min_delay=0.001, max_delay=0.01, gear_ratio=5)
motor_3 = Motor(pulse_pin=puls_3, dir_pin=dir_3, limit_pin=..., step_per_rev=1600, min_delay=0.001, max_delay=0.01, gear_ratio=5)

def init_motors():
    motor_1.init_motor()
    motor_2.init_motor()
    motor_3.init_motor()

arm = Arm(base_pos=Vector(0, 0, 0), tip_pos=Vector(-56, -56, -50), arm1_length=80, arm2_length=50)
print(arm.get_tip_pos())
joints = arm.get_joint_angles()
motor_2.angle = joints.theta1_deg
motor_3.angle = joints.theta2_deg
breakpoint()
arm.plot()