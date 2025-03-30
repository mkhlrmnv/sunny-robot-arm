from motor import Motor
# from arm import Arm
# from vector import Vector, Joints
from cooling import FanController
import threading

# Define motor pins
dir_1 = 19
puls_1 = 20

cooler = FanController()
fan_thread = threading.Thread(target=cooler.run, daemon=True)
fan_thread.start()

# initialize motors
motor_1 = Motor(pulse_pin=puls_1, dir_pin=dir_1, limit_pin=23, step_per_rev=1600, min_delay=0.001, max_delay=0.01, gear_ratio=1)


def init_motors():
    motor_1.init_motor(direction=-1)

init_motors()


# arm = Arm(base_pos=Vector(0, 0, 0), tip_pos=Vector(-56, -56, -50), arm1_length=80, arm2_length=50)
# print(arm.get_tip_pos())
# joints = arm.get_joint_angles()
# motor_2.angle = joints.theta1_deg
# motor_3.angle = joints.theta2_deg
# breakpoint()
# arm.plot()