from src.linear_rail import Motor
# from arm import Arm
# from vector import Vector, Joints
from cooling import FanController
import threading

# [[dir, puls]] => [[brown, white]]
pins = [[27, 4]]

cooler = FanController()
fan_thread = threading.Thread(target=cooler.run, daemon=True)
fan_thread.start()

# initialize motors
for pin in pins:
    motor = motor3 = Motor(pulse_pin=27, dir_pin=4, limit_pin=23, gear_ratio=1)
    motor.init_motor(direction=1)

