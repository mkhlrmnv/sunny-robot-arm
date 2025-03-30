from motor import Motor
# from arm import Arm
# from vector import Vector, Joints
from cooling import FanController
import threading

# [[dir, puls]] => [[brown, white]]
pins = [[26, 13]]

cooler = FanController()
fan_thread = threading.Thread(target=cooler.run, daemon=True)
fan_thread.start()

# initialize motors
for pin in pins:
    motor = Motor(pulse_pin=pin[1], dir_pin=pin[0], limit_pin=23, step_per_rev=1600, min_delay=0.001, max_delay=0.01, gear_ratio=1)
    motor.init_motor(direction=1)

