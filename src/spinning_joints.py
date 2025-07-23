from gpiozero import DigitalOutputDevice
import time
from gpiozero import Button
from signal import pause

from config import *

from multiprocessing import Event

class SpinningJoints:
    def __init__(self, 
                 shared, 
                 pulse_pin, 
                 dir_pin, 
                 name, 
                 limit_pin, 
                 step_per_rev=1600, 
                 gear_ratio=5, 
                 min_delay: float = MOTORS_MIN_DELAY, 
                 max_delay: float = MOTORS_MAX_DELAY, 
                 angle_limit=360):
        """
        Initialize the stepper motor with the given GPIO pins.

        :param pulse_pin: GPIO pin for sending step pulses.
        :param dir_pin: GPIO pin for setting the direction.
        """
        # asserts:)
        assert isinstance(pulse_pin, int), "Pulse pin must be an integer."
        assert isinstance(dir_pin, int), "Direction pin must be an integer."
        assert isinstance(name, str), "Name must be a string"
        assert isinstance(step_per_rev, int), "Steps per revolution must be an integer."
        assert isinstance(gear_ratio, (int, float)), "Gear ratio must be a number."
        assert isinstance(min_delay, (int, float)), "Minimum delay must be a number."
        assert isinstance(max_delay, (int, float)), "Maximum delay must be a number."
        assert min_delay >= 0, "Minimum delay must be non-negative."
        assert max_delay >= min_delay, "Maximum delay must be greater than or equal to minimum delay."

        self.shared = shared

        self.name = name
        
        self.pulse = DigitalOutputDevice(pulse_pin)
        self.direction = DigitalOutputDevice(dir_pin)

        self.limit_event = Event()

        self.limit_switch = Button(limit_pin, pull_up=True, bounce_time=0.0001)
        self.limit_switch.when_pressed = lambda: (print(f"Metal detected by {self.name}"), 
                                                self.limit_event.set())
        self.limit_switch.when_released = lambda: (print(f"No metal anymore by {self.name}"), 
                                                    self.limit_event.clear())

        self.angle_limit = angle_limit

        self.angle = 0
            
        self.steps = 0
        self.step_per_rev = step_per_rev
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.gear_ratio = gear_ratio


    def calc_delay(self, speed_percent):
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)
    
    def init_motor(self, speed=0.1):
        # pontto motor init is little bit more complicated to not get the 
        # cabel tangled there for it has own init sequence
        if self.name=="pontto":
            direction_change = False

            direction = 1

            while not self.limit_event.is_set():
                if not direction_change and abs(self.angle) > (180-RAIL_ANGLE):
                    self.angle = 0
                    time.sleep(2)
                    direction = -1 * direction
                    direction_change = True

                if direction_change and abs(self.angle) > (180 + RAIL_ANGLE):
                    raise TimeoutError("Motor didn't find init pos")

                self.step(direction=direction, speed=speed)

            if not direction_change:
                self.move_by_angle(-10, speed=speed)
                time.sleep(2)
                while not self.limit_event.is_set():
                    self.step(direction=-1, speed=speed)
            else:
                time.sleep(2)
                self.move_by_angle(-1, speed=speed)

        else:
            # first it gets out of the shutdown pos by spinning 15 degrees
            self.move_by_angle(-17, speed=speed)
            time.sleep(1)

            # then spins until it finds the limit switch
            while not self.limit_event.is_set():
                if abs(self.angle) > 270:
                    raise TimeoutError("Motor didn't find init pos")
                self.step(direction=-1, speed=speed)

        self.reset_position()
        print(f"Motor {self.name} initialized")

    def shutdown(self):
        if self.name == "pontto":
            self.init_motor()

        if self.name == "paaty":
            self.init_motor()
            time.sleep(2)
            self.move_by_angle(10.5, speed=0.01)

    def step(self, direction=1, speed=0.1):
        if abs(direction * (360 / (self.step_per_rev * self.gear_ratio)) > self.angle_limit):
            raise ValueError("Angle exceeded the limit.")
        
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 (forward) or -1 (backward).")

        self.direction.value = 1 if direction == 1 else 0
        delay = self.calc_delay(speed)

        self.pulse.on()
        time.sleep(delay)
        self.pulse.off()
        time.sleep(delay)
        self.steps -= direction
        self.angle -= direction * (360 / (self.step_per_rev * self.gear_ratio))
        # self.angle = round(self.angle, 3)

        if self.name=="pontto":
            self.shared.theta_1 -= direction * (360 / (self.step_per_rev * self.gear_ratio))
            # self.shared.theta_1 = round(self.shared.theta_1, 3)
        elif self.name=="paaty":
            self.shared.theta_2 -= direction * (360 / (self.step_per_rev * self.gear_ratio))
            # self.shared.theta_2 = round(self.shared.theta_2, 3)

    def move_by_angle(self, angle, speed=0.1): #, shared=None):

        # if self.name == "pontto":
        #     self.angle = self.shared.theta_1
        # elif self.name == "paaty":
        #     self.angle = self.shared.theta_2

        if abs(angle) > self.angle_limit:
           raise ValueError("Angle must be between -360 and 360 degrees.")

        print("angle ", self.angle)
        angle_per_step = 360 / (self.step_per_rev * self.gear_ratio)
        steps = int(angle / angle_per_step)
        print("steps ", steps)
        direction = 1 if angle < 0 else -1

        for _ in range(abs(steps)):
            # print("curr ", self.angle)
            self.step(direction=direction, speed=speed)

        # if shared is not None:
        #     if self.name == "pontto":
        #         shared.theta_1 = self.angle
        #     elif self.name == "paaty":
        #         shared.theta_2 = self.angle

    def move_to_angle(self, target_angle, speed=0.1): #, shared=None):
        if abs(target_angle) > self.angle_limit:
            raise ValueError("Target angle must be between -360 and 360 degrees.")

        if self.name == "pontto":
            self.angle = self.shared.theta_1
        elif self.name == "paaty":
            self.angle = self.shared.theta_2

        angle_diff = target_angle - self.angle
        print("diff", angle_diff)

        self.move_by_angle(angle_diff, speed=speed) #, shared=shared)

    def get_steps(self):
        return self.steps

    def get_angle(self):
        return self.angle

    def reset_position(self):
        self.steps = 0
        self.angle = 0

    def cleanup(self):
        """
        Cleanup method to release GPIO resources.
        """
        self.pulse.close()
        self.direction.close()
        self.limit_switch.close()
        print(f"Motor {self.name} cleaned up")


if __name__ == "__main__":
    from multiprocessing import Process, Queue, Manager
    manager = Manager()
    shared = manager.Namespace()
    motor_paaty = SpinningJoints(shared, 
                                pulse_pin=PAATY_MOTOR_PULSE_PIN, 
                                dir_pin=PAATY_MOTOR_DIR_PIN, 
                                limit_pin=PAATY_MOTOR_SENSOR_PIN, 
                                name="paaty", 
                                gear_ratio=PAATY_MOTOR_GEAR_RATIO)
        
    motor_pontto = SpinningJoints(shared, 
                                pulse_pin=PONTTO_MOTOR_PULSE_PIN, 
                                dir_pin=PONTTO_MOTOR_DIR_PIN, 
                                limit_pin=PONTTO_MOTOR_SENSOR_PIN, 
                                name="pontto",
                                gear_ratio=PONTTO_MOTOR_GEAR_RATION)
    
    # motor_pontto.move_by_angle(-190, speed=0.1)
    # motor_paaty.move_by_angle(-90, speed=0.5)

    motor_paaty.init_motor(speed=0.1)
    # motor_pontto.init_motor(speed=0.1)

    time.sleep(2)

    motor_paaty.shutdown()
    # motor_pontto.shutdown()
