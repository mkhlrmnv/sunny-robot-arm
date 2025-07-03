from gpiozero import DigitalOutputDevice
import time
from gpiozero import Button
from signal import pause

class SpinningJoints:
    def __init__(self, pulse_pin, dir_pin, name, limit_pin, step_per_rev=1600, gear_ratio=5, min_delay=1e-4, max_delay=1e-3, angle_limit=360):
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

        self.name = name
        
        self.pulse = DigitalOutputDevice(pulse_pin)
        self.direction = DigitalOutputDevice(dir_pin)

        self.limit_switch = Button(limit_pin, pull_up=True, bounce_time=0.00001)
        self.limit_switch.when_pressed = lambda: (print("Metal detected"), 
                                                setattr(self, 'init_pos', True))
        self.limit_switch.when_released = lambda: (print("No metal anymore"), 
                                                    setattr(self, 'init_pos', False))

        self.angle_limit = angle_limit
        self.steps = 0
        self.angle = 0
        self.step_per_rev = step_per_rev
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.gear_ratio = gear_ratio

        self.init_pos = self.limit_switch.is_pressed 

    def calc_delay(self, speed_percent):
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)
    
    def init_motor(self, direction=1, speed=0.5, pontto=False):
        # pontto motor init is little bit more complicated to not get the 
        # cabel tangled there for it has own init sequence
        if self.name=="pontto":
            print("initing pontto")
            while not self.limit_switch.is_pressed:
                if abs(self.angle) > 360:
                    raise TimeoutError("Motor didn't find init pos")
                self.step(direction=direction, speed=speed)

            print("angle after first init", self.angle)

            if abs(self.angle) > 180:
                time.sleep(1)
                self.move_by_angle(10 * (direction), speed=speed)
                while not self.limit_switch.is_pressed:
                    self.step(direction=-1*direction, speed=speed)
            else: 
                self.move_by_angle(10 * (-1*direction), speed=speed)
                time.sleep(1)
                while not self.limit_switch.is_pressed:
                    self.step(direction=-1*direction, speed=speed)

        else:
            # this one is basic one, mainly used for paaty motor
            while not self.limit_switch.is_pressed:
                if abs(self.angle) > 270:
                    raise TimeoutError("Motor didn't find init pos")
                self.step(direction=direction, speed=speed)

        self.reset_position()
        print(f"Motor {self.name} initialized")

    def step(self, direction=1, speed=0.5):
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
        self.angle = round(self.angle, 3)

    def move_by_angle(self, angle, speed=0.5):
        if abs(angle) > self.angle_limit:
           raise ValueError("Angle must be between -360 and 360 degrees.")

        angle_per_step = 360 / (self.step_per_rev * self.gear_ratio)
        steps = int(angle / angle_per_step)
        direction = 1 if angle < 0 else -1

        for _ in range(abs(steps)):
            # print("curr ", self.angle)
            self.step(direction=direction, speed=speed)

    def move_to_angle(self, target_angle, speed=0.5):
        if abs(target_angle) > self.angle_limit:
            raise ValueError("Target angle must be between -360 and 360 degrees.")

        print("target", target_angle)

        angle_diff = target_angle - self.angle

        print("diff", angle_diff)
        self.move_by_angle(angle_diff, speed=speed)

    def get_steps(self):
        return self.steps

    def get_angle(self):
        return self.angle

    def reset_position(self):
        self.steps = 0
        self.angle = 0

    def button_held(self):
        print("Button held")
        self.init_pos = True

    def button_release(self):
        print("released")
        self.init_pos = False

    def cleanup(self):
        """
        Cleanup method to release GPIO resources.
        """
        self.pulse.close()
        self.direction.close()
        self.limit_switch.close()
        print(f"Motor {self.name} cleaned up")


if __name__ == "__main__":
    motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, name="paaty", gear_ratio=5)
    motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, name="pontto", gear_ratio=5*32/10)
    
    # motor_pontto.move_by_angle(-190, speed=0.1)
    motor_paaty.move_by_angle(-90, speed=0.5)

    motor_paaty.init_motor(direction=-1)
    motor_pontto.init_motor(direction=1, speed=0.1)
