from gpiozero import DigitalOutputDevice
import time
from gpiozero import Button
from signal import pause

from multiprocessing import Event


class LinearRail:
    def __init__(self, shared, pulse_pin, dir_pin, limit_pin, step_per_rev=1600, gear_ratio=5, pitch=10, min_delay=1e-4, max_delay=1e-3):
        """
        Initialize the stepper motor with the given GPIO pins.

        :param pulse_pin: GPIO pin for sending step pulses.
        :param dir_pin: GPIO pin for setting the direction.
        """
        # asserts:)
        assert isinstance(pulse_pin, int), "Pulse pin must be an integer."
        assert isinstance(dir_pin, int), "Direction pin must be an integer."
        assert isinstance(step_per_rev, int), "Steps per revolution must be an integer."
        assert isinstance(gear_ratio, int), "Gear ratio must be an integer."
        assert isinstance(min_delay, (int, float)), "Minimum delay must be a number."
        assert isinstance(max_delay, (int, float)), "Maximum delay must be a number."
        assert min_delay >= 0, "Minimum delay must be non-negative."
        assert max_delay >= min_delay, "Maximum delay must be greater than or equal to minimum delay."
        
        self.shared = shared

        self.pulse = DigitalOutputDevice(pulse_pin)
        self.direction = DigitalOutputDevice(dir_pin)

        self.limit_event = Event()

        self.limit_switch = Button(limit_pin, pull_up=True, bounce_time=0.0001)
        self.limit_switch.when_held = lambda: (print("Limit switch is pressed"), self.limit_event.set())
        self.limit_switch.when_released = lambda: (print("Limit switch is released"), self.limit_event.clear())
        
        self.limit_switch.hold_time = 0.1
        self.limit_switch.hold_repeat = True

        self.distance = 0

        self.steps = 0
        self.angle = 0
        self.pitch = pitch  # Pitch of the linear rail in mm
        self.step_per_rev = step_per_rev
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.gear_ratio = gear_ratio

        self.stop = self.limit_event.is_set()  # Initialize stop state based on limit switch

    def calc_delay(self, speed_percent):
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)
    
    def init_motor(self, direction=1):
        while not self.limit_event.is_set():
            self.step(direction=direction, speed=0.1)
        self.move_by_angle(90 * (direction * -1), speed=0.5, ignore_limit=True)
        self.reset_position()
        print(f"Motor limit initialized, stop state {self.stop}")

    def step(self, direction=1, speed=0.5, ignore_limit=False):
        if self.limit_event.is_set() and not ignore_limit:
            print("Limit switch is pressed. Cannot move motor.")
            return

        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 (forward) or -1 (backward).")

        self.direction.value = 1 if direction == 1 else 0
        delay = self.calc_delay(speed)

        # print(delay)

        # print("self steps", self.steps)

        self.pulse.on()
        time.sleep(delay)
        self.pulse.off()
        time.sleep(delay)
        self.steps += direction
        self.angle += direction * (360 / (self.step_per_rev * self.gear_ratio))
        self.angle = round(self.angle, 3)
        self.distance += (-1 * direction) * (self.pitch / self.step_per_rev)  # negative because of the flipped direction

        self.shared.delta_r = self.distance

    def move_by_angle(self, angle, speed=0.5, ignore_limit=False):
        # if abs(angle) > 360:
        #    raise ValueError("Angle must be between -360 and 360 degrees.")

        angle_per_step = 360 / (self.step_per_rev * self.gear_ratio)
        steps = int(angle / angle_per_step)
        direction = 1 if angle > 0 else -1

        for _ in range(abs(steps)):
            self.step(direction=direction, speed=speed, ignore_limit=ignore_limit)

    def move_to_angle(self, target_angle, speed=0.5):
        if abs(target_angle) > 360:
            raise ValueError("Target angle must be between -360 and 360 degrees.")

        angle_diff = target_angle - self.angle
        self.move_by_angle(angle_diff, speed=speed)

    def move_by_distance(self, distance, speed=0.5, shared=None):
        """
        Move the linear rail by a specified distance in mm.

        :param distance: Distance to move in mm.
        :param speed: Speed of movement as a percentage (0 to 1).
        """
        if distance == 0:
            return

        steps_per_mm = self.step_per_rev / self.pitch
        steps = int(distance * steps_per_mm)
        direction = 1 if distance < 0 else -1 
        
        # because now init point is on the wrong side our motor works in wrong direction
        # so to get to use positive numbers when going right direction has to be flipped

        for _ in range(abs(steps)):
            self.step(direction=direction, speed=speed)

        if shared is not None:
            shared.delta_r = self.distance
    
    def move_to_distance(self, target_distance, speed=0.5, shared=None):
        """
        Move the linear rail to a specified distance in mm.

        :param target_distance: Target distance in mm.
        :param speed: Speed of movement as a percentage (0 to 1).
        """
        if target_distance < 0:
            raise ValueError("Target distance must be non-negative.")

        # for same reason that direction is flipped in previous function here
        # we need to input negative steps to get correctly current position

        print("self.distance in motor", self.distance)

        current_distance = self.distance
        distance_diff = target_distance - current_distance
        self.move_by_distance(distance_diff, speed=speed, shared=shared)

    def get_steps(self):
        return self.steps

    def get_angle(self):
        return self.angle

    def reset_position(self):
        self.steps = 0
        self.angle = 0

    def button_held(self):
        print("Button held")
        self.stop = True

    def button_release(self):
        print("released")
        self.stop = False

    def cleanup(self):
        """
        Cleanup method to release GPIO resources.
        """
        self.pulse.close()
        self.direction.close()
        self.limit_switch.close()
        print("Motor linear resources cleaned up.")


if __name__ == "__main__":
    from multiprocessing import Process, Queue, Manager
    manager = Manager()
    shared = manager.Namespace()
    motor = LinearRail(pulse_pin=27, shared=shared, dir_pin=4, limit_pin=24, gear_ratio=1)
    # motor.move_by_angle(-720*6, speed=0.5)
    # motor.init_motor(direction=1)
    motor.move_by_distance(-500, speed=1)
    
    motor.move_to_distance(10, speed=0.5)
    print("moved")
    print(motor.steps)
    time.sleep(1)
    motor.move_to_distance(40, speed=0.5)
    print("moved")
    print(motor.steps)
    time.sleep(1)
    motor.move_to_distance(30, speed=0.5)
    print("moved")
    print(motor.steps)
    time.sleep(1)
    motor.move_to_distance(50, speed=0.5)
    print("moved")

