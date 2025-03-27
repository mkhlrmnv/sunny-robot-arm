from gpiozero import DigitalOutputDevice
import time

class Motor:
    def __init__(self, pulse_pin, dir_pin, step_per_rev=1600, gear_ratio=5, min_delay=1e-5, max_delay=1):
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
        
        self.pulse = DigitalOutputDevice(pulse_pin)
        self.direction = DigitalOutputDevice(dir_pin)
        self.steps = 0
        self.angle = 0
        self.step_per_rev = step_per_rev
        self.min_delay = min_delay
        self.max_delay = max_delay
        self.gear_ratio = gear_ratio

    def calc_delay(self, speed_percent):
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)

    def step(self, steps=1, direction=1, speed=0.5):
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 (forward) or -1 (backward).")

        self.direction.value = 1 if direction == 1 else 0
        delay = self.calc_delay(speed)

        for _ in range(abs(steps)):
            self.pulse.on()
            time.sleep(1e-5)
            self.pulse.off()
            time.sleep(delay)
            self.steps += direction
            self.angle += direction * (360 / (self.step_per_rev * self.gear_ratio))
            self.angle = round(self.angle, 3)

    def move_by_angle(self, angle, speed=0.5):
        if abs(angle) > 360:
            raise ValueError("Angle must be between -360 and 360 degrees.")

        angle_per_step = 360 / (self.step_per_rev * self.gear_ratio)
        steps = int(angle / angle_per_step)
        direction = 1 if angle > 0 else -1
        self.step(steps=abs(steps), direction=direction, speed=speed)

    def move_to_angle(self, target_angle, speed=0.5):
        if abs(target_angle) > 360:
            raise ValueError("Target angle must be between -360 and 360 degrees.")

        angle_diff = target_angle - self.angle
        self.move_by_angle(angle_diff, speed=speed)

    def get_steps(self):
        return self.steps

    def get_angle(self):
        return self.angle

    def reset_position(self):
        self.steps = 0
        self.angle = 0


if __name__ == "__main__":
    stepper = Motor(pulse_pin=13, dir_pin=27)
    stepper.move_by_angle(180, speed=1)
