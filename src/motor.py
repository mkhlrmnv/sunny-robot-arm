# import RPi.GPIO as GPIO
import time

class Motor:
    def __init__(self, pulse_pin, dir_pin, step_per_rev=1600, min_delay=1e-5, max_delay=1):
        """
        Initialize the stepper motor with the given GPIO pins.
        
        :param pulse_pin: GPIO pin for sending step pulses.
        :param dir_pin: GPIO pin for setting the direction.
        """
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.steps = 0  # Tracks steps taken
        self.angle = 0  # Tracks angle
        self.step_per_rev = step_per_rev
        self.min_delay = min_delay
        self.max_delay = max_delay

        # GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        # GPIO.setup(self.pulse_pin, GPIO.OUT)
        # GPIO.setup(self.dir_pin, GPIO.OUT)

    def calc_delay(self, speed_percent):
        """
        Calculate the delay between steps based on the speed percentage.
        
        :param speed_percent: Speed percentage (0 to 1).
        :return: Delay between steps.
        """
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)

    def step(self, steps=1, direction=1, speed=0.5):
        """
        Move the stepper motor a given number of steps in the specified direction.
        
        :param steps: Number of steps to move.
        :param direction: 1 for forward, -1 for backward.
        :param delay: Time delay between steps (adjust for speed control).
        """
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 (forward) or -1 (backward).")
        
        # GPIO.output(self.dir_pin, GPIO.HIGH if direction == 1 else GPIO.LOW)

        for _ in range(abs(steps)):
            # GPIO.output(self.pulse_pin, GPIO.HIGH)
            time.sleep(1e-5) # Pulse width of 100KHz
            # GPIO.output(self.pulse_pin, GPIO.LOW)
            time.sleep(self.calc_delay(speed)) # Delay between steps
            self.steps += direction
            self.angle += direction * (360 / self.step_per_rev)

    def move_by_angle(self, angle, speed=0.5):
        """
        Move the stepper motor by a specified angle.
        
        :param angle: Angle in degrees to move.
        """
        if abs(angle) < 360:
            raise ValueError("Angle must be between -360 and 360 degrees.")
        
        angle_per_step = 360 / self.step_per_rev
        steps = int(angle / angle_per_step)
        direction = 1 if angle > 0 else -1
        self.step(steps=abs(steps), direction=direction, speed=speed)
    
    def get_steps(self):
        """
        Get the current step count
        """
        return self.steps

    def get_angle(self):
        """
        Get the current angle of the motor.
        """
        return self.angle

    def reset_position(self):
        """
        Reset the step and angle counter to zero.
        """
        self.steps = 0
        self.angle = 0

    def cleanup(self):
        """
        Clean up GPIO settings.
        """
        # GPIO.cleanup()


if __name__ == "__main__":
    stepper = Motor(pulse_pin=17, dir_pin=27)
    stepper.step(steps=100, direction=1, delay=0.002)  # Move 100 steps forward
    print("Current Position:", stepper.get_position())
    stepper.cleanup()