from linear_rail import LinearRail
from spinning_joints import SpinningJoints
import time
import sys
import termios
import tty

from multiprocessing import Process, Queue, Manager
manager = Manager()
shared = manager.Namespace()
shared.theta_1 = 0
shared.theta_2 = 0
shared.delta_r = 0
shared.timer = 0
shared.path_it = 0

from config import *

# Interactive terminal interface to control three motors.
# Function to capture a single key press without requiring ENTER.
def getch():
    if sys.stdin.isatty():
        import termios, tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    else:
        # Non-interactive mode: just read one character.
        return sys.stdin.read(1)

def main():
    # Instantiate three Motor objects with example GPIO pin assignments.
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
        
    motor_rail = LinearRail(shared, 
                                pulse_pin=RAIL_MOTOR_PULSE_PIN, 
                                dir_pin=RAIL_MOTOR_DIR_PIN, 
                                limit_pin=RAIL_MOTOR_SENSOR_PIN, 
                                gear_ratio=RAIL_MOTOR_GEAR_RATIO)

    angles_per_key = 1
    
    print("Keyboard Control for Three Motors")
    print("Controls:")
    print("  Motor paaty: 'w' (up), 's' (down)") # <- TODO has to be checked if direction match the desc
    print("  Motor pontto: 'a' (clockwise), 'd' (counter-clockwise)")
    print("  Motor rail: 'j' (left), 'k' (right)")
    print("  'i' to increase step per key and 'o' to decrease it")
    print("  'p' prints the status for all motors")
    print("  'q' quits the program")
    
    # Set a fixed speed (1 = maximum speed; adjust as needed)
    speed = 0.5
    while True:
        key = getch()
        if key == 'q':
            break
        elif key == 'w':
            motor_paaty.move_by_angle(angle=angles_per_key, speed=speed)
             #motor1.step(steps=step_per_key, direction=1, speed=speed)
            print("Motor 1: stepped clockwise.")
        elif key == 's':
            motor_paaty.move_by_angle(angle=-angles_per_key, speed=speed)
            print("Motor 1: stepped counter-clockwise.")

        elif key == 'a':
            motor_pontto.move_by_angle(angle=angles_per_key, speed=speed)
            print("Motor 2: stepped clockwise.")
        elif key == 'd':
            motor_pontto.move_by_angle(angle=-angles_per_key, speed=speed)
            print("Motor 2: stepped counter-clockwise.")

        elif key == 'j':
            motor_rail.move_by_distance(angle=angles_per_key, speed=speed)
            print("Motor 3: stepped clockwise.")
        elif key == 'k':
            motor_rail.move_by_distance(angle=-angles_per_key, speed=speed)
            print("Motor 3: stepped counter-clockwise.")
            
        elif key == 'p':
            # TODO: FIX THIS AS THE get steps function is gone
            print(f"Motor 1: Steps = {motor_pontto.steps}, Angle = {motor_pontto.angle}°") 
            print(f"Motor 2: Steps = {motor_paaty.steps}, Angle = {motor_paaty.angle}°")
            print(f"Motor 3: Steps = {motor_rail.steps}, Distance = {motor_rail.distance}°")
            print(f"Step per key: {angles_per_key}")

        elif key == 'i':
            angles_per_key += 10
            print(f"Step per key increased to {angles_per_key}")

        elif key == 'o':
            angles_per_key = max(1, angles_per_key - 10)  # Prevent it going below 1
            print(f"Step per key decreased to {angles_per_key}")

        else:
            print("Unrecognized key.")
    
    print("Exiting... Cleaning up.")
    # motor1.cleanup()
    # motor2.cleanup()
    # motor3.cleanup()

if __name__ == "__main__":
    main()
