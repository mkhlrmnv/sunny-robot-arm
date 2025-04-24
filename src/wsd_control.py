from src.linear_rail import LinearRail
from src.spinning_joints import SpinningJoints
import time
import sys
import termios
import tty

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
    motor1 = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=12, gear_ratio=5, angle_limit=360)
    motor2 = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=5, angle_limit=360) 
    motor3 = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=23, gear_ratio=1)   # <- linear rail

    angles_per_key = 1
    
    print("Keyboard Control for Three Motors")
    print("Controls:")
    print("  Motor 1: 'w' (clockwise), 's' (counter-clockwise)")
    print("  Motor 2: 'a' (clockwise), 'd' (counter-clockwise)")
    print("  Motor 3: 'j' (clockwise), 'k' (counter-clockwise)")
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
            motor1.move_by_angle(angle=angles_per_key, speed=speed)
             #motor1.step(steps=step_per_key, direction=1, speed=speed)
            print("Motor 1: stepped clockwise.")
        elif key == 's':
            motor1.move_by_angle(angle=-angles_per_key, speed=speed)
            print("Motor 1: stepped counter-clockwise.")

        elif key == 'a':
            motor2.move_by_angle(angle=angles_per_key, speed=speed)
            print("Motor 2: stepped clockwise.")
        elif key == 'd':
            motor2.move_by_angle(angle=-angles_per_key, speed=speed)
            print("Motor 2: stepped counter-clockwise.")

        elif key == 'j':
            motor3.move_by_angle(angle=angles_per_key, speed=speed)
            print("Motor 3: stepped clockwise.")
        elif key == 'k':
            motor3.move_by_angle(angle=-angles_per_key, speed=speed)
            print("Motor 3: stepped counter-clockwise.")
            
        elif key == 'p':
            print(f"Motor 1: Steps = {motor1.get_steps()}, Angle = {motor1.get_angle()}°")
            print(f"Motor 2: Steps = {motor2.get_steps()}, Angle = {motor2.get_angle()}°")
            print(f"Motor 3: Steps = {motor3.get_steps()}, Angle = {motor3.get_angle()}°")
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
