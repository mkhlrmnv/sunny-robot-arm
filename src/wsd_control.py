from motor import Motor
import time
import sys
import termios
import tty

# Interactive terminal interface to control three motors.
# Function to capture a single key press without requiring ENTER.
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    # Instantiate three Motor objects with example GPIO pin assignments.
    motor1 = Motor(pulse_pin=17, dir_pin=27)
    motor2 = Motor(pulse_pin=22, dir_pin=23)
    motor3 = Motor(pulse_pin=24, dir_pin=25)
    
    print("Keyboard Control for Three Motors")
    print("Controls:")
    print("  Motor 1: 'w' (clockwise), 's' (counter-clockwise)")
    print("  Motor 2: 'a' (clockwise), 'd' (counter-clockwise)")
    print("  Motor 3: 'j' (clockwise), 'k' (counter-clockwise)")
    print("  'p' prints the status for all motors")
    print("  'q' quits the program")
    
    # Set a fixed speed (1 = maximum speed; adjust as needed)
    speed = 1
    while True:
        key = getch()
        if key == 'q':
            break
        elif key == 'w':
            motor1.step(steps=1, direction=1, speed=speed)
            print("Motor 1: stepped clockwise.")
        elif key == 's':
            motor1.step(steps=1, direction=-1, speed=speed)
            print("Motor 1: stepped counter-clockwise.")

        elif key == 'a':
            motor2.step(steps=1, direction=1, speed=speed)
            print("Motor 2: stepped clockwise.")
        elif key == 'd':
            motor2.step(steps=1, direction=-1, speed=speed)
            print("Motor 2: stepped counter-clockwise.")

        elif key == 'j':
            motor3.step(steps=1, direction=1, speed=speed)
            print("Motor 3: stepped clockwise.")
        elif key == 'k':
            motor3.step(steps=1, direction=-1, speed=speed)
            print("Motor 3: stepped counter-clockwise.")
            
        elif key == 'p':
            print(f"Motor 1: Steps = {motor1.get_steps()}, Angle = {motor1.get_angle()}°")
            print(f"Motor 2: Steps = {motor2.get_steps()}, Angle = {motor2.get_angle()}°")
            print(f"Motor 3: Steps = {motor3.get_steps()}, Angle = {motor3.get_angle()}°")
        else:
            print("Unrecognized key.")
    
    print("Exiting... Cleaning up.")
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()

if __name__ == "__main__":
    main()