"""
keyboard_control_motors.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Interactive command-line utility to manually jog three axes (two rotary joints
and one linear rail) using keyboard input.  Uses blocking reads from stdin to
capture single-key commands without requiring ENTER.

Controls
--------
  w / s : move paaty joint up/down
  a / d : move pontto joint CCW/CW
  j / k : move linear rail right/left
  i / o : increase / decrease step-per-key increment
  p     : print current position/status of all motors
  q     : quit and exit cleanly

Usage
-----
>>> python keyboard_control_motors.py
"""
import sys
import time
from multiprocessing import Manager
from linear_rail import LinearRail
from spinning_joints import SpinningJoints
from config import (
    PAATY_MOTOR_PULSE_PIN, PAATY_MOTOR_DIR_PIN, PAATY_MOTOR_SENSOR_PIN,
    PAATY_MOTOR_GEAR_RATIO,
    PONTTO_MOTOR_PULSE_PIN, PONTTO_MOTOR_DIR_PIN, PONTTO_MOTOR_SENSOR_PIN,
    PONTTO_MOTOR_GEAR_RATION,
    RAIL_MOTOR_PULSE_PIN, RAIL_MOTOR_DIR_PIN, RAIL_MOTOR_SENSOR_PIN,
    RAIL_MOTOR_GEAR_RATIO
)

# ----------------------------------------------------------------------------
# Utility: read single keypress without ENTER
# ----------------------------------------------------------------------------
def getch() -> str:
    """
    Read one character from stdin immediately (no need to press ENTER).
    Works in a TTY; if not a terminal, falls back to a simple read.
    """
    if sys.stdin.isatty():
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch
    else:
        # Non-interactive fallback
        return sys.stdin.read(1)


# ----------------------------------------------------------------------------
# Main interactive loop
# ----------------------------------------------------------------------------
def main() -> None:
    """
    Instantiate motors and enter a loop to process keyboard commands.
    Exits when 'q' is pressed, cleaning up resources.
    """
    # Create shared namespace for inter-process states (angles, distance)
    manager = Manager()
    shared = manager.Namespace()
    shared.theta_1 = 0.0  # pontto angle
    shared.theta_2 = 0.0  # paaty angle
    shared.delta_r = 0.0  # rail position

    # Instantiate motor controllers
    motor_paaty = SpinningJoints(
        shared,
        pulse_pin=PAATY_MOTOR_PULSE_PIN,
        dir_pin=PAATY_MOTOR_DIR_PIN,
        limit_pin=PAATY_MOTOR_SENSOR_PIN,
        name="paaty",
        gear_ratio=PAATY_MOTOR_GEAR_RATIO
    )
    motor_pontto = SpinningJoints(
        shared,
        pulse_pin=PONTTO_MOTOR_PULSE_PIN,
        dir_pin=PONTTO_MOTOR_DIR_PIN,
        limit_pin=PONTTO_MOTOR_SENSOR_PIN,
        name="pontto",
        gear_ratio=PONTTO_MOTOR_GEAR_RATION
    )
    motor_rail = LinearRail(
        shared,
        pulse_pin=RAIL_MOTOR_PULSE_PIN,
        dir_pin=RAIL_MOTOR_DIR_PIN,
        limit_pin=RAIL_MOTOR_SENSOR_PIN,
        gear_ratio=RAIL_MOTOR_GEAR_RATIO
    )

    # Step amount per keypress (degrees for joints, mm for rail)
    angles_per_key = 1
    # Constant movement speed (0 to 1)
    speed = 0.5

    # Print help
    print("Keyboard Control for Three Motors")
    print("Controls:")
    print("  w / s : paaty joint up/down")
    print("  a / d : pontto joint CCW/CW")
    print("  j / k : rail right/left")
    print("  i / o : increase/decrease step size")
    print("  p     : print motor statuses")
    print("  q     : quit")

    # Interactive loop
    while True:
        key = getch()
        if key == 'q':
            # Exit loop
            break
        elif key == 'w':
            motor_paaty.move_by_angle(angles_per_key, speed)
            print(f"paaty ↑ +{angles_per_key}° → {motor_paaty.angle:.1f}°")
        elif key == 's':
            motor_paaty.move_by_angle(-angles_per_key, speed)
            print(f"paaty ↓ -{angles_per_key}° → {motor_paaty.angle:.1f}°")
        elif key == 'a':
            motor_pontto.move_by_angle(angles_per_key, speed)
            print(f"pontto CCW +{angles_per_key}° → {motor_pontto.angle:.1f}°")
        elif key == 'd':
            motor_pontto.move_by_angle(-angles_per_key, speed)
            print(f"pontto CW  -{angles_per_key}° → {motor_pontto.angle:.1f}°")
        elif key == 'j':
            motor_rail.move_by_distance(angles_per_key, speed)
            print(f"rail → +{angles_per_key} mm → {motor_rail.distance:.1f} mm")
        elif key == 'k':
            motor_rail.move_by_distance(-angles_per_key, speed)
            print(f"rail ← -{angles_per_key} mm → {motor_rail.distance:.1f} mm")
        elif key == 'p':
            # Print detailed status of all axes
            print("Status:")
            print(f"  pontto: steps={motor_pontto.steps}, angle={motor_pontto.angle:.1f}°")
            print(f"  paaty : steps={motor_paaty.steps}, angle={motor_paaty.angle:.1f}°")
            print(f"  rail  : steps={motor_rail.steps}, distance={motor_rail.distance:.1f} mm")
            print(f"  step size = {angles_per_key}")
        elif key == 'i':
            angles_per_key += 1
            print(f"Step size ↑ to {angles_per_key}")
        elif key == 'o':
            angles_per_key = max(1, angles_per_key - 1)
            print(f"Step size ↓ to {angles_per_key}")
        else:
            print(f"Unrecognized key: '{key}'")

    # Cleanup on exit
    print("Exiting... cleaning up GPIO resources.")
    motor_paaty.cleanup()
    motor_pontto.cleanup()
    motor_rail.cleanup()


if __name__ == "__main__":
    main()
