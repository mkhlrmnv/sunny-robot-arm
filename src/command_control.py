from motor import Motor

def main():
    # Instantiate three motors with example GPIO pin assignments.
    motor2 = Motor(pulse_pin=13, dir_pin=26, limit_pin=23, gear_ratio=5)
    motor3 = Motor(pulse_pin=27, dir_pin=4, limit_pin=22, gear_ratio=1)
    motor1 = Motor(pulse_pin=20, dir_pin=19, limit_pin=12, gear_ratio=5)
    
    motors = {1: motor1, 2: motor2, 3: motor3}
    
    print("Three Motor Control Interface")
    print("Commands available for each motor:")
    print("  1. move_by_angle - Move motor by a relative angle (in degrees).")
    print("  2. move_to_angle - Move motor to an absolute angle (in degrees).")
    print("  3. step        - Move motor a given number of steps.")
    print("  4. get_angle   - Display current angle.")
    print("  5. get_steps   - Display total step count.")
    print("  6. reset       - Reset motor position (steps and angle).")
    print("  7. back        - Return to motor selection.")
    print("Enter 'q' at any prompt to quit.")

    while True:
        selection = input("\nSelect a motor to control (1, 2, 3) or 'q' to quit: ").strip()
        if selection.lower() == 'q':
            break
        try:
            motor_num = int(selection)
            if motor_num not in motors:
                print("Invalid motor number. Please choose 1, 2, or 3.")
                continue
        except ValueError:
            print("Invalid input. Please enter a number (1, 2, 3) or 'q'.")
            continue

        selected_motor = motors[motor_num]
        print(f"\nMotor {motor_num} selected.")
        
        while True:
            print("\nCommands for Motor", motor_num)
            print(" 1: move_by_angle")
            print(" 2: move_to_angle")
            print(" 3: step")
            print(" 4: get_angle")
            print(" 5: get_steps")
            print(" 6: reset")
            print(" 7: back to motor selection")
            cmd = input("Enter command number (or 'q' to quit): ").strip()
            if cmd.lower() == 'q':
                return
            if cmd == "7":
                break

            try:
                if cmd == "1":
                    angle = float(input("Enter angle to move by (degrees, -360 to 360): "))
                    speed = float(input("Enter speed (0 to 1): "))
                    selected_motor.move_by_angle(angle, speed=speed)
                    print(f"Moved motor by {angle} degrees.")

                elif cmd == "2":
                    target_angle = float(input("Enter target angle (degrees, -360 to 360): "))
                    speed = float(input("Enter speed (0 to 1): "))
                    selected_motor.move_to_angle(target_angle, speed=speed)
                    print(f"Moved motor to {target_angle} degrees.")

                elif cmd == "3":
                    steps = int(input("Enter number of steps: "))
                    direction = int(input("Enter direction (1 for forward, -1 for backward): "))
                    speed = float(input("Enter speed (0 to 1): "))
                    selected_motor.step(steps=abs(steps), direction=direction, speed=speed)
                    print(f"Stepped {steps} steps in direction {direction}.")

                elif cmd == "4":
                    print("Current angle:", selected_motor.get_angle(), "degrees")
                
                elif cmd == "5":
                    print("Total steps taken:", selected_motor.get_steps())

                elif cmd == "6":
                    selected_motor.reset_position()
                    print("Motor position reset to 0.")
                
                else:
                    print("Invalid command. Please choose a valid option.")
            except Exception as e:
                print("An error occurred:", e)
    
    print("\nCleaning up motors...")
    for m in motors.values():
        m.cleanup()
    print("Exiting program.")

if __name__ == "__main__":
    main()