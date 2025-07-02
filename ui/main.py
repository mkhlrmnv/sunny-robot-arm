#!/usr/bin/python3
from flask import Flask, render_template_string, request, jsonify, redirect, url_for, render_template
import subprocess
import time
import select
import os
import sys
import threading

from src.spinning_joints import SpinningJoints
from src.linear_rail import LinearRail
from src.arm import Arm

app = Flask(__name__)

# Global variable for the interactive (manual control) session.
global_wsd_proc = None

# Add the absolute path to the 'src' directory to sys.path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_path)

# Now you can import the cooling module
# import cooling


# Start the cooling controller in its own thread.
def start_cooling_thread():
    print("starting cooling")
    # controller = cooling.FanController(fan_pin=18, min_temp=20, max_temp=45)
    # controller.run(verbal=False, interval=0.1)
    print("started cooling")

print("on own thread")
cooling_thread = threading.Thread(target=start_cooling_thread, daemon=True)
cooling_thread.start()


# Endpoint to serve the latest cooling reading as JSON.
@app.route('/cooling_info')
def cooling_info():
    try:
        reading = {
                    "temperature": 68,
                    "fan_speed": 69
                }
        # reading = cooling.latest_reading
    except AttributeError:
        reading = {"temperature": "--", "fan_speed": "--"}
    print(reading)
    return jsonify(reading)


def start_wsd_control():
    global global_wsd_proc
    print("Starting local wsd_control process...")
    # Build absolute path to your script in the src folder.
    # Get the absolute path to the directory containing this script
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Build path to the 'src/wsd_control.py' file relative to current script
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    script_path = os.path.join(current_dir, '..', 'src', 'wsd_control.py')
    script_path = os.path.abspath(script_path)  # Resolve to full absolute path
    # Use unbuffered mode with -u flag.
    command = f"python3 -u {script_path}"
    
    try:
        global_wsd_proc = subprocess.Popen(
            command, shell=True,
            cwd=base_dir,  # Set working directory to the src folder.
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        # Set stdout to non-blocking mode.
        os.set_blocking(global_wsd_proc.stdout.fileno(), False)
    except Exception as e:
        print("Error starting process:", e)
        return

    time.sleep(1)
    if global_wsd_proc.poll() is not None:
        # Process has terminated; capture and print error output.
        err = global_wsd_proc.stderr.read()
        print("wsd_control.py did not start successfully.")
        print("Error output:", err)
    else:
        print("wsd_control.py started successfully.")


def run_non_interactive_command(cmd):
    """Execute a one-off command locally and return its output and error."""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr


def cleanup_motors():
    """Call .cleanup() on any motor instance that exists, then zero them out."""
    global motor_paaty, motor_pontto, motor_rail

    if motor_paaty is not None:
        motor_paaty.cleanup()
    if motor_pontto is not None:
        motor_pontto.cleanup()
    if motor_rail is not None:
        motor_rail.cleanup()

    motor_paaty = motor_pontto = motor_rail = None


@app.route('/')
def index():
    global motor_paaty, motor_pontto, motor_rail
    if motor_paaty is not None or motor_pontto is not None or motor_rail is not None:
        cleanup_motors()
    return render_template('index.html')


@app.route('/init')
def init_motors():
    global motor_paaty, motor_pontto, motor_rail, angles_per_key
    motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, name="paaty", gear_ratio=5)
    motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, name="pontto", gear_ratio=5*32/10)
    motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)

    motor_paaty.init_motor(direction=-1)
    motor_pontto.init_motor(direction=1, speed=0.1)
    motor_rail.init_motor(direction=1)

    cleanup_motors()

@app.route('/play')
def play_path():
    arm = Arm()
    arm.init()
    arm.init_path()

    while True:
        arm.move()
        time.sleep(0.1)

motor_paaty = None
motor_pontto = None
motor_rail = None
angles_per_key = 1

# You can also protect these with a threading.Lock if you worry about concurrent requests:
motor_lock = threading.Lock()

@app.route('/manual')
def manual_control():
    global motor_paaty, motor_pontto, motor_rail, angles_per_key
    motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, name="paaty", gear_ratio=5)
    motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, name="pontto", gear_ratio=5*32/10)
    motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)
    angles_per_key = 1
    return render_template('manual_control.html')

@app.route('/send_char')
def send_char():
    global motor_paaty, motor_pontto, motor_rail
    cmd = request.args.get('cmd')
    response = ""

    with motor_lock:
        if cmd == 'w':
            motor_paaty.move_by_angle(angle=angles_per_key, speed=0.5)
            response = "Motor 1 ⬅️"
        elif cmd == 's':
            motor_paaty.move_by_angle(angle=-angles_per_key, speed=0.5)
            response = "Motor 1 ➡️"
        elif cmd == 'a':
            motor_pontto.move_by_angle(angle=angles_per_key, speed=0.5)
            response = "Motor 2 ⬅️"
        elif cmd == 'd':
            motor_pontto.move_by_angle(angle=-angles_per_key, speed=0.5)
            response = "Motor 2 ➡️"
        elif cmd == 'j':
            motor_rail.move_by_distance(distance=angles_per_key, speed=0.5)
            response = "Motor 3 ⬅️"
        elif cmd == 'k':
            motor_rail.move_by_distance(distance=-angles_per_key, speed=0.5)
            response = "Motor 3 ➡️"
        elif cmd == 'i':
            angles_per_key += 10
            response = f"Step per key increased to {angles_per_key}"
        elif cmd == 'o':
            angles_per_key = max(1, angles_per_key - 10)
            response = f"Step per key decreased to {angles_per_key}"
        else:
            response = "Unknown command"

    return response

@app.route('/unplug')
def unplug_step():
    return render_template('unplug.html')


@app.route('/unplug_done')
def unplug_done():
    return redirect(url_for('right_sensor_test'))


@app.route('/right_sensor_test')
def right_sensor_test():
    return render_template('right_sensor_test.html')


@app.route('/left_sensor_test')
def left_sensor_test():
    return render_template('left_sensor_test.html')


@app.route('/pontto_induction_sensor_test')
def pontto_induction_sensor_test():
    return render_template('pontto_induction_sensor_test.html')


@app.route('/paaty_induction_sensor_test')
def paaty_induction_sensor_test():
    return render_template('paaty_induction_sensor_test.html')


@app.route('/check_limit_sensor')
def check_limit_sensors():
    limit_sensor = True
    # TODO: putt actual sensor check here.
    if limit_sensor:
        return jsonify({'limit_sensor_pressed': True})
    else:
        return jsonify({'limit_sensor_pressed': False})


@app.route('/check_pontto_induction_sensor')
def check_pontto_induction_sensor():
    limit_sensor = True
    # TODO: putt actual sensor check here.
    if limit_sensor:
        return jsonify({'pontto_induction_sensor_pressed': True})
    else:
        return jsonify({'pontto_induction_sensor_pressed': False})


@app.route('/check_paaty_induction_sensor')
def check_paaty_induction_sensor():
    limit_sensor = True
    # TODO: putt actual sensor check here.
    if limit_sensor:
        return jsonify({'paaty_induction_sensor_pressed': True})
    else:
        return jsonify({'paaty_induction_sensor_pressed': False})


@app.route('/done')
def done():
    return render_template('done.html')



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)