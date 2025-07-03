#!/usr/bin/python3
from flask import Flask, render_template_string, request, jsonify, redirect, url_for, render_template
import plotly.graph_objs as go
import plotly.io as pio
import subprocess
import time
import select
import os
import sys
import threading
from multiprocessing import Process

from spinning_joints import SpinningJoints
from linear_rail import LinearRail
from arm import Arm
import cooling

app = Flask(__name__)

# Global variable for the interactive (manual control) session.
global_wsd_proc = None

arm = Arm()

# Start the cooling controller in its own thread.
def start_cooling_thread():
    print("starting cooling")
    controller = cooling.FanController(fan_pin=18, min_temp=20, max_temp=45)
    controller.run(verbal=False, interval=0.1)
    print("started cooling")

print("on own thread")
cooling_thread = threading.Thread(target=start_cooling_thread, daemon=True)
cooling_thread.start()

# Endpoint to serve the latest cooling reading as JSON.
@app.route('/cooling_info')
def cooling_info():
    try:
        reading = cooling.latest_reading
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


@app.route('/')
def index():
    stop_arm()
    return render_template('index.html')

# idea how to make robot plot in play
# def index():
#     # Generate a 3D scatter
#     import numpy as np
#     t = np.linspace(0, 4*np.pi, 100)
#     fig = go.Figure(data=[go.Scatter3d(
#         x=np.sin(t), y=np.cos(t), z=t,
#         mode='markers+lines'
#     )])
#     # Get the HTML <div> for the plot
#     plot_div = pio.to_html(fig, full_html=False)
# 
#     return render_template_string('''
#       <h1>Interactive 3D Plot</h1>
#       {{ plot_div|safe }}
#     ''', plot_div=plot_div)


arm_process = None

def start_arm(func, args):
    global arm_process, arm
    if arm_process is None or not arm_process.is_alive():
        arm_process = Process(target=func, args=(args,))
        arm_process.start()


def stop_arm():
    global arm_process
    print("Stopping arm process...")
    if arm_process and arm_process.is_alive():
        arm_process.terminate()
        arm_process.join()
        print("Arm process stopped.")
        arm_process = None


def is_arm_running():
    return arm_process is not None and arm_process.is_alive()


@app.route("/init")
def init():
    if not is_arm_running():
        start_arm()
    return render_template("init.html", running=is_arm_running())


@app.route("/stop")
def stop():
    stop_arm()
    return redirect(url_for("index"))

@app.route('/play')
def play_path():
    global arm
    arm.init()
    arm.init_path()

    while True:
        arm.move()
        time.sleep(0.1)

# You can also protect these with a threading.Lock if you worry about concurrent requests:
motor_lock = threading.Lock()

angles_per_key = 1

@app.route('/manual')
def manual_control():
    global angles_per_key
    angles_per_key = 1
    return render_template('manual_control.html')

@app.route('/move_arm')
def move_arm():
    global arm, angles_per_key, motor_lock
    cmd = request.args.get('cmd')
    response = ""
    status = "ok"

    if is_arm_running():
        arm.stop()
        
        
    with motor_lock:
        if cmd == 'motor_paaty_up':
            start_arm(arm.motor_paaty.move_by_angle, (angles_per_key, 0.5))
            # arm.motor_paaty.move_by_angle(angle=angles_per_key, speed=0.5)
            response = "Motor paaty moved up"
        elif cmd == 'motor_paaty_down':
            start_arm(arm.motor_paaty.move_by_angle, (-angles_per_key, 0.5))
            # arm.motor_paaty.move_by_angle(angle=-angles_per_key, speed=0.5)
            response = "Motor paaty moved down"
        elif cmd == 'motor_pontto_ccw':
            start_arm(arm.motor_pontto.move_by_angle, (angles_per_key, 0.5))
            # arm.motor_pontto.move_by_angle(angle=angles_per_key, speed=0.5)
            response = "Motor pontto moved ccw"
        elif cmd == 'motor_pontto_cw':
            start_arm(arm.motor_pontto.move_by_angle, (-angles_per_key, 0.5))
            # arm.motor_pontto.move_by_angle(angle=-angles_per_key, speed=0.5)
            response = "Motor pontto moved cw"
        elif cmd == 'motor_rail_right':
            arm.motor_rail.move_by_distance(distance=angles_per_key, speed=0.5)
            response = "Motor rail moved right"
        elif cmd == 'motor_rail_left':
            arm.motor_rail.move_by_distance(distance=-angles_per_key, speed=0.5)
            response = "Motor rail moved left"
        elif cmd == 'pl':
            angles_per_key += 10
            response = f"Step per key increased to {angles_per_key}"
        elif cmd == 'mn':
            angles_per_key = max(1, angles_per_key - 10)
            response = f"Step per key decreased to {angles_per_key}"
        elif cmd == 'set_step_size':
            try:
                new_value = int(request.args.get('to'))
                if new_value < 1:
                    raise ValueError("Step size must be positive")
                angles_per_key = new_value
                response = f"Step size set to {angles_per_key}"
            except Exception as e:
                status = "error"
                response = str(e)
        else:
            status = "error"
            response = "Unknown command"
    return jsonify({"status": status, "message": response, "step_size": angles_per_key})

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
    global arm
    limit_sensor = arm.motor_rail.limit_switch.is_pressed
    if limit_sensor:
        return jsonify({'limit_sensor_pressed': True})
    else:
        return jsonify({'limit_sensor_pressed': False})


@app.route('/check_pontto_induction_sensor')
def check_pontto_induction_sensor():
    global arm
    limit_sensor = arm.motor_pontto.limit_switch.is_pressed
    if limit_sensor:
        return jsonify({'pontto_induction_sensor_pressed': True})
    else:
        return jsonify({'pontto_induction_sensor_pressed': False})


@app.route('/check_paaty_induction_sensor')
def check_paaty_induction_sensor():
    global arm
    limit_sensor = arm.motor_paaty.limit_switch.is_pressed
    if limit_sensor:
        return jsonify({'paaty_induction_sensor_pressed': True})
    else:
        return jsonify({'paaty_induction_sensor_pressed': False})


@app.route('/done')
def done():
    return render_template('done.html')




if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000) #, debug=True, use_reloader=False)