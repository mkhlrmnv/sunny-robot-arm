#!/usr/bin/python3
from flask import Flask, render_template_string, request, jsonify, redirect, url_for, render_template
import subprocess
import time
import select
import os
import sys
import threading

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


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/init')
def init_motors():
    return render_template('init_motors.html')


@app.route('/play')
def play_path():
    return render_template('play_path.html')


@app.route('/manual')
def manual_control():
    global global_wsd_proc
    if global_wsd_proc is None or global_wsd_proc.poll() is not None:
        start_wsd_control()
        time.sleep(1)
        # Flush any initial output from the process.
        while True:
            rlist, _, _ = select.select([global_wsd_proc.stdout], [], [], 0.1)
            if not rlist:
                break
            dummy = global_wsd_proc.stdout.readline()
    return render_template('manual_control.html')


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


@app.route('/send_char')
def send_char():
    global global_wsd_proc
    cmd = request.args.get('cmd')
    # If process isn't started or has terminated, restart it.
    if global_wsd_proc is None or global_wsd_proc.poll() is not None:
        print("Interactive process not active. Restarting...")
        start_wsd_control()
        time.sleep(1)
    
    try:
        print("Sending command:", cmd)
        global_wsd_proc.stdin.write(cmd)
        global_wsd_proc.stdin.flush()
    except BrokenPipeError:
        print("Broken pipe detected. Restarting interactive process.")
        start_wsd_control()
        return "Interactive process restarted due to broken pipe."
    
    # Allow a brief moment for the process to respond.
    time.sleep(0.1)
    output = ""
    # Use select to check if there's data to read.
    rlist, _, _ = select.select([global_wsd_proc.stdout], [], [], 0.1)
    if rlist:
        # Read up to 1024 bytes to avoid blocking indefinitely.
        output = global_wsd_proc.stdout.read(1024)
    if output:
        print("Process output:", output)
        return output
    return "No output from process"


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)