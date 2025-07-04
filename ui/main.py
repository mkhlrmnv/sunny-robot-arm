#!/usr/bin/python3
from flask import Flask, render_template_string, request, jsonify, redirect, url_for, render_template
import plotly.graph_objs as go
import plotly.io as pio
import subprocess
import time, queue as _queue
import select
import os
import sys
import threading
from multiprocessing import Process, Queue

from spinning_joints import SpinningJoints
from linear_rail import LinearRail
from arm import Arm
import cooling
from helper import forward_kinematics

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
        reading = {
                    "temperature": round(69, 2),
                    "fan_speed": round(69 * 100, 0)
                }
        reading = cooling.latest_reading
    except AttributeError:
        reading = {"temperature": "--", "fan_speed": "--"}
    print(reading)
    return jsonify(reading)


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
        arm_process = Process(target=func, args=args)
        arm_process.start()


def start_arm_and_wait(func, args):
    global arm_process
    if arm_process is None or not arm_process.is_alive():
        arm_process = Process(target=func, args=args)
        arm_process.start()
        arm_process.join()


@app.route('/status')
def status():
    running = is_arm_running()
    return jsonify({"running": running})


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
    global arm
    if not is_arm_running():
        start_arm(arm.init, ())
    return render_template("init.html", running=is_arm_running())


@app.route("/stop")
def stop():
    stop_arm()
    return redirect(url_for("index"))

play_process = None
message_queue = Queue()


# ─── Routes ──────────────────────────────────────────────────────────────────
@app.route('/play')
def play_path():
    global arm
    print("Starting play_path")
    start_arm_and_wait(arm.init, ())
    arm.init_path()
    start_arm(arm.move, ())
    return render_template('play.html')  # your SSE+plot template


@app.route('/points')
def points():
    global arm
    
    # get your live joint values from arm
    theta1 = arm.motor_pontto.angle
    theta2 = arm.motor_paaty.angle
    delta_r = arm.linear_rail.distance

    print("theta1:", arm.theta_1, "theta2:", theta2, "delta_r:", delta_r)

    pts = forward_kinematics(theta1, theta2, delta_r)
    # pts is an (N×3) numpy array
    return jsonify(pts.tolist())


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
        stop_arm()

    with motor_lock:
        # Manual control stuff -> move up / down, etc.
        if cmd == 'motor_paaty_up':
            start_arm(arm.motor_paaty.move_by_angle, (angles_per_key, 0.5))
            response = "Motor paaty moving up"
        elif cmd == 'motor_paaty_down':
            start_arm(arm.motor_paaty.move_by_angle, (-angles_per_key, 0.5))
            response = "Motor paaty moving down"
        elif cmd == 'motor_pontto_ccw':
            start_arm(arm.motor_pontto.move_by_angle, (angles_per_key, 0.5))
            response = "Motor pontto moving ccw"
        elif cmd == 'motor_pontto_cw':
            start_arm(arm.motor_pontto.move_by_angle, (-angles_per_key, 0.5))
            response = "Motor pontto moving cw"
        elif cmd == 'motor_rail_right':
            start_arm(arm.motor_rail.move_by_distance, (angles_per_key, 0.5))
            response = "Motor rail moving right"
        elif cmd == 'motor_rail_left':
            start_arm(arm.motor_rail.move_by_distance, (-angles_per_key, 0.5))
            response = "Motor rail moving left"
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

        # Motor control API stuff to move by / to angles
        elif cmd == 'by_angle':
            motor = request.args.get('motor')
            angle = float(request.args.get('angle'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))
            m = getattr(arm, f"motor_{motor}")
            if check:
                if m == "motor_paaty":
                    arm._check_if_hypothetical_safe('theta_1', arm.theta_1 + angle)
                elif m == "motor_pontto":
                    arm._check_if_hypothetical_safe('theta_2', arm.theta_2 + angle)
            start_arm(m.move_by_angle, (angle, speed))
            print(arm.theta_2)
            response = f"{motor} moved by {angle}° at speed {speed} (safety={check})"

        elif cmd == 'to_angle':
            motor = request.args.get('motor')
            angle = float(request.args.get('angle'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))
            m = getattr(arm, f"motor_{motor}")
            if check:
                arm._check_if_hypothetical_safe(motor, angle)
            start_arm(m.move_to_angle, (angle, speed, check))
            response = f"{motor} moved to {angle}° at speed {speed} (safety={check})"

        elif cmd == 'by_distance':
            dist = float(request.args.get('dist'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))
            start_arm(arm.motor_rail.move_by_distance, (dist, speed, check))
            response = f"Rail moved by {dist} at speed {speed} (safety={check})"

        elif cmd == 'to_distance':
            dist = float(request.args.get('dist'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))
            start_arm(arm.motor_rail.move_to_distance, (dist, speed, check))
            response = f"Rail moved to {dist} at speed {speed} (safety={check})"

        elif cmd == 'to_point':
            x = float(request.args.get('x'))
            y = float(request.args.get('y'))
            z = float(request.args.get('z'))
            check = bool(int(request.args.get('check_safety', 1)))
            speed_rail = float(request.args.get('speed_rail', 0.5))
            speed_joints = float(request.args.get('speed_joints', 0.5))
            start_arm(arm.move_to_point, (x, y, z, check, speed_rail, speed_joints))
            response = f"Moved to point ({x},{y},{z}) at speeds rail:{speed_rail}, joints:{speed_joints} (safety={check})"

        elif cmd == 'to_angles':
            theta_1 = float(request.args.get('theta_1'))
            theta_2 = float(request.args.get('theta_2'))
            delta_r = float(request.args.get('delta_r'))
            check = bool(int(request.args.get('check_safety', 1)))
            speed_rail = float(request.args.get('speed_rail', 0.5))
            speed_joints = float(request.args.get('speed_joints', 0.5))
            start_arm(arm.move_to_angles, (theta_1, theta_2, delta_r, check, speed_rail, speed_joints))
            response = f"Moved to angles θ1:{theta_1}, θ2:{theta_2}, Δr:{delta_r} at speeds rail:{speed_rail}, joints:{speed_joints} (safety={check})"

        else:
            status = "error"
            response = "Unknown command"

    print(response)
    return jsonify({
        "status": status,
        "message": response,
        "step_size": angles_per_key
    })


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
    limit_sensor = arm.motor_rail.limit_event.is_set()
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