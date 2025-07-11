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
from multiprocessing import Process, Queue, Manager
import numpy as np
import atexit

from spinning_joints import SpinningJoints
from linear_rail import LinearRail
from arm import Arm
import cooling
from helper import forward_kinematics, check_solutions_safety, inverse_kinematics, choose_solution

app = Flask(__name__)

# Global variable for the interactive (manual control) session.
global_wsd_proc = None

manager = Manager()
shared = manager.Namespace()

shared.theta_1 = 0
shared.theta_2 = 0
shared.delta_r = 0
shared.path_it = 0
shared.timer = 0

arm = Arm(shared)

# Start the cooling controller in its own thread.
def start_cooling_thread():
    print("starting cooling")
    controller = cooling.FanController(fan_pin=18, min_temp=20, max_temp=45)
    atexit.register(controller.shutdown)
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
        return arm_process.exitcode

@app.route('/status')
def status():
    print("exit code ", arm_process.exitcode)
    if arm_process is not None and arm_process.exitcode == 66:
        return jsonify({"running": "waiting"})

    elif arm_process is not None and arm_process.exitcode == 67:
        return jsonify({"running": "init_failed"})
    
    elif arm_process is not None and arm_process.exitcode == 68:
        return jsonify({"running": "path_ended"})
    
    elif arm_process is not None and arm_process.exitcode == 69:
        return jsonify({"running": "arm_stopped_safety"})

    elif arm_process is not None and arm_process.is_alive():
        return jsonify({"running": "running"})
        
    elif arm_process is not None and not arm_process.is_alive():
        return jsonify({"running": "stopped"})
    
    elif arm_process is None:
        return jsonify({"running": "not_started"})


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

@app.route("/shutdown")
def shutdown():
    global arm
    if is_arm_running():
        stop_arm()

    start_arm(arm.shutdown, ())
    return render_template("shutdown.html", running=is_arm_running())

@app.route("/stop")
def stop():
    stop_arm()
    return redirect(url_for("index"))

play_process = None
message_queue = Queue()


@app.route("/init_play_path")
def init_play_path():
    global arm
    if not is_arm_running():
        start_arm(arm.init, ())
    return render_template("init_play_path.html", running=is_arm_running())


@app.route('/choose_path')
def choose_path():
    return render_template('choose_path.html', running=is_arm_running())


@app.route('/play')
def play_path():
    global arm
    name = request.args.get('cmd')

    print("name ", name)
    print("os", os.path.dirname(__file__))

    path = os.path.join(os.path.dirname(__file__), '..', 'paths', name)

    arm.init_path(np.load(path), duration=0)
    arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
    print("starting to move")
    play_thread = threading.Thread(target=start_play_path_loop, daemon=True)
    play_thread.start()
    return render_template('play.html')  # your SSE+plot template


def start_play_path_loop():
    print("starting while loop")
    arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
    shared.path_it = 0
    while start_arm_and_wait(arm.move, (shared,)) == 0 or start_arm_and_wait(arm.move, (shared,)) == 66:
        arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
        arm.iteration = shared.path_it
        
    print("out of while loop")


@app.route('/points')
def points():
    global arm
    
    # get your live joint values from arm
    theta1 = shared.theta_1
    theta2 = shared.theta_2
    delta_r = shared.delta_r

    print("theta1:", theta1, "theta2:", theta2, "delta_r:", delta_r)

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
            start_arm(arm.motor_paaty.move_by_angle, (angles_per_key, 0.5, shared))
            response = "Motor paaty moving up"
        elif cmd == 'motor_paaty_down':
            start_arm(arm.motor_paaty.move_by_angle, (-angles_per_key, 0.5, shared))
            response = "Motor paaty moving down"
        elif cmd == 'motor_pontto_ccw':
            start_arm(arm.motor_pontto.move_by_angle, (angles_per_key, 0.5, shared))
            print("theta 1", shared.theta_1)
            response = "Motor pontto moving ccw"
        elif cmd == 'motor_pontto_cw':
            print("theta 1", shared.theta_1)
            start_arm(arm.motor_pontto.move_by_angle, (-angles_per_key, 0.5, shared))
            response = "Motor pontto moving cw"
        elif cmd == 'motor_rail_right':
            start_arm(arm.motor_rail.move_by_distance, (angles_per_key, 0.5, shared))
            response = "Motor rail moving right"
        elif cmd == 'motor_rail_left':
            start_arm(arm.motor_rail.move_by_distance, (-angles_per_key, 0.5, shared))
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
            speed = float(request.args.get('speed', 0.1))
            check = bool(int(request.args.get('check_safety', 1)))

            m = getattr(arm, f"motor_{motor}", None)
            if m is None:
                status, response = "error", f"Invalid motor: {motor}"
            else:
                # Update shared values
                arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r

                if check:
                    if motor == 'pontto':
                        target = shared.theta_1 + angle
                        end_point = forward_kinematics(target, shared.theta_2, shared.delta_r)[-1]
                        try:
                            sols = inverse_kinematics(*end_point, verbal=False)
                        except ValueError as e:
                            status, response = "error", f"Inverse kinematics failed: {e}"
                            return jsonify({"status": status, "message": response})
                        _, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
                        if round(c_th2, 1) != round(shared.theta_2, 2) or round(c_dr, 1) != round(shared.delta_r, 1):
                            status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                            return jsonify({"status": status, "message": response})
                    elif motor == 'paaty':
                        target = shared.theta_2 + angle
                        end_point = forward_kinematics(shared.theta_1, target, shared.delta_r)[-1]
                        try:
                            sols = inverse_kinematics(*end_point, verbal=False)
                        except ValueError as e:
                            status, response = "error", f"Inverse kinematics failed: {e}"
                            return jsonify({"status": status, "message": response})
                        c_th1, _, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
                        if round(c_th1, 1) != round(shared.theta_1, 1) or round(c_dr, 1) != round(shared.delta_r, 1):
                            status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                            return jsonify({"status": status, "message": response})
                    else:
                        status, response = "error", f"Invalid motor: {motor}"
                        return jsonify({"status": status, "message": response})

                    arm.init_path(np.array([end_point]), duration=0)
                    return_code = start_arm_and_wait(arm.move, (shared, (speed, 0.5),))
                    if return_code == 0:
                        response = f"Motor {motor} moved by {angle}° to {target} (with safety check)"
                    else:
                        status, response = "error", f"Function returned with exit code {return_code}"

                else:
                    m.angle = getattr(shared, f"theta_{1 if motor == 'pontto' else 2}")
                    if start_arm_and_wait(m.move_by_angle, (angle, speed, shared)) == 0:
                        index = 1 if motor == 'pontto' else 2
                        response = f"Motor {motor} moved by {angle}° to new angle: {getattr(shared, f'theta_{index}')}"
                    else:
                        status, response = "error", "Movement didn’t complete"

        elif cmd == 'to_angle':
            motor = request.args.get('motor')
            angle = float(request.args.get('angle'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))

            m = getattr(arm, f"motor_{motor}", None)
            if m is None:
                status, response = "error", f"Invalid motor: {motor}"
            else:
                arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
                if check:
                    if motor == 'pontto':
                        origin = shared.theta_1
                        end_point = forward_kinematics(angle, shared.theta_2, shared.delta_r)[-1]
                        try:
                            sols = inverse_kinematics(*end_point, verbal=False)
                        except ValueError as e:
                            status, response = "error", f"Inverse kinematics failed: {e}"
                            return jsonify({"status": status, "message": response})
                        _, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
                        if round(c_th2, 1) != round(shared.theta_2, 2) or round(c_dr, 1) != round(shared.delta_r, 1):
                            status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                            return jsonify({"status": status, "message": response})
                    elif motor == 'paaty':
                        origin = shared.theta_2
                        end_point = forward_kinematics(shared.theta_1, angle, shared.delta_r)[-1]
                        try:
                            sols = inverse_kinematics(*end_point, verbal=False)
                        except ValueError as e:
                            status, response = "error", f"Inverse kinematics failed: {e}"
                            return jsonify({"status": status, "message": response})
                        c_th1, _, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
                        if round(c_th1, 1) != round(shared.theta_1, 1) or round(c_dr, 1) != round(shared.delta_r, 1):
                            status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                            return jsonify({"status": status, "message": response})
                    else:
                        status, response = "error", f"Invalid motor: {motor}"
                        return jsonify(status=status, response=response)

                    arm.init_path(np.array([end_point]), duration=0)
                    return_code = start_arm_and_wait(arm.move, (shared, (speed, 0.5),))
                    if return_code == 0:
                        response = f"Motor {motor} moved from {origin} to {angle} (with safety check)"
                    else:
                        status, response = "error", f"Function returned with code {return_code}"
                        
                else:
                    origin = m.angle = getattr(shared, f"theta_{1 if motor == 'pontto' else 2}")
                    if start_arm_and_wait(m.move_to_angle, (angle, speed, shared)) == 0:
                        index = 1 if motor == 'pontto' else 2
                        response = f"Motor {motor} moved from {origin}° to new angle: {getattr(shared, f'theta_{index}')}"
                    else:
                        status, response = "error", "Movement didn’t complete"


        elif cmd == 'by_distance':
            dist = float(request.args.get('dist'))
            speed = float(request.args.get('speed', 0.1))
            check = bool(int(request.args.get('check_safety', 1)))
            
            if check:
                arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
                target = arm.motor_rail.distance + dist
                end_point = forward_kinematics(arm.theta_1, arm.theta_2, target)[-1]
                try:
                    sols = inverse_kinematics(*end_point, verbal=False)
                except ValueError as e:
                    status, response = "error", f"Inverse kinematics failed: {e}"
                    return jsonify({"status": status, "message": response})
                c_th1, c_th2, c_dr = choose_solution(sols, (arm.theta_1, arm.theta_2, arm.delta_r))
                if round(c_th1, 1) != round(arm.theta_1, 2) or round(c_th2, 1) != round(arm.theta_2, 1):
                    status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                    return jsonify({"status": status, "message": response})
                
                arm.init_path(np.array([end_point]), duration=0)
                return_code = start_arm_and_wait(arm.move, (shared, (0.5, speed),))
                if return_code == 0:
                    response = f"Rail moved by {dist} to {target} (with safety check)"
                else:
                    status, response = "error", f"Function returned with exit code {return_code}"

            else:
                arm.motor_rail.distance = shared.delta_r
                if start_arm_and_wait(arm.motor_rail.move_by_distance, (dist, speed, shared)) == 0:
                    response = f"Rail moved by {dist} to new distance: {shared.delta_r}"
                else:
                    status, response = "error", "Movement didn’t complete"

        elif cmd == 'to_distance':
            dist = float(request.args.get('dist'))
            speed = float(request.args.get('speed', 0.5))
            check = bool(int(request.args.get('check_safety', 1)))
            if check:
                arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r
                origin = shared.delta_r
                end_point = forward_kinematics(arm.theta_1, arm.theta_2, dist)[-1]
                try:
                    sols = inverse_kinematics(*end_point, verbal=False)
                except ValueError as e:
                    status, response = "error", f"Inverse kinematics failed: {e}"
                    return jsonify({"status": status, "message": response})
                c_th1, c_th2, c_dr = choose_solution(sols, (arm.theta_1, arm.theta_2, arm.delta_r))
                if round(c_th1, 1) != round(arm.theta_1, 2) or round(c_th2, 1) != round(arm.theta_2, 1):
                    status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                    return jsonify({"status": status, "message": response})
                
                arm.init_path(np.array([end_point]), duration=0)
                return_code = start_arm_and_wait(arm.move, (shared, (0.5, speed),))
                if return_code == 0:
                    response = f"Rail moved to {dist} from {origin} (with safety check)"
                else:
                    status, response = "error", f"Function returned with exit code {return_code}"

            else:
                arm.motor_rail.distance = shared.delta_r
                if start_arm_and_wait(arm.motor_rail.move_to_distance, (dist, speed, shared)) == 0:
                    response = f"Rail moved by {dist} to new distance: {shared.delta_r}"
                else:
                    status, response = "error", "Movement didn’t complete"

        elif cmd == 'to_point':
            x = float(request.args.get('x'))
            y = float(request.args.get('y'))
            z = float(request.args.get('z'))
            check = bool(int(request.args.get('check_safety', 1)))
            speed_rail = float(request.args.get('speed_rail', 0.5))
            speed_joints = float(request.args.get('speed_joints', 0.1))

            arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r

            sols = inverse_kinematics(x, y, z, verbal=False)
            c_th1, c_th2, c_dr = choose_solution(sols, (arm.theta_1, arm.theta_2, arm.delta_r))
            origin = forward_kinematics(c_th1, c_th2, c_dr)[-1]

            arm.init_path(np.array([[x, y, z]]), duration=0)
            return_code = start_arm_and_wait(arm.move, (shared, (speed_joints, speed_rail), check))
            if return_code == 0:
                response = f"Arm moved from ({origin[0], origin[1], origin[2]} to ({x}, {y}, {z})"
            else:
                status, response = "error", f"Function returned with exit code {return_code}"
 
        elif cmd == 'to_angles':
            theta_1 = float(request.args.get('theta_1'))
            theta_2 = float(request.args.get('theta_2'))
            delta_r = float(request.args.get('delta_r'))
            check = bool(int(request.args.get('check_safety', 1)))
            speed_rail = float(request.args.get('speed_rail', 0.5))
            speed_joints = float(request.args.get('speed_joints', 0.5))

            arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r

            end_point = forward_kinematics(theta_1, theta_2, delta_r)[-1]

            arm.init_path(np.array([end_point]), duration=0)
            return_code = start_arm_and_wait(arm.move, (shared, (speed_joints, speed_rail), check))
            if return_code == 0:
                response = f"Arm moved to angles: {theta_1}, {theta_2}, {delta_r} (with safety check)"
            else:
                status, response = "error", f"Function returned with exit code {return_code}"
            
        elif cmd == 'init':
            if start_arm_and_wait(arm.init, ()) == 0:
                response = f"Arm initialized and motor set to angles: theta_1 -> {shared.theta_1}, theta_2 -> {shared.theta_2}, delta_r -> {shared.delta_r}"
            else:
                status = "error"
                response = "One of the motors couldn't initialize"

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
    limit_sensor = arm.motor_pontto.limit_event.is_set()
    if limit_sensor:
        return jsonify({'pontto_induction_sensor_pressed': True})
    else:
        return jsonify({'pontto_induction_sensor_pressed': False})


@app.route('/check_paaty_induction_sensor')
def check_paaty_induction_sensor():
    global arm
    limit_sensor = arm.motor_paaty.limit_event.is_set()
    if limit_sensor:
        return jsonify({'paaty_induction_sensor_pressed': True})
    else:
        return jsonify({'paaty_induction_sensor_pressed': False})


@app.route('/done')
def done():
    return render_template('done.html')




if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000) #, debug=True, use_reloader=False)