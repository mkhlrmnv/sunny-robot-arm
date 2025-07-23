#!/usr/bin/python3
"""
main.py
~~~~~~

Flask-based web server for controlling and visualizing a 3-DOF robot arm.

Features:
  - Manual and API-driven control of two rotary joints and one linear rail
  - Path playback with safety checks and inverse/forward kinematics
  - Real-time 3D plotting via Plotly
  - Cooling fan monitoring and control
  - Sensor and limit-switch diagnostics
"""
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

# Local modules for hardware control and kinematics
from spinning_joints import SpinningJoints
from linear_rail import LinearRail
from arm import Arm
import cooling
from kinematics_and_safety import forward_kinematics, check_solutions_safety, inverse_kinematics, choose_solution

# Flask app initialization
app = Flask(__name__)

# Global variable for the interactive (manual control) session.
global_wsd_proc = None

# Shared state across processes/threads
manager = Manager()
shared = manager.Namespace()

shared.theta_1 = 0.0
shared.theta_2 = 0.0
shared.delta_r = 0.0
shared.path_it = 0
shared.timer = 0
shared.path = None

# Arm controller instantiation using shared namespace
arm = Arm(shared)

# Global process handle for arm motion
arm_process = None

# ----------------------------------------------------------------------------
# Cooling fan thread startup
# ----------------------------------------------------------------------------
def start_cooling_thread() -> None:
    """Start the background fan-controller loop."""

    print("starting cooling")
    controller = cooling.FanController(fan_pin=18, min_temp=30, max_temp=60)
    
    # Ensure fan is shut down when the app exits
    atexit.register(controller.shutdown)
    # Blocking loop adjusts fan speed periodically
    controller.run(verbal=False, interval=0.1)

    print("started cooling")

# Launch the cooling thread as a daemon
# print("on own thread")
cooling_thread = threading.Thread(target=start_cooling_thread, daemon=True)
cooling_thread.start()


# ----------------------------------------------------------------------------
# Utility functions for arm process management
# ----------------------------------------------------------------------------
def start_arm(func, args: tuple) -> None:
    """
    Launch arm control function in a separate process.  If one is already
    running, stop it first before starting a new one.
    This function doesn't wait for respond, but returns instantly after process is 
    started.
    """
    global arm_process, arm
    if arm_process and arm_process.is_alive():
        stop_arm()
    arm_process = Process(target=func, args=args)
    arm_process.start()


def start_arm_and_wait(func, args: tuple) -> int:
    """
    Launch arm control function and block until it completes. This functions
    waits to the end of execution and Returns exit code.
    """
    global arm_process
    if arm_process and arm_process.is_alive():
        stop_arm()
        arm_process = Process(target=func, args=args)
        arm_process.start()
        arm_process.join()
        return arm_process.exitcode if arm_process else -1 # TODO: <- this was changed so hopefully all works
        
def stop_arm():
    """Forcefully terminate any running arm process."""
    global arm_process
    print("Stopping arm process...")
    if arm_process and arm_process.is_alive():
        arm_process.kill()
        arm_process.join()
        print("Alive after all this?", arm_process.is_alive())
        arm_process = None

def is_arm_running():
    """Check if the arm process is currently alive."""
    return arm_process is not None and arm_process.is_alive()
        

# ----------------------------------------------------------------------------
# Flask endpoints
# ----------------------------------------------------------------------------
@app.route('/')
def index():
    """
    Home page: stop any motion and render main UI.
    Also calling this stops arms movement at any point
    """
    stop_arm()
    return render_template('index.html')


@app.route('/cooling_info')
def cooling_info():
    """Return the latest fan temperature/speed reading as JSON."""
    try:
        reading = cooling.latest_reading
    except AttributeError:
        reading = {"temperature": "--", "fan_speed": "--"}
    # print(reading)
    return jsonify(reading)


@app.route('/status')
def status(): # TODO: hopefully this worsk:)
    """Report current arm process state."""
    if arm_process is None:
        return jsonify({'running': 'not_started'})
    if arm_process.is_alive():
        return jsonify({'running': 'running'})
    code = arm_process.exitcode
    # Map specific exit codes to states
    return jsonify({'running': {
        66: 'waiting',
        67: 'init_failed',
        68: 'path_ended',
        69: 'arm_stopped_safety'
    }.get(code, 'stopped')})


@app.route("/init")
def init():
    """Initialize (home) the arm joints."""
    global arm
    if not is_arm_running():
        start_arm(arm.init, ())
    return render_template("init.html", running=is_arm_running())


@app.route("/init_play_path")
def init_play_path():
    """
    Initialize (home) the arm joints before executing path
    Basically the same as /init, but after completion loads different template
    """
    global arm
    if not is_arm_running():
        start_arm(arm.init, ())
    return render_template("init_play_path.html", running=is_arm_running())


@app.route("/shutdown")
def shutdown():
    """Moves arm to safe position for power off."""
    global arm
    if is_arm_running():
        stop_arm()

    start_arm(arm.shutdown, ())
    return render_template("shutdown.html", running=is_arm_running())


@app.route("/stop")
def stop():
    """Stop any motion and redirect to home."""
    stop_arm()
    return redirect(url_for("index"))


@app.route('/choose_path')
def choose_path():
    '''Renders template for choosing the path'''
    return render_template('choose_path.html', running=is_arm_running())


@app.route('/play')
def play_path():
    '''Executes the path and renders live visualization of the robot'''
    global arm

    # Read query parameters from the URL
    name = request.args.get('name')                     # filename of the path to play back
    duration = int(request.args.get('duration'))        # total time (seconds) over which to traverse the path
    dynamic_lamp = bool(int(request.args.get('lamp')))  # whether to animate the lamp during playback

    # Resolve the filesystem path to the saved path file
    path = os.path.join(os.path.dirname(__file__), '..', 'paths', name)

    # Initialize the arm's path: load the file and set up shared.path, duration, lamp flag
    arm.init_path(path, duration=duration, dynamic_lamp=dynamic_lamp)

    print("starting to move")
    
    # Start the playback loop in a background daemon thread so the HTTP request
    # can return immediately (rendering play.html), while motion continues
    play_thread = threading.Thread(target=start_play_path_loop, daemon=True)
    play_thread.start()

    # renders template
    return render_template('play.html')  # your SSE+plot template


def start_play_path_loop():
    """Loop through the initialized path, moving the arm step by step."""

    print("starting while loop")

    # Keep invoking arm.move in a blocking fashion until the path is executed
    # which shows is exitcode 0 that is returned only when robot has reached the end
    exit_code = 0
    while exit_code == 0:
        exit_code = start_arm_and_wait(arm.move, ())
        print("iteration", shared.path_it)
        
    print("out of while loop")


@app.route('/angles')
def angles():
    '''Returns current state of the angles as JSON {'theta1': ..., 'theta_2: ..., 'delta_r': ...}.'''

    # log for debugging
    print(f"theta1: {shared.theta_1}, theta2: {shared.theta_2}, delta_r: {shared.delta_r}")

    # return as JSON
    return jsonify({
        'theta1':  shared.theta_1,
        'theta2':  shared.theta_2,
        'delta_r': shared.delta_r
    })


@app.route('/points')
def points():
    '''Returns coordinates of the robot as JSON list of [x,y,z].'''
    global arm
    pts = forward_kinematics(shared.theta_1, shared.theta2, shared.delta_r)
    # pts is an (N×3) numpy array
    return jsonify(pts.tolist())


@app.route('/path_points')
def path_points():
    """Return current planned path (if any) as JSON list."""
    if shared.path is None:
        pts = []
        return jsonify(pts)
    else:
        pts = shared.path
        # pts is an (N×3) numpy array
        return jsonify(pts.tolist())


@app.route('/manual')
def manual_control():
    '''Renders the manual control page'''
    global angles_per_key
    angles_per_key = 1
    return render_template('manual_control.html')


@app.route('/api_play_path')
def api_play_path():
    '''
    Allows api request for play path
    Mainly is different from /play_path in a way that it init the motor and waits
    until the path is executed by returning the JSON stating the result
    '''
    global arm

    # Get the variables, same as in /play_path
    name = request.args.get('name')
    duration = int(request.args.get('duration'))
    dynamic_lamp = bool(int(request.args.get('lamp', 1)))

    path = os.path.join(os.path.dirname(__file__), '..', 'paths', name)

    # inits motors and waits for it
    start_arm_and_wait(arm.init, ())

    # init the path
    arm.init_path(path, duration=duration, dynamic_lamp=dynamic_lamp)

    exit_code = 0

    # starts the while loop that will end when robot has reached the end of path
    # in other words when arm.move stops exiting with 0
    while exit_code == 0:
        exit_code = start_arm_and_wait(arm.move, ())
        print("iteration", shared.path_it)

    # based on the exit code returns JSON that stated the result
    if exit_code == 68:
        return jsonify({
            "status": 'ok',
            "message": 'arm reached the end of the path'
            })
    
    elif exit_code == 69:
        return jsonify({
            "status": 'error',
            "message": 'robot stopped due to safety reasons'
            })
    
    else: 
        return jsonify({
            "status": 'error',
            "message": 'robot stopped due to unknown reasons'
            })


# variable for manual control
angles_per_key = 1

@app.route('/move_arm')
def move_arm():
    global arm, angles_per_key
    '''
    THE API handler, honestly the hella mess of a function. Sorry for that:)
    
    Commenting it will make it supper long and unreadable, basically first cmds 
    are for manual control and moving robot by steps. Then it moves to more advanced
    api calls where it first collects the arguments. Mainly on the safety argument it then 
    chooses two paths, if safety is on it calculated required end point of the robot
    to reach request angle / point / whatever and then reassures that is safe before executing it.
    If safety is of then is just moves the robot to he requested parameters. 

    If everything was successful API returns JSON with "status" "ok" and then some clarifications 
    on what was done, and if there was an error status of JSON becomes "error" and API tries to describe what 
    were wrong.

    I will comment to_angle handeling and all there rest works similarly so you will get the idea
    '''

    cmd = request.args.get('cmd')
    response = ""
    status = "ok"

    if is_arm_running():
        stop_arm()

    # Manual control stuff -> move up / down, etc.
    if cmd == 'motor_paaty_up':
        start_arm(arm.motor_paaty.move_by_angle, (angles_per_key, 0.5))
        response = "Motor paaty moving up"
    elif cmd == 'motor_paaty_down':
        start_arm(arm.motor_paaty.move_by_angle, (-angles_per_key, 0.5))
        response = "Motor paaty moving down"
    elif cmd == 'motor_pontto_ccw':
        start_arm(arm.motor_pontto.move_by_angle, (angles_per_key, 0.5))
        # print("theta 1", shared.theta_1)
        response = "Motor pontto moving ccw"
    elif cmd == 'motor_pontto_cw':
        print("theta 1 brefore ", shared.theta_1)
        start_arm(arm.motor_pontto.move_by_angle, (-angles_per_key, 0.5))
        print("theta 1 after ", shared.theta_1)
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
        # gets the arguments
        motor = request.args.get('motor')
        angle = float(request.args.get('angle'))
        speed = float(request.args.get('speed', 0.1))
        check = bool(int(request.args.get('check_safety', 1)))
        m = getattr(arm, f"motor_{motor}", None)

        if m is None:
            status, response = "error", f"Invalid motor: {motor}"
        else:
            if check:
                # if safety is on goes here

                #chooses the right motor
                if motor == 'pontto':
                    #calculates the end position of the robot based on current and requested angles
                    target = shared.theta_1 + angle
                    end_point = forward_kinematics(target, shared.theta_2, shared.delta_r)[-1]
                    
                    try:
                        # tries to IK function
                        sols = inverse_kinematics(*end_point, verbal=False)

                    except ValueError as e:
                        # if fails returns with status error and error message
                        status, response = "error", f"Inverse kinematics failed: {e}"
                        return jsonify({"status": status, "message": response})
                    
                    # If it gets the IK solution, checks that rest of the angles stays the same, 
                    # as user has asked to move only one angle
                    # if angles differ return that movement isn't possible
                    _, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
                    if round(c_th2, 1) != round(shared.theta_2, 1) or round(c_dr, 1) != round(shared.delta_r, 1):
                        status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                        return jsonify({"status": status, "message": response})

                elif motor == 'paaty':
                    # same as with pontto but different angles are used
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

                # if moving only one angle is safe, inits one point path and executes the move function
                arm.init_path(np.array([end_point]), duration=0, dynamic_lamp=False)
                return_code = start_arm_and_wait(arm.move, ((speed, 0.5),))

                # waits for the result
                if return_code == 0:
                    target = shared.theta_1 if motor == 'pontto' else shared.theta_2
                    response = f"Motor {motor} moved by {angle}° to {target} (with safety check)"
                else:
                    status, response = "error", f"Function returned with exit code {return_code}"

            # if safety is not wanted just moves the angle:)
            else:
                m.angle = getattr(shared, f"theta_{1 if motor == 'pontto' else 2}")
                if start_arm_and_wait(m.move_by_angle, (angle, speed)) == 0:
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
                    if round(c_th2, 1) != round(shared.theta_2, 1) or round(c_dr, 1) != round(shared.delta_r, 1):
                        status, response = "error", f"Safety check failed: safe movement will require movement of other motors as current theta_2 angle {shared.theta_2} doesn't macth required {c_th2} or current delta_r {shared.delta_r} doesn't macth required {c_dr}"
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

                arm.init_path(np.array([end_point]), duration=0, dynamic_lamp=False)
                return_code = start_arm_and_wait(arm.move, ((speed, 0.5),))
                if return_code == 0:
                    angle = shared.theta_1 if motor == 'pontto' else shared.theta_2
                    response = f"Motor {motor} moved from {origin} to {angle} (with safety check)"
                else:
                    status, response = "error", f"Function returned with code {return_code}"
                    
            else:
                origin = m.angle = getattr(shared, f"theta_{1 if motor == 'pontto' else 2}")
                if start_arm_and_wait(m.move_to_angle, (angle, speed)) == 0:
                    index = 1 if motor == 'pontto' else 2
                    response = f"Motor {motor} moved from {origin}° to new angle: {getattr(shared, f'theta_{index}')}"
                else:
                    status, response = "error", "Movement didn’t complete"


    elif cmd == 'by_distance':
        dist = float(request.args.get('dist'))
        speed = float(request.args.get('speed', 0.1))
        check = bool(int(request.args.get('check_safety', 1)))
        
        if check:
            target = shared.delta_r + dist
            print("targer ", target)
            end_point = forward_kinematics(shared.theta_1, shared.theta_2, target)[-1]
            try:
                sols = inverse_kinematics(*end_point, verbal=False)
            except ValueError as e:
                status, response = "error", f"Inverse kinematics failed: {e}"
                return jsonify({"status": status, "message": response})
            c_th1, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
            if round(c_th1, 1) != round(shared.theta_1, 2) or round(c_th2, 1) != round(shared.theta_2, 1):
                status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                return jsonify({"status": status, "message": response})
            
            arm.init_path(np.array([end_point]), duration=0, dynamic_lamp=False)
            return_code = start_arm_and_wait(arm.move, ((0.5, speed),))
            if return_code == 0:
                response = f"Rail moved by {dist} to {shared.delta_r} (with safety check)"
            else:
                status, response = "error", f"Function returned with exit code {return_code}"

        else:
            # arm.motor_rail.distance = shared.delta_r
            if start_arm_and_wait(arm.motor_rail.move_by_distance, (dist, speed)) == 0:
                response = f"Rail moved by {dist} to new distance: {shared.delta_r}"
            else:
                status, response = "error", "Movement didn’t complete"

    elif cmd == 'to_distance':
        dist = float(request.args.get('dist'))
        speed = float(request.args.get('speed', 0.5))
        check = bool(int(request.args.get('check_safety', 1)))
        if check:
            origin = shared.delta_r
            print("dist", dist)
            end_point = forward_kinematics(shared.theta_1, shared.theta_2, dist)[-1]
            try:
                sols = inverse_kinematics(*end_point, verbal=False)
            except ValueError as e:
                status, response = "error", f"Inverse kinematics failed: {e}"
                return jsonify({"status": status, "message": response})
            c_th1, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
            if round(c_th1, 1) != round(shared.theta_1, 2) or round(c_th2, 1) != round(shared.theta_2, 1):
                status, response = "error", "Safety check failed: safe movement will require movement of other motors"
                return jsonify({"status": status, "message": response})
            
            arm.init_path(np.array([end_point]), duration=0, dynamic_lamp=False)
            return_code = start_arm_and_wait(arm.move, ((0.5, speed),))
            if return_code == 0:
                response = f"Rail moved to {shared.delta_r} from {origin} (with safety check)"
            else:
                status, response = "error", f"Function returned with exit code {return_code}"

        else:
            # arm.motor_rail.distance = shared.delta_r
            if start_arm_and_wait(arm.motor_rail.move_to_distance, (dist, speed)) == 0:
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

        try:
            sols = inverse_kinematics(x, y, z, verbal=False)
        except ValueError as e:
            status, response = "error", f"Inverse kinematics failed: {e}"
            return jsonify({"status": status, "message": response})

        c_th1, c_th2, c_dr = choose_solution(sols, (shared.theta_1, shared.theta_2, shared.delta_r))
        origin = forward_kinematics(c_th1, c_th2, c_dr)[-1]

        arm.init_path(np.array([[x, y, z]]), duration=0, dynamic_lamp=False)
        return_code = start_arm_and_wait(arm.move, ((speed_joints, speed_rail), check))
        if return_code == 0:
            res = forward_kinematics(shared.theta_1, shared.theta_2, shared.delta_r)[-1]
            response = f"Arm moved from ({origin[0], origin[1], origin[2]} to ({res[0]}, {res[1]}, {res[2]})"
        else:
            status, response = "error", f"Function returned with exit code {return_code}"

    elif cmd == 'to_angles':
        theta_1 = float(request.args.get('theta_1'))
        theta_2 = float(request.args.get('theta_2'))
        delta_r = float(request.args.get('delta_r'))
        check = bool(int(request.args.get('check_safety', 1)))
        speed_rail = float(request.args.get('speed_rail', 0.5))
        speed_joints = float(request.args.get('speed_joints', 0.5))

        # arm.theta_1, arm.theta_2, arm.delta_r = shared.theta_1, shared.theta_2, shared.delta_r

        end_point = forward_kinematics(theta_1, theta_2, delta_r)[-1]

        arm.init_path(np.array([end_point]), duration=0, dynamic_lamp=False)
        return_code = start_arm_and_wait(arm.move, ((speed_joints, speed_rail), check))
        if return_code == 0:
            response = f"Arm moved to angles: {shared.theta_1}, {shared.theta_2}, {shared.delta_r} (with safety check)"
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
    '''Renders the first page of sensor test'''
    return render_template('unplug.html')


@app.route('/unplug_done')
def unplug_done():
    '''redirects to the next part of sensor test'''
    return redirect(url_for('right_sensor_test'))


@app.route('/right_sensor_test')
def right_sensor_test():
    '''Instructs user what sensor to press'''
    return render_template('right_sensor_test.html')


@app.route('/left_sensor_test')
def left_sensor_test():
    '''Same as above'''
    return render_template('left_sensor_test.html')


@app.route('/pontto_induction_sensor_test')
def pontto_induction_sensor_test():
    '''Same as above'''
    return render_template('pontto_induction_sensor_test.html')


@app.route('/paaty_induction_sensor_test')
def paaty_induction_sensor_test():
    '''Same as above'''
    return render_template('paaty_induction_sensor_test.html')


@app.route('/check_limit_sensor')
def check_limit_sensors():
    global arm
    '''End point for checking it the limit is pressed'''
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
    '''End point for checking the induction sensors'''
    global arm
    limit_sensor = arm.motor_paaty.limit_event.is_set()
    if limit_sensor:
        return jsonify({'paaty_induction_sensor_pressed': True})
    else:
        return jsonify({'paaty_induction_sensor_pressed': False})


@app.route('/done')
def done():
    '''The sensor test done page'''
    return render_template('done.html')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000) #, debug=True, use_reloader=False)