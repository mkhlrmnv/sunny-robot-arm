"""
arm.py
~~~~~~

High-level Arm controller that coordinates two rotary joints (pontto, paaty),
one linear rail, a dynamic lamp, and a warning sound during path playback.

Key Responsibilities:
  - Initialize and shutdown hardware axes and lamp/sound
  - Load and manage execution of 3D paths with safety checks
  - Step joints/rail in small increments to follow the path
  - Coordinate lamp effects and warning buzzer on large moves or errors

Dependencies:
  - SpinningJoints, LinearRail for motor control
  - Lamp, WarningSound for visual/audio alerts
  - Kinematics (inverse_kinematics, forward_kinematics, choose_solution)
  - Shared multiprocessing.Namespace for cross-process state
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import os
from dotenv import load_dotenv
load_dotenv() 

from kinematics_and_safety import *
from sun_helper import *
from lamp import Lamp
from linear_rail import LinearRail
from spinning_joints import SpinningJoints
from config import *
from warning_sound import WarningSound

class Arm:
    """
    Controller for a 3-DOF robot arm:
      - Motor #1: pontto (rotates horizontally)
      - Motor #2: paaty  (rotates vertically)
      - Motor #3: linear rail (slides in Y)
    Also controls a lamp and warning buzzer during motion.
    """

    def __init__(self,
                 shared,
                 init_pos: np.ndarray = [BASE_X_OFFSET, BASE_Y_OFFSET, BASE_Z_OFFSET],
                 dz1: float = PONTTO_Z_OFFSET,
                 dx1: float = PONTTO_X_OFFSET,
                 dx2: float = PAATY_X_OFFSET,
                 dy1: float = PONTTO_ARM_LENGTH,
                 end_link_length: float = PAATY_ARM_LENGTH,
                 theta_r: float = RAIL_ANGLE,
                 rail_length: float = RAIL_MAX_LIMIT,
                 lamp_url: str = os.getenv("WLED_ADR")
                 ) -> None:
        """
        Initialize motor drivers, lamp, and warning sound.

        :param shared: multiprocessing.Namespace for sharing joint states
        :param init_pos: translation matrix to the pos of the base
        :param dz1, dx1, dx2, dy1: offsets of the robots joints
        :param end_link_length: length of the second arm of the robot
        :param theta_1: angle of the rail
        :param: rail_length: length of the rail
        :param lamp_url: HTTP URL for controlling the RGB lamp
        """
        self.init_pos=init_pos
        self.dz1=dz1
        self.dx1=dx1
        self.dx2=dx2
        self.dy1=dy1
        self.end_link_length=end_link_length
        self.theta_r=theta_r
        self.rail_length=rail_length

        # basic variables
        self.theta_1 = None
        self.theta_2 = None
        self.delta_r = None
        
        # Path-following state
        self.required_theta_1 = None
        self.required_theta_2 = None
        self.required_delta_r = None
        self.current_path = None
        self.current_path_colors = None
        self.iteration = 0
        self.duration_per_point = 0
        
        self.shared = shared

        self.motor_paaty = SpinningJoints(shared, 
                                          pulse_pin=PAATY_MOTOR_PULSE_PIN, 
                                          dir_pin=PAATY_MOTOR_DIR_PIN, 
                                          limit_pin=PAATY_MOTOR_SENSOR_PIN, 
                                          name="paaty", 
                                          gear_ratio=PAATY_MOTOR_GEAR_RATIO)
        
        self.motor_pontto = SpinningJoints(shared, 
                                           pulse_pin=PONTTO_MOTOR_PULSE_PIN, 
                                           dir_pin=PONTTO_MOTOR_DIR_PIN, 
                                           limit_pin=PONTTO_MOTOR_SENSOR_PIN, 
                                           name="pontto",
                                           gear_ratio=PONTTO_MOTOR_GEAR_RATION)
        
        self.motor_rail = LinearRail(shared, 
                                     pulse_pin=RAIL_MOTOR_PULSE_PIN, 
                                     dir_pin=RAIL_MOTOR_DIR_PIN, 
                                     limit_pin=RAIL_MOTOR_SENSOR_PIN, 
                                     gear_ratio=RAIL_MOTOR_GEAR_RATIO)

        self.lamp = Lamp(https_url=lamp_url)
        # self.warning_sound = WarningSound("sounds/ai_warning_2.wav")


    def init(self) -> bool:
        """
        Home all three axes and reset to known base pose.
        Blink the lamp and sound warning while homing.

        Returns True if successful, else exits with code 67.
        """

        # Indicate homing in progress
        self.lamp.set_to_blink()

        warning_sound = WarningSound("sounds/ai_warning_2.wav")
        warning_sound.start()

        try:
            print("Starting init")

            self.motor_paaty.init_motor(speed=0.1)
            self.motor_pontto.init_motor(speed=0.1)
            self.motor_rail.init_motor(direction=1)

        except TimeoutError:
            print("One of the motor couldn't init")
            exit(67)
            return False
        
        # Assume homed angles/position from config constants
        self.motor_pontto.shared.theta_1 = self.motor_pontto.angle = self.theta_1 = RAIL_ANGLE
        self.motor_paaty.shared.theta_2 = self.motor_paaty.angle = self.theta_2 = THETA_2_VAL_IN_INIT
        self.motor_rail.shared.delta_r = self.motor_rail.distance = self.delta_r = DELTA_R_VAL_IN_INIT

        # Restore lamp to solid off state and stop buzzer
        self.lamp.set_brightness(0)
        self.lamp.set_to_solid()
        warning_sound.stop()

        return True


    def shutdown(self) -> None:
        """
        Park the arm: blink lamp, move all axes back to home, then solid lamp off.
        """

        self.lamp.set_to_blink()
        
        self.motor_paaty.shutdown()
        self.motor_pontto.shutdown()
        self.motor_rail.init_motor()

        self.lamp.set_brightness(0)
        self.lamp.set_to_solid()


    def init_path(self, path: str | np.ndarray, duration: float, dynamic_lamp: bool = True):
        """
        Load a path for playback and set timing.

        :param path: filepath (str) or Nx3 numpy array
        :param duration: total traversal time (sec)
        :param dynamic_lamp: if True, lamp color follows path_colors
        """

        # Load JSON if provided a filename
        if type(path) == str:
            self.current_path, self.current_path_colors = un_jsonify_path(path)
        elif type(path) == np.ndarray or type(path) == np.array:
            self.current_path = path
        else:
            raise ValueError("Path must be filepath or numpy array")
        
        # Disable lamp colors if static desired
        if not dynamic_lamp:
            self.current_path_colors = None
        
        # Compute pause duration per waypoint
        self.duration_per_point = duration / len(self.current_path)
        self.shared.path = self.current_path
        self.iteration = self.shared.path_it = 0


    def move(self, speeds=None, check_safety=True):
        """
        Advance arm by one waypoint in current_path.
        Returns False to signal 'more to do', or exits with codes 68/69 on completion/error.
        """

        if self.current_path is None:
            raise ValueError("Initialize path first")

        # Check end of path
        if self.shared.path_it >= len(self.current_path):
            print("Robot has already reached the end of the path")
            exit(68)
            return
        
        # Set default speeds if not provided
        if speeds is None:
            speed_joint = 0.1
            speed_rail = 0.5
        else:
            speed_joint, speed_rail = speeds

        # Throttle to maintain timing
        since_last_move = time.time() - self.shared.timer
        if since_last_move < self.duration_per_point:
            print(f"Waiting for {self.duration_per_point - since_last_move:.2f} seconds before next move")
            time.sleep(self.duration_per_point - since_last_move)
            # exit(66)
        self.shared.timer = time.time()
        
        try:
            # Compute or reuse next target joint values
            if self._target_not_set():
                distance_to_target = self._compute_next_target(check_safety=check_safety)

            # Alert if large move
            warning_sound = WarningSound("sounds/ai_warning_2.wav")
            original_lamp_state = self.lamp.get_state()
            if distance_to_target > MIN_WARNING_DISTANCE:
                self.lamp.set_to_blink()
                warning_sound.start()
        
            print(f"Moving to target: theta_1={self.required_theta_1}, "
                  f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")

            MAX_ITERATIONS = 1000
            iteration = 0

            # Loop stepping each axis until all at target or error
            for _ in range(MAX_ITERATIONS):
                done = True
                # Step pontto joint toward required_theta_1
                if not self._step_towards('theta_1', self.required_theta_1, check_safety):
                    self.motor_pontto.move_to_angle(self.required_theta_1, speed=speed_joint)
                    done = False
                # Step rail
                if not self._step_towards('delta_r', self.required_delta_r, check_safety):
                    self.motor_rail.move_by_distance(self.required_delta_r, speed=speed_rail)
                    done = False
                # Step paaty joint
                if not self._step_towards('theta_2', self.required_theta_2, check_safety):
                    self.motor_paaty.move_to_angle(self.required_theta_2, speed=speed_joint)
                    done = False
                if done:
                    break
            if not done:
                raise RuntimeError("Could not reach target safely")

            # Clear target for next iteration
            self._clear_target()

            # Update lamp color or restore prior state
            if self.current_path_colors is not None:
                color = self.current_path_colors[self.shared.path_it]
                # self.lamp.set_color(*color, verbal=True)
                self.lamp.set_to_solid(*color)
            else:
                self.lamp.set_state(original_lamp_state)
                # self.lamp.set_to_solid()
            warning_sound.stop()

            # on first iteration timer starts only after robot has reached the point
            if self.shared.path_it == 0:
                self.shared.timer = time.time()

            # Advance to next waypoint and signal caller
            self.shared.path_it += 1
            return False
        except ValueError as e:
            print(f"Error: {e}")
            print("Stopping robot.")
            exit(69)

    # --- Internal helpers ---

    def _target_not_set(self) -> bool:
        '''Checks if target is set'''
        return any(v is None for v in (self.required_theta_1, self.required_theta_2, self.required_delta_r))


    def _step_towards(self, attr: str, target: float, step_size: int = 1, check_safety: bool = True) -> None:
        """
        Returns True if 'attr' (theta_1, theta_2, or delta_r) is already at target.
        Raises ValueError if any intermediate step would violate safety.
        """
        current = getattr(self.shared, attr)
        diff = target - current

        print("diff in step", diff)

        if abs(diff) < 0.226:  # Already at target (with tolerance)
            return True

        direction = diff / abs(diff)  # +1 or -1
        steps = int(abs(diff) // step_size)
        remainder = abs(diff) % step_size

        # Simulate hypothetical steps and check safety
        for i in range(steps):
            intermediate = current + direction * step_size * (i + 1)
            if check_safety:
                self._check_if_hypothetical_safe(attr, intermediate)

        # Check the remainder step (final position)
        if remainder > 0:
            final = current + direction * (steps * step_size + remainder)
            if check_safety:
                self._check_if_hypothetical_safe(attr, final)

        # If all steps are safe, perform the final move
        setattr(self, attr, target)
        print(f"Moved {attr} to {target} safely.")
        return False


    def _check_if_hypothetical_safe(self, attr: str, value: float) -> bool:
        """
        Checks whether setting shared.attr to value keeps the arm collision-free.
        Raises ValueError if unsafe.
        """
        # Temporarily set value
        current_state = {
            'theta_1': self.shared.theta_1,
            'theta_2': self.shared.theta_2,
            'delta_r': self.shared.delta_r,
        }

        current_state[attr] = value

        if not check_solutions_safety(
            (current_state['theta_1'], current_state['theta_2'], current_state['delta_r'])
        ):
            raise ValueError(
                f"Unsafe configuration detected when moving {attr} to {value}."
            )


    def _quantize(self, desired: float, current: float, motor) -> float:
        """Snap a desired angle/distance to nearest actual motor step."""
        step_angle = 360.0 / (motor.step_per_rev * motor.gear_ratio)
        # how many steps from current → desired
        n_steps = int((desired - current) / step_angle)
        return current + n_steps * step_angle


    def _compute_next_target(self, check_safety: bool = True) -> float:
        """
        Inverse-kinematics to compute required angles/distance for next waypoint.
        Quantizes them to nearest motor step increments.
        Returns Euclidean distance from current end-effector to next waypoint.
        """
        next_point = self.current_path[self.shared.path_it]
        curr_point = forward_kinematics(self.shared.theta_1, self.shared.theta_2, self.shared.delta_r)[-1]

        sols = inverse_kinematics(*next_point, verbal=False, check_safety=check_safety)
        self.required_theta_1, self.required_theta_2, self.required_delta_r = choose_solution(
            sols, (self.shared.theta_1, self.shared.theta_2, self.shared.delta_r)
        )

        # Snap to full motor steps
        self.required_theta_1 = self._quantize(self.required_theta_1, self.shared.theta_1, self.motor_pontto)
        self.required_theta_2 = self._quantize(self.required_theta_2, self.shared.theta_2, self.motor_paaty)
        self.required_delta_r = self._quantize(self.required_delta_r, self.shared.delta_r, self.motor_rail)

        print(f"Next target: theta_1={self.required_theta_1}, "
                f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")

        # returns distance to next point
        return np.linalg.norm(np.array(next_point) - np.array(curr_point))


    def _clear_target(self) -> None:
        """Reset required_theta_1/2 and required_delta_r to None."""
        self.required_theta_1 = None
        self.required_theta_2 = None
        self.required_delta_r = None


    def draw_all(self, index, ax) -> None:
        '''Helper function for drawing all stuff, used in testing'''
        ax.clear()
        try:
            self.move()
        except ValueError as e:
            print(f"Error during move: {e}")
        draw_all_safety_boxes(ax)
        draw_robot(ax, points=forward_kinematics(self.theta_1, self.theta_2, self.delta_r))
        plot_path(ax, self.current_path, linestyle='None')


# Example usage
if __name__ == "__main__":

    from multiprocessing import Process, Queue, Manager
    manager = Manager()
    shared = manager.Namespace()
    shared.theta_1 = 0
    shared.theta_2 = 0
    shared.delta_r = 0
    shared.timer = 0
    shared.path_it = 0


    arm = Arm(shared)
    arm.init()
    arm.init_path(path_file_path="paths/finish_sun_path.json", duration=0)

    while True:
       arm.move(shared=shared)
       time.sleep(0.1)

    # fig = plt.figure(figsize=(9, 6))
    # ax = fig.add_subplot(111, projection='3d')
# 
    # ani = FuncAnimation(fig, arm.draw_all, frames=len(arm.current_path), fargs=(ax,), interval=100)
    # plt.show()

    
