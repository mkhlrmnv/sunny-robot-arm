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
    def __init__(self,
                 shared,
                 init_pos=[BASE_X_OFFSET, BASE_Y_OFFSET, BASE_Z_OFFSET],
                 dz1=PONTTO_Z_OFFSET,
                 dx1=PONTTO_X_OFFSET,
                 dx2=PAATY_X_OFFSET,
                 dy1=PONTTO_ARM_LENGTH,
                 end_link_length=PAATY_ARM_LENGTH,
                 theta_r=RAIL_ANGLE,
                 rail_length=RAIL_MAX_LIMIT,
                 lamp_url=os.getenv("WLED_ADR")
                 ):
        """
        Initializes the robotic arm controller with a specified number of joints.
        """
        self.init_pos=init_pos
        self.dz1=dz1
        self.dx1=dx1
        self.dx2=dx2
        self.dy1=dy1
        self.end_link_length=end_link_length
        self.theta_r=theta_r
        self.rail_length=rail_length

        self.theta_1 = None
        self.theta_2 = None
        self.delta_r = None

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
        self.warning_sound = WarningSound("sounds/timanttei_leikattu.wav")


    def init(self):

        self.lamp.set_to_blink()
        self.warning_sound.start()

        try:
            print("Starting init")
            self.motor_paaty.init_motor(speed=0.1)
            self.motor_pontto.init_motor(speed=0.1)
            self.motor_rail.init_motor(direction=1)

            # time.sleep(5)
        except TimeoutError:
            print("One of the motor couldn't init")
            exit(67)
            return False
        
        self.motor_pontto.shared.theta_1 = self.motor_pontto.angle = self.theta_1 = RAIL_ANGLE

        self.motor_paaty.shared.theta_2 = self.motor_paaty.angle = self.theta_2 = THETA_2_VAL_IN_INIT

        self.motor_rail.shared.delta_r = self.motor_rail.distance = self.delta_r = DELTA_R_VAL_IN_INIT

        self.lamp.set_brightness(0)
        self.lamp.set_to_solid()
        self.warning_sound.stop()

        return True


    def shutdown(self):
        self.lamp.set_to_blink()

        self.motor_paaty.shutdown()
        self.motor_pontto.shutdown()
        self.motor_rail.init_motor()
        
        self.lamp.set_brightness(0)
        self.lamp.set_to_solid()


    def init_path(self, path, duration, dynamic_lamp=True):
        if type(path) == str:
            self.current_path, self.current_path_colors = un_jsonify_path(path_file_path)
        elif type(path) == np.ndarray or type(path) == np.array:
            self.current_path = path
        else:
            print(type(path))
            raise ValueError("path should be in str format if it's path to the file or np.array if it is straight path")
        
        if not dynamic_lamp:
            self.current_path_colors = None
        
        self.duration_per_point = duration / len(self.current_path)

        self.shared.path = self.current_path

        self.iteration = self.shared.path_it = 0
        
        if self.current_path is None:
            raise ValueError("Path initialization failed. Check the path file.")


    def move(self, speeds=None, check_safety=True):

        if self.lamp.brightness == 0 or self.lamp.effect != 0:
            self.lamp.set_brightness(255)
            self.lamp.set_to_solid()

        if self.current_path is None:
            raise ValueError("Initialize path first")

        if self.iteration >= len(self.current_path):
            print("Robot has already reached the end of the path")
            exit(68)
            return
        
        if speeds is None:
            speed_joint = 0.1
            speed_rail = 0.5
        else:
            speed_joint, speed_rail = speeds

        # print("self theta 1", self.theta_1)
        # print("self theta 2", self.theta_2)
        # print("self delta r", self.delta_r)

        # self.motor_pontto.angle = self.theta_1
        # self.motor_paaty.angle = self.theta_2
        # self.motor_rail.distance = self.delta_r

        since_last_move = time.time() - self.shared.timer
        if since_last_move < self.duration_per_point:
            print(f"Waiting for {self.duration_per_point - since_last_move:.2f} seconds before next move")
            exit(66)

        self.shared.timer = time.time()
        
        try:
            if self._target_not_set():
                distance_to_target = self._compute_next_target(check_safety=check_safety)

            # if movement is over 10cm, lamp will blink
            if distance_to_target > 100:
                self.lamp.set_to_blink()
            
            self.warning_sound.start()
        
            print(f"Moving to target: theta_1={self.required_theta_1}, "
                  f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")

            MAX_ITERATIONS = 1000
            iteration = 0

            while iteration < MAX_ITERATIONS:
                iteration += 1

                unsafe_joints = []      # joints that hit a safety wall this pass
                progress_made = False   # did _any_ joint advance?
                all_at_target = True    # assume done until proven otherwise

                try:
                    print("required", self.required_theta_1)
                    at_target = self._step_towards('theta_1', self.required_theta_1,
                                                check_safety=check_safety)
                    print("theta 1 at target", at_target)
                    if not at_target:
                        self.motor_pontto.move_to_angle(self.required_theta_1, speed=speed_joint) # , shared=shared)
                        # if shared is not None:
                        #     shared.theta_1 = self.theta_1
                        print("moved the motor")
                        progress_made = True
                        all_at_target = False
                except ValueError:
                    unsafe_joints.append('theta_1')
                    all_at_target = False

                try:
                    at_target = self._step_towards('delta_r', self.required_delta_r,
                                                check_safety=check_safety)
                    print("delta at target", at_target)
                    if not at_target:
                        self.motor_rail.move_to_distance(self.required_delta_r, speed=speed_rail) #, shared=shared)
                        # if shared is not None:
                        #     shared.delta_r = self.delta_r
                        progress_made = True
                        all_at_target = False
                except ValueError:
                    unsafe_joints.append('delta_r')
                    all_at_target = False

                try:
                    at_target = self._step_towards('theta_2', self.required_theta_2,
                                                check_safety=check_safety)
                    print("theta 2 at target", at_target)
                    if not at_target:
                        self.motor_paaty.move_to_angle(self.required_theta_2, speed=speed_joint) #, shared=shared)
                        # if shared is not None:
                        #     shared.theta_2 = self.theta_2
                        progress_made = True
                        all_at_target = False
                except ValueError:
                    unsafe_joints.append('theta_2')
                    all_at_target = False
                
                if all_at_target:
                    break                             # reached goal – great!

                if not progress_made:                 # no joint could move this pass
                    raise RuntimeError(f"Stopping: no safe motion "
                                    f"(unsafe joints: {', '.join(unsafe_joints)})")

                if iteration >= MAX_ITERATIONS:
                    raise RuntimeError(f"Stopping: max iterations reached "
                                    f"({MAX_ITERATIONS}) without reaching target.")

            if unsafe_joints and not progress_made:
                # Either every joint was unsafe, or the rest were already at target
                # so no further progress toward the goal is possible.
                raise RuntimeError(f"Stopping: no safe motion (unsafe joints: "
                                f"{', '.join(unsafe_joints)})")

            self._clear_target()

            if self.current_path_colors is not None:
                color = self.current_path_colors[self.shared.path_it]
                # self.lamp.set_color(*color, verbal=True)
                self.lamp.set_to_solid(*color)
            else:
                self.lamp.set_to_solid()

            self.warning_sound.stop()
            
            self.shared.path_it += 1
        
            return False
        
        
        except ValueError as e:
            print(f"Error: {e}")
            print("Stopping robot.")
            exit(69)


    def _target_not_set(self):
        return any(v is None for v in (self.required_theta_1, self.required_theta_2, self.required_delta_r))


    def _step_towards(self, attr, target, step_size=1, check_safety=True):
        """
        Check if moving 'attr' from current to 'target' in steps is safe.
        If yes, set to target immediately (since we know the whole path is safe) and return True.
        If already at target, return True.
        If not safe at any point, raise ValueError.
        """
        current = getattr(self.shared, attr)
        diff = target - current

        print("diff in step", diff)

        if abs(diff) < 1e-6:  # Already at target (with tolerance)
            return True

        direction = diff / abs(diff)  # +1 or -1
        steps = int(abs(diff) // step_size)
        remainder = abs(diff) % step_size

        # Simulate all steps
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


    def _check_if_hypothetical_safe(self, attr, value):
        """
        Check if the robot would be safe if 'attr' was set to 'value',
        while keeping other joint values as they currently are.
        """
        # Temporarily set value
        current_state = {
            'theta_1': self.shared.theta_1,
            'theta_2': self.shared.theta_2,
            'delta_r': self.shared.delta_r,
        }

        current_state[attr] = value
        # print("current state", current_state)

        if not check_solutions_safety(
            (current_state['theta_1'], current_state['theta_2'], current_state['delta_r'])
        ):
            raise ValueError(
                f"Unsafe configuration detected when moving {attr} to {value}."
            )


    def _quantize(self, desired: float, current: float, motor) -> float:
        step_angle = 360.0 / (motor.step_per_rev * motor.gear_ratio)
        # how many steps from current → desired
        n_steps = int((desired - current) / step_angle)
        return current + n_steps * step_angle


    def _compute_next_target(self, check_safety=True):
        """Compute next target joint values based on current path."""
        next_point = self.current_path[self.shared.path_it]
        curr_point = forward_kinematics(self.shared.theta_1, self.shared.theta_2, self.shared.delta_r)[-1]

        sols = inverse_kinematics(*next_point, verbal=False, check_safety=check_safety)
        self.required_theta_1, self.required_theta_2, self.required_delta_r = choose_solution(
            sols, (self.shared.theta_1, self.shared.theta_2, self.shared.delta_r)
        )

        self.required_theta_1 = self._quantize(self.required_theta_1, self.shared.theta_1, self.motor_pontto)
        self.required_theta_2 = self._quantize(self.required_theta_2, self.shared.theta_2, self.motor_paaty)
        self.required_delta_r = self._quantize(self.required_delta_r, self.shared.delta_r, self.motor_rail)

        print(f"Next target: theta_1={self.required_theta_1}, "
                f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")

        # returns distance to next point
        return np.linalg.norm(np.array(next_point) - np.array(curr_point))


    def _clear_target(self):
        """Reset required joint targets after reaching a point."""
        self.required_theta_1 = None
        self.required_theta_2 = None
        self.required_delta_r = None


    def draw_all(self, index, ax):
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
    # base_pos = Vector(0, 0, 0)
    # tip_pos = Vector(0, 80, 50)
    # arm = Arm(base_pos, tip_pos)
# 
    # target_pos = Vector(63.64, 63.64, 48.9898)
    # angles = arm.calc_join_angles(target_pos)
    # arm.set_joint_angles(angles)
    # arm.plot()

    # Init motors
    # motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, gear_ratio=5)
    # motor_paaty.move_by_angle(90, speed=0.5)
    #motor_paaty.init_motor(direction=-1)
    

    # motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=5*32/10)
    # motor_pontto.move_by_angle(-90, speed=0.5)
    # motor_pontto.init_motor(direction=1, speed=0.2)

    # motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)
    # motor_rail.init_motor(direction=1)

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

    
