import time
import numpy as np
from helper import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# from linear_rail import LinearRail
# from spinning_joints import SpinningJoints

class Arm:
    def __init__(self,
                 init_pos=[925.39, -219.38, 0],
                 dz1=100,
                 dx1=57.5,
                 dx2=107,
                 dy1=830,
                 end_link_length=950,
                 theta_r=137.9,
                 rail_length=1000
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

        self.current_path = None
        self.theta_1 = None
        self.theta_2 = None
        self.delta_r = None

        self.required_theta_1 = None
        self.required_theta_2 = None
        self.required_delta_r = None

        self.iteration = 0

        # self.motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, gear_ratio=1)
        # self.motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=1)
        # self.motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)


    def init(self):
        try:
            print("Starting init")
            # self.motor_paaty.init_motor(direction=-1)
            # self.motor_pontto.init_motor(direction=1)
            # self.motor_rail.init_motor(direction=1)
        except TimeoutError:
            print("One of the motor couldn't init")
            return False
        
        self.theta_1 = 137.9
        self.theta_2 = 135    # TODO: FIX THIS
        self.delta_r = 0

        return True


    def init_path(self):
        # self.current_path, _ = get_sun_path()
        self.current_path = np.load("paths/test_path.npy")
        
    def move(self):
        if self.current_path is None:
            raise ValueError("Initialize path first")

        try:
            if self._target_not_set():
                self._compute_next_target()

            elif not self._step_towards('theta_1', self.required_theta_1):
                self._check_if_new_is_safe()
                # TODO: ADD ACTUAL MOTOR CONTROL HERE
                pass  # Move theta_1

            elif not self._step_towards('delta_r', self.required_delta_r):
                # TODO: ADD ACTUAL MOTOR CONTROL HERE
                self._check_if_new_is_safe()
                pass  # Move delta_r

            elif not self._step_towards('theta_2', self.required_theta_2):
                self._check_if_new_is_safe()
                # TODO: ADD ACTUAL MOTOR CONTROL HERE
                pass  # Move theta_2

            else:
                self.iteration += 1
                self._clear_target()

                if self.iteration >= len(self.current_path):
                    print("Robot reached the end of the path")
                    return

                self._compute_next_target()
        
        except ValueError as e:
            print(f"Error: {e}")
            print("Stopping robot.")
            exit()

    def _check_if_new_is_safe(self):
        if not check_solutions_safety((self.theta_1, self.theta_2, self.delta_r)):
            raise ValueError("New joint angles are not safe. Please adjust the path or angles.")

    def _target_not_set(self):
        return any(v is None for v in (self.required_theta_1, self.required_theta_2, self.required_delta_r))


    def _step_towards(self, attr, target, step_size=1):
        """Move joint 'attr' towards 'target'. Return True if reached."""
        current = getattr(self, attr)
        diff = target - current

        if abs(diff) < step_size:
            setattr(self, attr, target)
            return True  # Target reached
        else:
            setattr(self, attr, current + step_size * (diff / abs(diff)))
            return False  # Still moving


    def _compute_next_target(self):
        """Compute next target joint values based on current path."""
        next_point = self.current_path[self.iteration]
        sols = inverse_kinematics(*next_point)
        self.required_theta_1, self.required_theta_2, self.required_delta_r = choose_solution(
            sols, (self.theta_1, self.theta_2, self.delta_r)
        )


    def _clear_target(self):
        """Reset required joint targets after reaching a point."""
        self.required_theta_1 = None
        self.required_theta_2 = None
        self.required_delta_r = None


    def draw_all(self, index, ax):
        ax.clear()
        self.move()
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
    # motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, gear_ratio=1)
    # motor_paaty.init_motor(direction=-1)

    # motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=1)
    # motor_pontto.init_motor(direction=1)

    # motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)
    # motor_rail.init_motor(direction=1)


    arm = Arm()
    arm.init()
    arm.init_path()

    fig = plt.figure(figsize=(18, 9))
    ax = fig.add_subplot(111, projection='3d')

    ani = FuncAnimation(fig, arm.draw_all, frames=len(arm.current_path), fargs=(ax,), interval=10)
    plt.show()

    
