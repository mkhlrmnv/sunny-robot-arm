import time
import numpy as np
from helper import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from linear_rail import LinearRail
from spinning_joints import SpinningJoints

class Arm:
    def __init__(self,
                 shared,
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

        self.motor_paaty = SpinningJoints(shared, pulse_pin=20, dir_pin=19, limit_pin=23, name="paaty", gear_ratio=5)
        self.motor_pontto = SpinningJoints(shared, pulse_pin=13, dir_pin=26, limit_pin=22, name="pontto", gear_ratio=5*32/10)
        self.motor_rail = LinearRail(shared, pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)


    def init(self):
        try:
            print("Starting init")
            self.motor_paaty.init_motor(direction=-1, speed=0.1)
            # self.motor_pontto.init_motor(direction=1, speed=0.1)
            # self.motor_rail.init_motor(direction=1)

            time.sleep(5)
        except TimeoutError:
            print("One of the motor couldn't init")
            return False
        
        self.motor_pontto.shared.theta_1 = self.motor_pontto.angle = self.theta_1 = 137.9

        self.motor_paaty.shared.theta_2 = self.motor_paaty.angle = self.theta_2 = 90+69.795 

        self.motor_rail.shared.delta_r = self.motor_rail.distance = self.delta_r = 0

        return True

    def set_from_shared(self, shared):
        """
        Set the arm's joint angles and rail position from shared namespace.
        """
        self.theta_1 = shared.theta_1
        self.theta_2 = shared.theta_2
        self.delta_r = shared.delta_r

        self.motor_pontto.angle = self.theta_1
        self.motor_paaty.angle = self.theta_2


    def init_path(self, path):
        # self.current_path, _ = get_sun_path()
        # self.current_path = np.load("paths/test_path.npy")
        # self.current_path = np.array([self.current_path[0]])
        self.current_path = path

    def move(self, shared=None):
        if self.current_path is None:
            raise ValueError("Initialize path first")

        if self.iteration >= len(self.current_path):
            print("Robot has already reached the end of the path")
            exit(68)
            return

        self.motor_pontto.angle = self.theta_1
        self.motor_paaty.angle = self.theta_2
        self.motor_rail.distance = self.delta_r

        try:
            if self._target_not_set():
                self._compute_next_target()

            print(f"Moving to target: theta_1={self.required_theta_1}, "
                  f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")

            if not self._step_towards('theta_1', self.required_theta_1):
                self.motor_pontto.move_to_angle(self.required_theta_1, speed=0.1)
                # pass  # Move theta_1

            if not self._step_towards('delta_r', self.required_delta_r):
                self.motor_rail.move_to_distance(self.required_delta_r, speed=0.5)
                # pass  # Move delta_r

            if not self._step_towards('theta_2', self.required_theta_2):
                print("Moving theta_2 to", self.required_theta_2)
                self.motor_paaty.move_to_angle(self.required_theta_2, speed=0.5)
                # pass  # Move theta_2

            self._clear_target()

            shared.theta_1 = self.theta_1
            shared.theta_2 = self.theta_2
            shared.delta_r = self.delta_r

            return False
        
        except ValueError as e:
            print(f"Error: {e}")
            print("Stopping robot.")
            exit(69)


    def _target_not_set(self):
        return any(v is None for v in (self.required_theta_1, self.required_theta_2, self.required_delta_r))


    def _step_towards(self, attr, target, step_size=1):
        """
        Check if moving 'attr' from current to 'target' in steps is safe.
        If yes, set to target immediately (since we know the whole path is safe) and return True.
        If already at target, return True.
        If not safe at any point, raise ValueError.
        """
        current = getattr(self, attr)
        diff = target - current

        if abs(diff) < 1e-6:  # Already at target (with tolerance)
            return True

        direction = diff / abs(diff)  # +1 or -1
        steps = int(abs(diff) // step_size)
        remainder = abs(diff) % step_size

        # Simulate all steps
        for i in range(steps):
            intermediate = current + direction * step_size * (i + 1)
            self._check_if_hypothetical_safe(attr, intermediate)

        # Check the remainder step (final position)
        if remainder > 0:
            final = current + direction * (steps * step_size + remainder)
            self._check_if_hypothetical_safe(attr, final)

        # If all steps are safe, perform the final move
        setattr(self, attr, target)
        return False


    def _check_if_hypothetical_safe(self, attr, value):
        """
        Check if the robot would be safe if 'attr' was set to 'value',
        while keeping other joint values as they currently are.
        """
        # Temporarily set value
        current_state = {
            'theta_1': self.theta_1,
            'theta_2': self.theta_2,
            'delta_r': self.delta_r,
        }

        current_state[attr] = value
        # print("current state", current_state)

        if not check_solutions_safety(
            (current_state['theta_1'], current_state['theta_2'], current_state['delta_r'])
        ):
            raise ValueError(
                f"Unsafe configuration detected when moving {attr} to {value}."
            )


    def _compute_next_target(self):
        """Compute next target joint values based on current path."""
        next_point = self.current_path[self.iteration]
        print("nect point in compute", next_point)
        sols = inverse_kinematics(*next_point, verbal=False)
        print("Solutions found:", sols)
        self.required_theta_1, self.required_theta_2, self.required_delta_r = choose_solution(
            sols, (self.theta_1, self.theta_2, self.delta_r)
        )

        print(f"Next target: theta_1={self.required_theta_1}, "
                f"theta_2={self.required_theta_2}, delta_r={self.required_delta_r}")


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
    # motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, gear_ratio=5)
    # motor_paaty.move_by_angle(90, speed=0.5)
    #motor_paaty.init_motor(direction=-1)
    

    # motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=5*32/10)
    # motor_pontto.move_by_angle(-90, speed=0.5)
    # motor_pontto.init_motor(direction=1, speed=0.2)

    # motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)
    # motor_rail.init_motor(direction=1)


    arm = Arm()
    arm.init()
    arm.init_path()

    while True:
        arm.move()
        time.sleep(0.1)

    # fig = plt.figure(figsize=(18, 9))
    # ax = fig.add_subplot(111, projection='3d')
# 
    # ani = FuncAnimation(fig, arm.draw_all, frames=len(arm.current_path), fargs=(ax,), interval=10)
    # plt.show()

    
