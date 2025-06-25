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
        self.current_path, _ = get_sun_path()
        
    def move(self):
        if (type(self.current_path) != None) and ((self.iteration - 1) < len(self.current_path)):
            next_point = self.current_path[self.iteration-1]
        elif ((self.iteration - 1) >= len(self.current_path)):
            print("Robot reached the end of the path")
            return
        else:
            raise ValueError("Init path first")
        
        try:
            sols = inverse_kinematics(*next_point)

            self.theta_1, self.theta_2, self.delta_r = choose_solution(sols, (self.theta_1, self.theta_2, self.delta_r))
            self.iteration += 1

        except:
            self.iteration += 1

    def draw_all(self, index, ax):
        ax.clear()
        self.move()
        draw_all_safety_boxes(ax)
        draw_robot(ax, points=forward_kinematics(self.theta_1, self.theta_2, self.delta_r))
        plot_path(ax, self.current_path)


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

    
