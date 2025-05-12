import time
import numpy as np
from vector import Vector, Joints
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from linear_rail import LinearRail
from spinning_joints import SpinningJoints

class Arm:
    def __init__(self, base_pos: Vector, tip_pos: Vector, arm1_length=80, arm2_length=50):
        """
        Initializes the robotic arm controller with a specified number of joints.
        """
        self.base_pos = base_pos
        self.tip_pos = tip_pos
        self.arm1_length = arm1_length
        self.arm2_length = arm2_length
        self.joint_angles = self.calc_join_angles(tip_pos)

    def get_base_pos(self):
        return self.base_pos

    def get_tip_pos(self):
        """Returns the current position of the end-effector."""
        return self.tip_pos

    def get_joint_angles(self):
        """Returns the current joint angles."""
        return self.joint_angles
    
    def calc_join_angles(self, pos: Vector):
        """
        Calculate the joint angles for the robotic arm to reach a given position.
        Args:
            pos (Vector): The target position as a Vector object.
        Returns:
            list: A list containing the calculated angles [theta1, theta2].
        Raises:
            AssertionError: If the input position is not of type Vector.
            ValueError: If the target position is unreachable.
            AssertionError: If the calculation of theta1 or theta2 results in NaN.
        """
        assert isinstance(pos, Vector), "Pos should be type Vector"

        distance = np.sqrt((self.base_pos.x - pos.x)**2 + (self.base_pos.y - pos.y)**2 + (self.base_pos.z - pos.z)**2)
        if distance > (self.arm1_length + self.arm2_length) or distance < (self.arm1_length - self.arm2_length):
            raise ValueError("The position is unreachable")

        # calculates theta2
        delta_z = pos.z- self.base_pos.z
        # breakpoint()
        if (self.arm2_length**2 - delta_z**2) == 0:
            alpha = 0
        else:
            alpha = np.sqrt(self.arm2_length**2 - delta_z**2)
        
        theta3 = np.arccos(delta_z / self.arm2_length)
        theta2 = theta3 + np.deg2rad(90)

        # calculates theta1
        delta_y = pos.y - self.base_pos.y
        theta1 = np.arcsin(delta_y / (self.arm1_length + alpha))

        assert not np.isnan(theta1), "Calculation on theta1 failed"
        assert not np.isnan(theta2), "Calculation on theta2 failed"
        
        return Joints(theta1, theta2)
    
    def calc_pos(self, joints: Joints):
        """
        Calculate the position of the robot arm's end effector based on given joint angles.

        Args:
            theta1 (float): The angle of the first joint in radians.
            theta2 (float): The angle of the second joint in radians.

        Returns:
            Vector: The position of the end effector as a Vector object with x, y, and z coordinates.

        Raises:
            AssertionError: If theta1 or theta2 are not of type float.
            AssertionError: If the calculation of x, y, or z results in NaN.
        """
        assert isinstance(joints, Joints), "joints should be type Joint"

        theta3 = joints.theta2 - np.deg2rad(90)
        alpha = np.sin(theta3) * self.arm2_length
        delta_z = np.sqrt(self.arm2_length**2 + alpha**2)
        delta_x = np.sin(joints.theta1) * (self.arm1_length + alpha)
        delta_y = np.cos(joints.theta1) * (self.arm1_length + alpha)

        assert not np.isnan(delta_x), "Calculation of x failed"
        assert not np.isnan(delta_y), "Calculation of y failed"
        assert not np.isnan(delta_z), "Calculation of z failed"

        return Vector(self.base_pos.x + delta_x, 
                      self.base_pos.y + delta_y, 
                      self.base_pos.z + delta_z)
        
    def set_joint_angles(self, joints: Joints):
        self.joint_angles = joints
        self.tip_pos = self.calc_pos(joints)

    def set_tip_pos(self, pos: Vector):
        self.tip_pos = pos
        self.joint_angles = self.calc_join_angles(pos)

    def plot(self):
        """
        Visualize the current state of the robotic arm in 3D.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Base joint (origin)
        base = self.base_pos

        # First joint
        theta1 = self.joint_angles.theta1
        arm1_end = Vector(
            base.x + np.sin(theta1) * self.arm1_length,
            base.y - np.cos(theta1) * self.arm1_length,
            base.z
        )

        # Second joint (tip)
        theta3 = self.joint_angles.theta2 - np.deg2rad(90)
        delta_z = np.cos(theta3) * self.arm2_length
        delta_x = np.sin(theta1) * np.sin(theta3) * self.arm2_length
        delta_y = np.cos(theta1) * np.sin(theta3) * self.arm2_length

        tip = Vector(
            arm1_end.x + delta_x,
            arm1_end.y + delta_y,
            arm1_end.z + delta_z
        )

        # Plot links
        xs = [base.x, arm1_end.x, tip.x]
        ys = [base.y, arm1_end.y, tip.y]
        zs = [base.z, arm1_end.z, tip.z]

        ax.plot(xs, ys, zs, marker='o', linewidth=2, markersize=6)
        ax.scatter([tip.x], [tip.y], [tip.z], color='red', label='End Effector')

        ax.set_xlim(-150, 150)
        ax.set_ylim(-150, 150)
        ax.set_zlim(0, 150)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Arm Visualization')
        ax.legend()
        plt.show()


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
    motor_paaty = SpinningJoints(pulse_pin=20, dir_pin=19, limit_pin=23, gear_ratio=1)
    motor_paaty.init_motor(direction=-1)

    motor_pontto = SpinningJoints(pulse_pin=13, dir_pin=26, limit_pin=22, gear_ratio=1)
    motor_pontto.init_motor(direction=1)
    
    motor_rail = LinearRail(pulse_pin=27, dir_pin=4, limit_pin=24, gear_ratio=1)
    motor_rail.init_motor(direction=1)

    
