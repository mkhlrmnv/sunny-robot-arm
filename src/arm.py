import time
import numpy as np
from .vector import Vector

class RobotArmController:
    def __init__(self, base_pos: Vector, tip_pos: Vector, arm1_length=80, arm2_length=50, num_joints=3):
        """
        Initializes the robotic arm controller with a specified number of joints.
        """
        self.base_pos = base_pos
        self.tip_pos = tip_pos
        self.num_joints = num_joints
        self.joint_angles = np.zeros(num_joints)  # Store joint angles (in degrees)
        self.arm1_length = arm1_length
        self.arm2_length = arm2_length

    def get_base_pos(self):
        return self.base_pos

    def get_tip_pos(self):
        """Returns the current position of the end-effector."""
        return self.tip_pos

    def get_joint_angles(self):
        """Returns the current joint angles."""
        return self.joint_angles
    
    def calc_join_angles(self, pos: Vector):
        assert type(pos) == Vector, "wrong type"

        # calculates theta2
        delta_z = self.base_pos.z - pos.z
        alpha = np.sqrt(self.arm2_length**2 - delta_z**2)
        theta3 = np.arccos(delta_z / self.arm2_length)
        theta2 = theta3 + np.deg2rad(90)

        # calculates theta1
        delta_y = self.base_pos.y - pos.y
        theta1 = np.arccos(delta_y / (self.arm1_length + alpha))

        return (theta1, theta2)

        
    def move_to(self, x, y, z, duration=1.0):
        """
        Moves the end-effector to the specified (x, y, z) position.
        """
        print(f"Moving to position: ({x}, {y}, {z})")
        time.sleep(duration)  # Simulate movement time
        self.position = np.array([x, y, z])
        print("Movement complete.")

    def set_joint_angles(self, joint_angles, duration=1.0):
        """
        Sets the robot arm to specified joint angles.
        """
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joint_angles)}")
        
        print(f"Setting joint angles to: {joint_angles}")
        time.sleep(duration)  # Simulate movement time
        self.joint_angles = np.array(joint_angles)
        print("Joint movement complete.")

# Example usage
if __name__ == "__main__":
    arm = RobotArmController()
    print("Initial position:", arm.get_position())
    arm.move_to(10, 5, 15)
    print("New position:", arm.get_position())
    arm.set_joint_angles([30, 45, 60])
    print("New joint angles:", arm.get_joint_angles())