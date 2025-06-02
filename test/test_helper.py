import unittest
from unittest.mock import patch
import time
from scipy.spatial.transform import Rotation as R

from src import helper
import numpy as np

class TestHelper(unittest.TestCase):
    def test_inverce_kinematics(self):
        # Define the initial position (in homogeneous coordinates)
        initial_position = np.array([0, 0, 0, 1]).T
        theta_1= 156
        theta_2 = -45

        # 0 -> 1
        T0_1 = np.eye(4)
        T0_1[2, 3] = 100

        # 1 -> 2
        T1_2 = np.eye(4)
        T1_2[:3, :3] = R.from_euler('z', theta_1, degrees=True).as_matrix()

        # 2 -> 3
        T2_3 = np.eye(4)
        T2_3[0, 3] = 57.5

        # 3 -> 4
        T3_4 = np.eye(4)
        T3_4[1, 3] = 830

        T4_5 = np.eye(4)
        T4_5[0, 3] = 107

        T5_6 = np.eye(4)
        T5_6[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

        T6_7 = np.eye(4)
        T6_7[1, 3] = 950

        transforms = [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7]

        # 3) Accumulate and collect points
        T_cum = np.eye(4)
        points = [T_cum @ np.array([0,0,0,1])]
        for T in transforms:
            T_cum = T_cum @ T
            points.append(T_cum @ np.array([0,0,0,1]))

        # strip off the homogeneous 1’s
        points = np.array(points)[:,:3]

        end_point = points[-1]

        th1, th2 = helper.inverse_kinematics(points[-1, 0], points[-1, 1], points[-1, 2])

        assert np.isclose(th1, theta_1, atol=1e-2), f"Expected {theta_1}, got {th1}"
        assert np.isclose(th2, theta_2, atol=1e-2), f"Expected {theta_2}, got {th2}"

    def test_forward_kinematics(self):
        # Define the joint angles
        theta1_deg = 156
        theta2_deg = -45

        # Compute the end-effector position using forward kinematics
        end_effector_pos = helper.forward_kinematics(theta1_deg, theta2_deg)

        # Expected position based on manual calculations or known values
        expected_pos = np.array([-761.09556809, -1305.01003068, -571.75144213])

        # Check if the computed position is close to the expected position
        np.testing.assert_allclose(end_effector_pos, expected_pos, rtol=1e-5, atol=1e-5)

if __name__ == '__main__':
    unittest.main()