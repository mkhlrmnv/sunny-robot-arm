import unittest
from unittest.mock import patch
import time
from scipy.spatial.transform import Rotation as R

from src import helper
import numpy as np

class TestHelper(unittest.TestCase):

    def test_inverce_kinematics_manual(self):
        # Generate random delta_r, theta_1, and theta_2 values
        delta_r = 300
        theta_1 = 59
        theta_2 = 89

        print("delta_r", delta_r)
        print("theta_1", theta_1)
        print("theta_2", theta_2)

        # Define the initial position (in homogeneous coordinates)
        initial_position = np.array([0, 0, 0, 1]).T
        theta_r = 137.9

        T_base = np.eye(4)
        T_base[0, 3] = 925.39
        T_base[1, 3] = -219.38

        T0_1 = np.eye(4)
        T0_1[:3, :3] = R.from_euler('z', theta_r, degrees=True).as_matrix()

        T1_2 = np.eye(4)
        T1_2[1, 3] = delta_r

        # 0 -> 1
        T2_3 = np.eye(4)
        T2_3[2, 3] = 100

        # 1 -> 2
        T3_4 = np.eye(4)
        T3_4[:3, :3] = R.from_euler('z', theta_1 - theta_r, degrees=True).as_matrix()

        # 2 -> 3
        T4_5 = np.eye(4)
        T4_5[0, 3] = 57.5

        # 3 -> 4
        T5_6 = np.eye(4)
        T5_6[1, 3] = 830

        T6_7 = np.eye(4)
        T6_7[0, 3] = 107

        T7_8 = np.eye(4)
        T7_8[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

        T8_9 = np.eye(4)
        T8_9[1, 3] = 950

        transforms = [T_base, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8, T8_9]

        # 3) Accumulate and collect points
        T_cum = np.eye(4)
        points = [T_cum @ np.array([0, 0, 0, 1])]
        for T in transforms:
            T_cum = T_cum @ T
            points.append(T_cum @ np.array([0, 0, 0, 1]))

        # strip off the homogeneous 1’s
        points = np.array(points)[:, :3]

        end_point = points[-1]

        th1, th2, dr = helper.inverse_kinematics(end_point[0], end_point[1], end_point[2], verbal=True)

        expected_theta1_global = theta_1
        assert np.isclose(th1, expected_theta1_global, atol=1e-2), f"Expected {expected_theta1_global}, got {th1}"
        assert np.isclose(th2, theta_2, atol=1e-2), f"Expected {theta_2}, got {th2}"
        assert np.isclose(dr, delta_r, atol=1e-2), f"Expected {delta_r}, got {dr}"

    def test_inverce_kinematics(self):
        num_of_iterations = 100
        counter = 0
        
        for _ in range(num_of_iterations):
            counter += 1
            print("counter: ", counter)

            # Generate random delta_r, theta_1, and theta_2 values
            delta_r = np.random.uniform(0, 2000)  # Random value between -500 and 500
            theta_1 = np.random.uniform(-360, 360)     # Random angle in degrees (0 to 360)
            theta_2 = np.random.uniform(-180, 180)     # Random angle in degrees (0 to 360)

            print("delta_r", delta_r)
            print("theta_1", theta_1)
            print("theta_2", theta_2)

            # Define the initial position (in homogeneous coordinates)
            initial_position = np.array([0, 0, 0, 1]).T
            theta_r = 137.9

            T_base = np.eye(4)
            T_base[0, 3] = 925.39
            T_base[1, 3] = -219.38

            T0_1 = np.eye(4)
            T0_1[:3, :3] = R.from_euler('z', theta_r, degrees=True).as_matrix()

            T1_2 = np.eye(4)
            T1_2[1, 3] = delta_r

            # 0 -> 1
            T2_3 = np.eye(4)
            T2_3[2, 3] = 100

            # 1 -> 2
            T3_4 = np.eye(4)
            T3_4[:3, :3] = R.from_euler('z', theta_1 - theta_r, degrees=True).as_matrix()

            # 2 -> 3
            T4_5 = np.eye(4)
            T4_5[0, 3] = 57.5

            # 3 -> 4
            T5_6 = np.eye(4)
            T5_6[1, 3] = 830

            T6_7 = np.eye(4)
            T6_7[0, 3] = 107

            T7_8 = np.eye(4)
            T7_8[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

            T8_9 = np.eye(4)
            T8_9[1, 3] = 950

            transforms = [T_base, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8, T8_9]

            # 3) Accumulate and collect points
            T_cum = np.eye(4)
            points = [T_cum @ np.array([0, 0, 0, 1])]
            for T in transforms:
                T_cum = T_cum @ T
                points.append(T_cum @ np.array([0, 0, 0, 1]))

            # strip off the homogeneous 1’s
            points = np.array(points)[:, :3]

            end_point = points[-1]

            th1, th2, dr = helper.inverse_kinematics(end_point[0], end_point[1], end_point[2], 
                                                     T_base=T_base, theta_r=theta_r, verbal=True)

            expected_theta1_global = theta_1
            assert np.isclose(th1, expected_theta1_global, atol=1e-2), f"Expected {expected_theta1_global}, got {th1}"
            assert np.isclose(th2, theta_2, atol=1e-2), f"Expected {theta_2}, got {th2}"
            assert np.isclose(dr, delta_r, atol=1e-2), f"Expected {delta_r}, got {dr}"

    def test_forward_kinematics(self):
        # Define the joint angles
        theta1_deg = 0
        theta2_deg = -45
        delta_r = 1000

        # Compute the end-effector position using forward kinematics
        end_effector_pos = helper.forward_kinematics(theta1_deg, theta2_deg, delta_r)

        # Expected position based on manual calculations or known values
        expected_pos = np.array([419.46338104, 540.39560115, -571.75144213])

        # Check if the computed position is close to the expected position
        np.testing.assert_allclose(end_effector_pos[-1], expected_pos, rtol=1e-5, atol=1e-5)

if __name__ == '__main__':
    unittest.main()