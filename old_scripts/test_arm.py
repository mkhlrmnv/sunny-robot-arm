import unittest
import numpy as np
import time
from src import Arm, Vector, Joints
from unittest.mock import patch

class TestArm(unittest.TestCase):
    
    def setUp(self):
        """Set up a robot arm instance before each test."""
        self.base_pos = Vector(0, 0, 0)
        self.tip_pos = Vector(0, 80, 50)
        self.arm = Arm(self.base_pos, self.tip_pos)
        angles = self.arm.get_joint_angles()
        expected_angles = Joints(np.deg2rad(90), np.deg2rad(90)*3)
        self.assertEqual(angles, expected_angles)
    
    def test_get_base_pos(self):
        """Test if get_base_pos returns the correct base position."""
        self.assertEqual(self.arm.get_base_pos(), self.base_pos)
    
    def test_get_tip_pos(self):
        """Test if get_tip_pos returns the correct tip position."""
        self.assertEqual(self.arm.get_tip_pos(), self.tip_pos)
    
    def test_calc_joint_angles_valid(self):
        """Test calc_joint_angles with a reachable position."""
        target_pos = Vector(63.64, 63.64, 48.9898)
        angles = self.arm.calc_join_angles(target_pos)
        expected_angles = Joints(0.7854045668803609, 4.511031574028651)
        self.assertEqual(angles, expected_angles)
    
    def test_calc_joint_angles_unreachable(self):
        """Test calc_joint_angles with an unreachable position."""
        target_pos = Vector(200, 200, 200)  # Beyond reach
        with self.assertRaises(ValueError):
            self.arm.calc_join_angles(target_pos)

if __name__ == '__main__':
    unittest.main()