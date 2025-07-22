"""
test_integration_kinematics.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Integration tests for kinematics_and_safety module.

These tests verify that the kinematics system works correctly 
with realistic robot configurations and solar tracking scenarios.
"""

import unittest
import numpy as np
import sys
import os

# Add src directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from kinematics_and_safety import (
        inverse_kinematics,
        forward_kinematics,
        choose_solution,
        check_solutions_safety
    )
    from config import (
        RAIL_ANGLE, THETA_2_VAL_IN_INIT, DELTA_R_VAL_IN_INIT,
        PONTTO_ARM_LENGTH, PAATY_ARM_LENGTH, RAIL_MAX_LIMIT, 
        RAIL_MIN_LIMIT, PONTTO_MOTOR_MIN_ANGLE
    )
except ImportError as e:
    print(f"Import error: {e}")
    print("Run from project root: python -m pytest test/ -v")


class TestKinematicsIntegration(unittest.TestCase):
    """Integration tests for the complete kinematics system."""

    def test_robot_initialization_position(self):
        """Test that robot can reach its initialization position."""
        # Standard initialization position
        theta1 = RAIL_ANGLE
        theta2 = THETA_2_VAL_IN_INIT
        delta_r = DELTA_R_VAL_IN_INIT
        
        # Forward kinematics should work without errors
        points = forward_kinematics(theta1, theta2, delta_r)
        
        # Should have reasonable end-effector position
        end_effector = points[-1]
        
        # Check that position is within reasonable bounds
        max_reach = PONTTO_ARM_LENGTH + PAATY_ARM_LENGTH + RAIL_MAX_LIMIT
        distance_from_origin = np.linalg.norm(end_effector)
        
        self.assertLess(distance_from_origin, max_reach)
        self.assertGreater(distance_from_origin, 100)  # Not too close to origin

    def test_solar_tracking_workspace(self):
        """Test typical solar tracking positions."""
        # Sample positions that might occur during solar tracking
        test_positions = [
            (-670, -1720, -430),    # Morning position  
            (-900, -700, 850),    # Noon position   
            (360, 980, 850),    # Afternoon position
            (1550, 1200, -100),   # Edge of workspace
        ]
        
        successful_solutions = 0
        
        for x, y, z in test_positions:
            with self.subTest(position=(x, y, z)):
                try:
                    # Try to solve inverse kinematics
                    solutions = inverse_kinematics(x, y, z, check_safety=False)
                    
                    # Verify by forward kinematics
                    chosen_sol = choose_solution(solutions, (RAIL_ANGLE, THETA_2_VAL_IN_INIT, 0))
                    verify_points = forward_kinematics(*chosen_sol)
                    verify_position = verify_points[-1]
                    
                    # Should be close to target
                    error = np.linalg.norm(verify_position - np.array([x, y, z]))
                    self.assertLess(error, 5.0, f"Position error: {error:.2f}mm")
                    
                    successful_solutions += 1
                    
                except ValueError:
                    # Some positions might be unreachable - that's OK
                    pass
        
        # Should be able to reach at least half of the test positions
        self.assertGreaterEqual(successful_solutions, len(test_positions) // 2)

    def test_smooth_trajectory_following(self):
        """Test that robot can follow a smooth trajectory."""
        # Create a simple linear trajectory
        start_pos = np.array([-1000, -1200, -185])
        end_pos = np.array([-1000, -710, 150])
        
        # Interpolate between start and end
        num_points = 10
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            pos = start_pos + t * (end_pos - start_pos)
            trajectory.append(pos)
        
        # Try to solve IK for each point
        joint_trajectory = []
        current_state = (RAIL_ANGLE, THETA_2_VAL_IN_INIT, 0)
        
        for pos in trajectory:
            try:
                solutions = inverse_kinematics(pos[0], pos[1], pos[2], check_safety=False)
                chosen_sol = choose_solution(solutions, current_state)
                joint_trajectory.append(chosen_sol)
                current_state = chosen_sol
                
            except ValueError:
                self.fail(f"Could not reach trajectory point {pos}")
        
        # Check that joint movements are reasonable (not too large jumps)
        for i in range(1, len(joint_trajectory)):
            prev_joints = np.array(joint_trajectory[i-1])
            curr_joints = np.array(joint_trajectory[i])
            joint_change = np.abs(curr_joints - prev_joints)
            
            # No single joint should move more than 45 degrees or 200mm
            self.assertLess(joint_change[0], 45, "Theta1 change too large")
            self.assertLess(joint_change[1], 45, "Theta2 change too large") 
            self.assertLess(joint_change[2], 200, "Rail change too large")

    def test_workspace_boundaries(self):
        """Test behavior at workspace boundaries."""
        # Test positions near the workspace limits
        boundary_tests = [
            # Near rail limits
            (500, RAIL_MAX_LIMIT - 50, 600),
            (500, 50, 600),
            
            # Near joint limits  
            (PONTTO_ARM_LENGTH + PAATY_ARM_LENGTH - 100, 400, 0),
            
            # High positions
            (600, 400, PAATY_ARM_LENGTH - 100),
            
            # Low positions
            (600, 400, -200),
        ]
        
        for x, y, z in boundary_tests:
            with self.subTest(boundary_pos=(x, y, z)):
                try:
                    solutions = inverse_kinematics(x, y, z, check_safety=False)
                    
                    # If solution found, verify it's valid
                    for sol in solutions:
                        theta1, theta2, delta_r = sol
                        
                        # Check joint limits (approximate)
                        self.assertGreaterEqual(delta_r, -1)
                        self.assertLessEqual(delta_r, RAIL_MAX_LIMIT)
                        self.assertGreaterEqual(theta2, -180)
                        self.assertLessEqual(theta2, 180)
                        
                except ValueError:
                    # Expected for some boundary positions
                    pass

    def test_safety_system_integration(self):
        """Test integration of safety checking with kinematics."""
        # Test positions that should be safe
        safe_positions = [
            (-500, 1420, -500), # safe
            (-70, 900, 800),    # safe
            (-70, 900, -400),   # safe
            (-1000, 1500, 210)  # not safe
        ]
        
        for x, y, z in safe_positions:
            with self.subTest(safe_pos=(x, y, z)):
                try:
                    # Get solutions with safety checking
                    safe_solutions = inverse_kinematics(x, y, z, check_safety=True)
                    
                    # All returned solutions should pass safety check
                    for sol in safe_solutions:
                        self.assertTrue(check_solutions_safety(sol))
                        
                except ValueError:
                    # Some positions might not have safe solutions
                    pass

    def test_repeatability(self):
        """Test that kinematics calculations are repeatable."""
        x, y, z = -650, 420, 500
        
        # Calculate multiple times
        results = []
        for _ in range(5):
            try:
                solutions = inverse_kinematics(x, y, z, check_safety=False)
                current_state = (RAIL_ANGLE, THETA_2_VAL_IN_INIT, 0)
                chosen = choose_solution(solutions, current_state)
                results.append(chosen)
            except ValueError:
                self.skipTest("Position not reachable")
        
        # All results should be identical
        if results:
            first_result = results[0]
            for result in results[1:]:
                np.testing.assert_array_almost_equal(
                    first_result, result, decimal=6,
                    err_msg="Kinematics calculations not repeatable"
                )


class TestPerformanceIntegration(unittest.TestCase):
    """Test performance with realistic workloads."""

    def test_batch_ik_performance(self):
        """Test performance with batch inverse kinematics."""
        import time
        import random
        
        # Generate batch of positions
        num_positions = 100

        positions = []
        for _ in range(num_positions):
            th1 = random.uniform(PONTTO_MOTOR_MIN_ANGLE, 360+PONTTO_MOTOR_MIN_ANGLE)
            th2 = random.uniform(-180, 180)
            dr = random.uniform(RAIL_MIN_LIMIT, RAIL_MAX_LIMIT)

            p = forward_kinematics(th1, th2, dr)[-1]
            positions.append(p)
        
        # Time the batch processing
        start_time = time.time()
        successful = 0
        
        for x, y, z in positions:
            try:
                solutions = inverse_kinematics(x, y, z, check_safety=False, verbal=False)
                successful += 1
            except ValueError:
                pass
        
        elapsed = time.time() - start_time
        
        # Should process at least 50 positions per second
        rate = successful / elapsed
        self.assertGreater(rate, 50, f"IK rate too slow: {rate:.1f} pos/sec")

    def test_safety_checking_performance(self):
        """Test performance of safety checking."""
        import time
        import random
        
        # Generate test configurations
        num_positions = 100
        configurations = []
        for _ in range(num_positions):
            th1 = random.uniform(PONTTO_MOTOR_MIN_ANGLE, 360+PONTTO_MOTOR_MIN_ANGLE)
            th2 = random.uniform(-180, 180)
            dr = random.uniform(RAIL_MIN_LIMIT, RAIL_MAX_LIMIT)
            configurations.append((th1, th2, dr))
        
        start_time = time.time()
        
        for config in configurations:
            check_solutions_safety(config)
        
        elapsed = time.time() - start_time
        
        # Should check at least 100 configurations per second
        rate = len(configurations) / elapsed
        self.assertGreater(rate, 100, f"Safety check rate too slow: {rate:.1f} checks/sec")


if __name__ == '__main__':
    unittest.main(verbosity=2)
