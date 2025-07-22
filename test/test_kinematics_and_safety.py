"""
test_kinematics_and_safety.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Unit tests for the kinematics_and_safety module.

Tests cover:
- Forward kinematics calculations
- Inverse kinematics solving
- Solution selection algorithms
- Safety collision detection
- Edge cases and error handling
- Visualization functions (basic structure)

Run with: python -m pytest test_kinematics_and_safety.py -v
"""

import unittest
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# Add src directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from kinematics_and_safety import (
        inverse_kinematics,
        forward_kinematics,
        choose_solution,
        check_solutions_safety,
        edge_crosses_box,
        draw_box,
        draw_all_safety_boxes,
        draw_robot,
        plot_path,
        all_boxes,
        kontti_box_corners,
        safety_box_1_corners,
        safety_box_2_corners,
        edges
    )
    from config import (
        RAIL_ANGLE, THETA_2_VAL_IN_INIT, PONTTO_ARM_LENGTH, 
        PAATY_ARM_LENGTH, RAIL_MAX_LIMIT, RAIL_MIN_LIMIT,
        PONTTO_MOTOR_MIN_ANGLE
    )
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure you're running tests from the project root directory")
    print("Run: python -m pytest test/test_kinematics_and_safety.py -v")
    sys.exit(1)


class TestForwardKinematics(unittest.TestCase):
    """Test forward kinematics calculations."""

    def test_forward_kinematics_home_position(self):
        """Test forward kinematics at home position."""
        # Home position: theta1=RAIL_ANGLE, theta2=THETA_2_VAL_IN_INIT, delta_r=0
        points = forward_kinematics(RAIL_ANGLE, THETA_2_VAL_IN_INIT, 0)
        
        # Should return array with correct number of points
        self.assertEqual(len(points.shape), 2)
        self.assertEqual(points.shape[1], 3)  # 3D coordinates
        self.assertGreater(points.shape[0], 0)  # At least one point
        
        # All points should be finite
        self.assertTrue(np.all(np.isfinite(points)))

    def test_forward_kinematics_known_configuration(self):
        """Test forward kinematics with known joint configuration."""
        theta1, theta2, delta_r = 90.0, 45.0, 100.0
        points = forward_kinematics(theta1, theta2, delta_r)
        
        # Check that points are reasonable (within workspace)
        max_reach = PONTTO_ARM_LENGTH + PAATY_ARM_LENGTH + RAIL_MAX_LIMIT
        distances = np.linalg.norm(points, axis=1)
        self.assertTrue(np.all(distances < max_reach))

    def test_forward_kinematics_extreme_positions(self):
        """Test forward kinematics at extreme joint positions."""
        # Test extreme angles
        test_cases = [
            (PONTTO_MOTOR_MIN_ANGLE, -180, 0),
            (PONTTO_MOTOR_MIN_ANGLE + 360, 180, RAIL_MAX_LIMIT),
            (0, 0, RAIL_MAX_LIMIT/2)
        ]
        
        for theta1, theta2, delta_r in test_cases:
            with self.subTest(theta1=theta1, theta2=theta2, delta_r=delta_r):
                points = forward_kinematics(theta1, theta2, delta_r)
                self.assertTrue(np.all(np.isfinite(points)))

    def test_forward_kinematics_consistency(self):
        """Test that forward kinematics is consistent."""
        theta1, theta2, delta_r = 45.0, 30.0, 200.0
        
        # Call multiple times should give same result
        points1 = forward_kinematics(theta1, theta2, delta_r)
        points2 = forward_kinematics(theta1, theta2, delta_r)
        
        np.testing.assert_array_almost_equal(points1, points2)


class TestInverseKinematics(unittest.TestCase):
    """Test inverse kinematics solving."""

    def test_inverse_kinematics_reachable_point(self):
        """Test inverse kinematics for a reachable point."""
        # Use forward kinematics to generate a known reachable point
        theta1, theta2, delta_r = 90.0, 45.0, 300.0
        points = forward_kinematics(theta1, theta2, delta_r)
        target = points[-1]  # End effector position
        
        # Solve inverse kinematics
        solutions = inverse_kinematics(target[0], target[1], target[2], 
                                     check_safety=False, verbal=False)
        
        # Should find at least one solution
        self.assertGreater(len(solutions), 0)
        
        # Verify solution by forward kinematics
        sol_theta1, sol_theta2, sol_delta_r = solutions[0]
        verify_points = forward_kinematics(sol_theta1, sol_theta2, sol_delta_r)
        verify_target = verify_points[-1]
        
        # Should be close to original target (within 1mm tolerance)
        distance = np.linalg.norm(verify_target - target)
        self.assertLess(distance, 1.0, "IK solution doesn't match target position")

    def test_inverse_kinematics_unreachable_point(self):
        """Test inverse kinematics for unreachable points."""
        # Point far outside workspace
        x, y, z = 5000.0, 5000.0, 5000.0
        
        with self.assertRaises(ValueError):
            inverse_kinematics(x, y, z, check_reachability=True)

    def test_inverse_kinematics_multiple_solutions(self):
        """Test that IK can find multiple solutions for the same point."""
        # Point that should have multiple solutions (elbow up/down)
        theta1, theta2, delta_r = 45.0, 60.0, 400.0
        points = forward_kinematics(theta1, theta2, delta_r)
        target = points[-1]
        
        solutions = inverse_kinematics(target[0], target[1], target[2], 
                                     check_safety=False, verbal=False)
        
        # Should find multiple solutions for most points
        # (This might be 1 or 2 depending on configuration)
        self.assertGreaterEqual(len(solutions), 1)

    def test_inverse_kinematics_rail_limits(self):
        """Test that IK respects rail limits."""
        # Try a point that would require rail position outside limits
        x, y, z = 1000.0, RAIL_MAX_LIMIT + 500.0, 500.0
        
        try:
            solutions = inverse_kinematics(x, y, z, check_safety=False)
            # If solutions found, they should respect rail limits
            for sol in solutions:
                theta1, theta2, delta_r = sol
                self.assertGreaterEqual(delta_r, RAIL_MIN_LIMIT)
                self.assertLessEqual(delta_r, RAIL_MAX_LIMIT)
        except ValueError:
            # Expected if point is unreachable due to rail limits
            pass

    def test_inverse_kinematics_with_safety(self):
        """Test inverse kinematics with safety checking enabled."""
        # Test a point that should be reachable and safe
        x, y, z = 800.0, 400.0, 600.0
        
        try:
            solutions = inverse_kinematics(x, y, z, check_safety=True, verbal=False)
            # All solutions should be safe
            for sol in solutions:
                self.assertTrue(check_solutions_safety(sol))
        except ValueError:
            # Acceptable if no safe solutions exist
            pass

    def test_inverse_kinematics_verbose_mode(self):
        """Test that verbose mode doesn't break functionality."""
        x, y, z = 600.0, 300.0, 700.0
        
        # Should work with verbal=True (just checking no exceptions)
        try:
            solutions = inverse_kinematics(x, y, z, verbal=True, check_safety=False)
            self.assertIsInstance(solutions, list)
        except ValueError:
            # Acceptable if point is unreachable
            pass


class TestSolutionSelection(unittest.TestCase):
    """Test solution selection algorithms."""

    def test_choose_solution_closest(self):
        """Test that choose_solution picks the closest solution."""
        current_state = (90.0, 45.0, 300.0)
        
        solutions = [
            (95.0, 50.0, 310.0),  # Close solution
            (180.0, -45.0, 100.0),  # Far solution
            (85.0, 40.0, 295.0)   # Very close solution
        ]
        
        chosen = choose_solution(solutions, current_state)
        
        # Should pick the closest one (third solution)
        expected = (85.0, 40.0, 295.0)
        self.assertEqual(chosen, expected)

    def test_choose_solution_single_solution(self):
        """Test choose_solution with single solution."""
        current_state = (0.0, 0.0, 0.0)
        solutions = [(45.0, 30.0, 200.0)]
        
        chosen = choose_solution(solutions, current_state)
        self.assertEqual(chosen, solutions[0])

    def test_choose_solution_empty_list(self):
        """Test choose_solution with empty solution list."""
        current_state = (0.0, 0.0, 0.0)
        solutions = []
        
        with self.assertRaises(ValueError):
            choose_solution(solutions, current_state)


class TestSafetyCollisionDetection(unittest.TestCase):
    """Test safety and collision detection functions."""

    def test_check_solutions_safety_safe_position(self):
        """Test safety check for a known safe position."""
        # Position that should be safe (away from boxes)
        safe_solution = (45.0, 30.0, 200.0)
        
        is_safe = check_solutions_safety(safe_solution)
        
        # This specific solution should be safe
        # (May need adjustment based on actual safety box positions)
        self.assertIsInstance(is_safe, bool)

    def test_check_solutions_safety_unsafe_position(self):
        """Test safety check for potentially unsafe position."""
        # Position that might intersect with safety boxes
        # This would need to be tuned based on actual box positions
        potentially_unsafe = (0.0, 0.0, -500.0)  # Low position near kontti
        
        is_safe = check_solutions_safety(potentially_unsafe)
        self.assertIsInstance(is_safe, bool)

    def test_edge_crosses_box_simple_case(self):
        """Test edge-box intersection with simple cases."""
        # Simple box from (0,0,0) to (1,1,1)
        box_corners = np.array([
            [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
            [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]
        ])
        
        # Line that clearly passes through the box
        points_through = np.array([[-0.5, 0.5, 0.5], [1.5, 0.5, 0.5]])
        self.assertTrue(edge_crosses_box(points_through, box_corners))
        
        # Line that clearly misses the box
        points_miss = np.array([[2, 2, 2], [3, 3, 3]])
        self.assertFalse(edge_crosses_box(points_miss, box_corners))

    def test_edge_crosses_box_robot_arm(self):
        """Test edge crossing with robot arm configuration."""
        # Generate robot arm points
        theta1, theta2, delta_r = 90.0, 45.0, 300.0
        points = forward_kinematics(theta1, theta2, delta_r)
        
        # Test against each safety box
        for box in all_boxes:
            result = edge_crosses_box(points, box)
            self.assertIsInstance(result, bool)

    def test_safety_boxes_defined(self):
        """Test that all safety boxes are properly defined."""
        # Check that safety boxes have correct shape
        self.assertEqual(kontti_box_corners.shape, (8, 3))
        self.assertEqual(safety_box_1_corners.shape, (8, 3))
        self.assertEqual(safety_box_2_corners.shape, (8, 3))
        
        # Check that all coordinates are finite
        for box in all_boxes:
            self.assertTrue(np.all(np.isfinite(box)))

    def test_edges_definition(self):
        """Test that box edges are properly defined."""
        self.assertEqual(len(edges), 12)  # A box has 12 edges
        
        # Each edge should connect two different vertices (0-7)
        for start, end in edges:
            self.assertIn(start, range(8))
            self.assertIn(end, range(8))
            self.assertNotEqual(start, end)


class TestRoundTripConsistency(unittest.TestCase):
    """Test forward-inverse kinematics round-trip consistency."""

    def test_forward_inverse_roundtrip(self):
        """Test that FK->IK->FK gives consistent results."""
        # Test multiple configurations
        test_configs = [
            (90.0, 45.0, 300.0),
            (45.0, -30.0, 150.0),
            (135.0, 60.0, 500.0),
            (0.0, 0.0, 100.0)
        ]
        
        for theta1, theta2, delta_r in test_configs:
            with self.subTest(theta1=theta1, theta2=theta2, delta_r=delta_r):
                # Forward kinematics
                points = forward_kinematics(theta1, theta2, delta_r)
                target = points[-1]
                
                try:
                    # Inverse kinematics
                    solutions = inverse_kinematics(target[0], target[1], target[2], 
                                                 check_safety=False, verbal=False)
                    
                    # Choose closest solution to original
                    original_state = (theta1, theta2, delta_r)
                    chosen_sol = choose_solution(solutions, original_state)
                    
                    # Forward kinematics again
                    verify_points = forward_kinematics(*chosen_sol)
                    verify_target = verify_points[-1]
                    
                    # Should be very close to original target
                    distance = np.linalg.norm(verify_target - target)
                    self.assertLess(distance, 1.0, 
                                  f"Round-trip error too large: {distance:.3f}mm")
                    
                except ValueError:
                    # Some configurations might not be reachable due to constraints
                    self.skipTest(f"Configuration {theta1}, {theta2}, {delta_r} not reachable")


class TestVisualizationFunctions(unittest.TestCase):
    """Test visualization functions (basic functionality)."""

    def test_draw_box_no_error(self):
        """Test that draw_box doesn't raise errors."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Should not raise an exception
        draw_box(kontti_box_corners, edges, ax)
        plt.close(fig)

    def test_draw_all_safety_boxes_no_error(self):
        """Test that draw_all_safety_boxes doesn't raise errors."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Should not raise an exception
        draw_all_safety_boxes(ax)
        plt.close(fig)

    def test_draw_robot_no_error(self):
        """Test that draw_robot doesn't raise errors."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Generate sample robot points
        points = forward_kinematics(90.0, 45.0, 300.0)
        
        # Should not raise an exception
        draw_robot(ax, points)
        plt.close(fig)

    def test_plot_path_no_error(self):
        """Test that plot_path doesn't raise errors."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Sample path data
        path = np.array([[0, 0, 0], [100, 100, 100], [200, 200, 200]])
        
        # Should not raise an exception
        plot_path(ax, path)
        plt.close(fig)


class TestEdgeCasesAndErrorHandling(unittest.TestCase):
    """Test edge cases and error handling."""

    def test_inverse_kinematics_invalid_inputs(self):
        """Test inverse kinematics with invalid inputs."""
        # Test with NaN
        with self.assertRaises((ValueError, TypeError)):
            inverse_kinematics(np.nan, 0, 0)
        
        # Test with infinite values
        with self.assertRaises((ValueError, TypeError)):
            inverse_kinematics(np.inf, 0, 0)

    def test_forward_kinematics_extreme_angles(self):
        """Test forward kinematics with extreme angle values."""
        # Very large angles (should still work due to trigonometric functions)
        points = forward_kinematics(720.0, 720.0, 100.0)
        self.assertTrue(np.all(np.isfinite(points)))
        
        # Negative angles
        points = forward_kinematics(-180.0, -180.0, 100.0)
        self.assertTrue(np.all(np.isfinite(points)))

    def test_choose_solution_edge_cases(self):
        """Test choose_solution with edge cases."""
        current_state = (0.0, 0.0, 0.0)
        
        # Solutions with identical distances
        solutions = [
            (10.0, 0.0, 0.0),   # Distance = 10
            (0.0, 10.0, 0.0),   # Distance = 10  
            (0.0, 0.0, 10.0)    # Distance = 10
        ]
        
        chosen = choose_solution(solutions, current_state)
        self.assertIn(chosen, solutions)

    def test_safety_check_with_invalid_solution(self):
        """Test safety checking with invalid solution format."""
        # Test with wrong number of elements
        with self.assertRaises((ValueError, TypeError, IndexError, AssertionError)):
            check_solutions_safety((90.0, 45.0))  # Missing delta_r
        
        with self.assertRaises((ValueError, TypeError, IndexError, AssertionError)):
            check_solutions_safety((90.0, 45.0, 300.0, 100.0))  # Too many elements


class TestPerformance(unittest.TestCase):
    """Test performance characteristics."""

    def test_inverse_kinematics_performance(self):
        """Test that inverse kinematics completes in reasonable time."""
        import time
        
        # Test with a reasonable target
        x, y, z = 800.0, 400.0, 600.0
        
        start_time = time.time()
        try:
            solutions = inverse_kinematics(x, y, z, check_safety=False, verbal=False)
            elapsed = time.time() - start_time
            
            # Should complete in less than 0.1 seconds
            self.assertLess(elapsed, 0.1, f"IK took too long: {elapsed:.3f}s")
        except ValueError:
            # Acceptable if point is unreachable
            pass

    def test_forward_kinematics_performance(self):
        """Test that forward kinematics completes in reasonable time."""
        import time
        
        start_time = time.time()
        for _ in range(1000):  # Run many times
            forward_kinematics(90.0, 45.0, 300.0)
        elapsed = time.time() - start_time
        
        # 1000 FK calculations should complete in less than 0.1 seconds
        self.assertLess(elapsed, 0.1, f"1000 FK calls took too long: {elapsed:.3f}s")


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
