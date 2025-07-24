#!/usr/bin/env python3
"""
test_move_arm_api.py
~~~~~~~~~~~~~~~~~~~

Comprehensive unit tests for the Flask API endpoints in the sunny-robot-arm project.

Test Classes
------------
TestMoveArmAPI
    Tests for all move_arm endpoint commands including manual control,
    motor control, position control, and initialization.
TestAPIHelpers
    Tests for helper functions and error conditions.
TestAPIIntegration
    Integration tests that verify end-to-end API functionality.

Dependencies
------------
- unittest for test framework
- unittest.mock for mocking hardware interactions
- flask for testing Flask app
- json for JSON response parsing
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import json
import sys
import os

# Add src and ui directories to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(project_root, 'src'))
sys.path.insert(0, os.path.join(project_root, 'ui'))

# Mock hardware-dependent modules before importing main
sys.modules['spinning_joints'] = Mock()
sys.modules['linear_rail'] = Mock()
sys.modules['arm'] = Mock()
sys.modules['cooling'] = Mock()

# Import main after mocking dependencies
import main


class TestMoveArmAPI(unittest.TestCase):
    """Test the /move_arm API endpoint with all commands."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create Flask test client
        main.app.config['TESTING'] = True
        self.client = main.app.test_client()
        
        # Mock the arm object and shared state
        self.mock_arm = Mock()
        self.mock_shared = Mock()
        
        # Set up mock motor objects
        self.mock_arm.motor_paaty = Mock()
        self.mock_arm.motor_pontto = Mock()
        self.mock_arm.motor_rail = Mock()
        
        # Mock shared state values
        self.mock_shared.theta_1 = 45.0
        self.mock_shared.theta_2 = -30.0
        self.mock_shared.delta_r = 500.0
        
        # Replace global objects in main module
        main.arm = self.mock_arm
        main.shared = self.mock_shared
        main.angles_per_key = 10
        
        # Mock process control functions
        self.start_arm_patcher = patch('main.start_arm')
        self.start_arm_and_wait_patcher = patch('main.start_arm_and_wait')
        self.is_arm_running_patcher = patch('main.is_arm_running')
        self.stop_arm_patcher = patch('main.stop_arm')
        
        self.mock_start_arm = self.start_arm_patcher.start()
        self.mock_start_arm_and_wait = self.start_arm_and_wait_patcher.start()
        self.mock_is_arm_running = self.is_arm_running_patcher.start()
        self.mock_stop_arm = self.stop_arm_patcher.start()
        
        # Set default return values
        self.mock_is_arm_running.return_value = False
        self.mock_start_arm_and_wait.return_value = 0  # Success
        
        # Mock kinematics functions
        self.fk_patcher = patch('main.forward_kinematics')
        self.ik_patcher = patch('main.inverse_kinematics')
        self.choose_solution_patcher = patch('main.choose_solution')
        
        self.mock_fk = self.fk_patcher.start()
        self.mock_ik = self.ik_patcher.start()
        self.mock_choose_solution = self.choose_solution_patcher.start()
        
        # Set up kinematics mock returns
        self.mock_fk.return_value = [
            [0, 0, 0],           # Base
            [100, 0, 200],       # Joint 1
            [200, 100, 400],     # Joint 2
            [300, 200, 600]      # End effector
        ]
        self.mock_ik.return_value = [(45.0, -30.0, 500.0)]
        self.mock_choose_solution.return_value = (45.0, -30.0, 500.0)
    
    def tearDown(self):
        """Clean up patches."""
        self.start_arm_patcher.stop()
        self.start_arm_and_wait_patcher.stop()
        self.is_arm_running_patcher.stop()
        self.stop_arm_patcher.stop()
        self.fk_patcher.stop()
        self.ik_patcher.stop()
        self.choose_solution_patcher.stop()
    
    def test_manual_control_paaty_up(self):
        """Test manual control command: motor_paaty_up."""
        response = self.client.get('/move_arm?cmd=motor_paaty_up')
        data = json.loads(response.data)
        
        self.assertEqual(response.status_code, 200)
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor paaty moving up', data['message'])
        self.assertEqual(data['step_size'], 10)
        
        # Verify start_arm was called with correct parameters
        self.mock_start_arm.assert_called_once()
        args = self.mock_start_arm.call_args
        self.assertEqual(args[0][0], self.mock_arm.motor_paaty.move_by_angle)
        self.assertEqual(args[0][1], (10, 0.5))  # angles_per_key, speed
    
    def test_manual_control_paaty_down(self):
        """Test manual control command: motor_paaty_down."""
        response = self.client.get('/move_arm?cmd=motor_paaty_down')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor paaty moving down', data['message'])
        
        # Check negative angle
        args = self.mock_start_arm.call_args
        self.assertEqual(args[0][1], (-10, 0.5))  # negative angles_per_key
    
    def test_manual_control_pontto_ccw(self):
        """Test manual control command: motor_pontto_ccw."""
        response = self.client.get('/move_arm?cmd=motor_pontto_ccw')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor pontto moving ccw', data['message'])
        
        args = self.mock_start_arm.call_args
        self.assertEqual(args[0][0], self.mock_arm.motor_pontto.move_by_angle)
    
    def test_manual_control_pontto_cw(self):
        """Test manual control command: motor_pontto_cw."""
        response = self.client.get('/move_arm?cmd=motor_pontto_cw')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor pontto moving cw', data['message'])
    
    def test_manual_control_rail_movements(self):
        """Test manual control commands: motor_rail_right and motor_rail_left."""
        # Test right movement
        response = self.client.get('/move_arm?cmd=motor_rail_right')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor rail moving right', data['message'])
        
        # Test left movement
        response = self.client.get('/move_arm?cmd=motor_rail_left')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor rail moving left', data['message'])
    
    def test_step_size_control(self):
        """Test step size increase/decrease commands."""
        # Test increase
        response = self.client.get('/move_arm?cmd=pl')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Step per key increased to 20', data['message'])
        self.assertEqual(data['step_size'], 20)
        
        # Test decrease
        response = self.client.get('/move_arm?cmd=mn')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Step per key decreased to 10', data['message'])
        self.assertEqual(data['step_size'], 10)
    
    def test_set_step_size(self):
        """Test set_step_size command."""
        # Test valid step size
        response = self.client.get('/move_arm?cmd=set_step_size&to=25')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Step size set to 25', data['message'])
        self.assertEqual(data['step_size'], 25)
        
        # Test invalid step size (negative)
        response = self.client.get('/move_arm?cmd=set_step_size&to=-5')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn('Step size must be positive', data['message'])
        
        # Test invalid step size (non-integer)
        response = self.client.get('/move_arm?cmd=set_step_size&to=abc')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
    
    def test_by_angle_pontto_with_safety(self):
        """Test by_angle command for pontto motor with safety check."""
        response = self.client.get('/move_arm?cmd=by_angle&motor=pontto&angle=15.5&speed=0.2&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows the target angle (current + delta), which is shared.theta_1 = 45.0 from mock
        self.assertIn('Motor pontto moved by 15.5° to 45.0 (with safety check)', data['message'])
        
        # Verify kinematics calls
        self.mock_fk.assert_called()
        self.mock_ik.assert_called()
        self.mock_choose_solution.assert_called()
    
    def test_by_angle_paaty_with_safety(self):
        """Test by_angle command for paaty motor with safety check."""
        response = self.client.get('/move_arm?cmd=by_angle&motor=paaty&angle=-10.0&speed=0.3&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows the current theta_2 from shared state (-30.0 from mock)
        self.assertIn('Motor paaty moved by -10.0° to -30.0 (with safety check)', data['message'])
    
    def test_by_angle_without_safety(self):
        """Test by_angle command without safety check."""
        response = self.client.get('/move_arm?cmd=by_angle&motor=pontto&angle=5.0&speed=0.1&check_safety=0')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Motor pontto moved by 5.0°', data['message'])
        
        # Verify motor method was called directly
        self.mock_start_arm_and_wait.assert_called()
    
    def test_by_angle_invalid_motor(self):
        """Test by_angle command with invalid motor name."""
        response = self.client.get('/move_arm?cmd=by_angle&motor=invalid&angle=10&speed=0.1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn('Invalid motor: invalid', data['message'])
    
    def test_by_angle_safety_failure(self):
        """Test by_angle command when safety check fails."""
        # Mock choose_solution to return different values (safety failure)
        self.mock_choose_solution.return_value = (45.0, -25.0, 500.0)  # Different theta_2
        
        response = self.client.get('/move_arm?cmd=by_angle&motor=pontto&angle=15&speed=0.1&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn('Safety check failed', data['message'])
    
    def test_by_angle_ik_failure(self):
        """Test by_angle command when inverse kinematics fails."""
        self.mock_ik.side_effect = ValueError("Point unreachable")
        
        response = self.client.get('/move_arm?cmd=by_angle&motor=pontto&angle=180&speed=0.1&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn('Inverse kinematics failed: Point unreachable', data['message'])
    
    def test_to_angle_pontto(self):
        """Test to_angle command for pontto motor."""
        response = self.client.get('/move_arm?cmd=to_angle&motor=pontto&angle=60.0&speed=0.4&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows current shared.theta_1 (45.0) in both origin and final position
        self.assertIn('Motor pontto moved from 45.0 to 45.0 (with safety check)', data['message'])
    
    def test_to_angle_paaty(self):
        """Test to_angle command for paaty motor."""
        response = self.client.get('/move_arm?cmd=to_angle&motor=paaty&angle=-45.0&speed=0.6&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows current shared.theta_2 (-30.0) in both origin and final position
        self.assertIn('Motor paaty moved from -30.0 to -30.0 (with safety check)', data['message'])
    
    def test_by_distance_with_safety(self):
        """Test by_distance command with safety check."""
        self.mock_arm.motor_rail.distance = 500.0
        self.mock_arm.theta_1 = 45.0
        self.mock_arm.theta_2 = -30.0
        self.mock_arm.delta_r = 500.0
        
        response = self.client.get('/move_arm?cmd=by_distance&dist=100.0&speed=0.2&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows current shared.delta_r (500.0) as the result
        self.assertIn('Rail moved by 100.0 to 500.0 (with safety check)', data['message'])
    
    def test_by_distance_without_safety(self):
        """Test by_distance command without safety check."""
        response = self.client.get('/move_arm?cmd=by_distance&dist=50.0&speed=0.3&check_safety=0')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Rail moved by 50.0 to new distance:', data['message'])
    
    def test_to_distance_with_safety(self):
        """Test to_distance command with safety check."""
        self.mock_arm.theta_1 = 45.0
        self.mock_arm.theta_2 = -30.0
        self.mock_arm.delta_r = 500.0
        
        response = self.client.get('/move_arm?cmd=to_distance&dist=600.0&speed=0.5&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows current shared.delta_r (500.0) as both origin and result
        self.assertIn('Rail moved to 500.0 from 500.0 (with safety check)', data['message'])
    
    def test_to_distance_without_safety(self):
        """Test to_distance command without safety check."""
        response = self.client.get('/move_arm?cmd=to_distance&dist=700.0&speed=0.4&check_safety=0')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Rail moved by 700.0 to new distance:', data['message'])
    
    def test_to_point(self):
        """Test to_point command."""
        response = self.client.get('/move_arm?cmd=to_point&x=300&y=200&z=600&check_safety=1&speed_rail=0.6&speed_joints=0.2')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message format in actual code is different - it shows the forward kinematics result
        self.assertIn('Arm moved from ((300, 200, 600) to (300, 200, 600)', data['message'])
        
        # Verify kinematics calls
        self.mock_ik.assert_called_with(300.0, 200.0, 600.0, verbal=False)
        self.mock_choose_solution.assert_called()
    
    def test_to_point_ik_failure(self):
        """Test to_point command when inverse kinematics fails."""
        self.mock_ik.side_effect = ValueError("Point unreachable")
        
        response = self.client.get('/move_arm?cmd=to_point&x=9999&y=9999&z=9999')
        data = json.loads(response.data)

        print("data: ", data)
        
        self.assertEqual(data['status'], 'error')
        # The actual error message format from the code
        self.assertIn('Inverse kinematics failed: Point unreachable', data['message'])
    
    def test_to_angles(self):
        """Test to_angles command."""
        response = self.client.get('/move_arm?cmd=to_angles&theta_1=50.0&theta_2=-35.0&delta_r=550.0&check_safety=1&speed_rail=0.7&speed_joints=0.3')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        # The message shows the current shared state values (45.0, -30.0, 500.0) not the requested ones
        self.assertIn('Arm moved to angles: 45.0, -30.0, 500.0 (with safety check)', data['message'])
        
        # Verify forward kinematics was called
        self.mock_fk.assert_called_with(50.0, -35.0, 550.0)
    
    def test_init_command(self):
        """Test init command."""
        response = self.client.get('/move_arm?cmd=init')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'ok')
        self.assertIn('Arm initialized and motor set to angles:', data['message'])
        self.assertIn('theta_1 -> ', data['message'])
        self.assertIn('theta_2 -> ', data['message'])
        self.assertIn('delta_r -> ', data['message'])
        
        # Verify arm.init was called
        self.mock_start_arm_and_wait.assert_called()
        args = self.mock_start_arm_and_wait.call_args
        self.assertEqual(args[0][0], self.mock_arm.init)
    
    def test_init_command_failure(self):
        """Test init command when initialization fails."""
        self.mock_start_arm_and_wait.return_value = 1  # Failure
        
        response = self.client.get('/move_arm?cmd=init')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn("One of the motors couldn't initialize", data['message'])
    
    def test_unknown_command(self):
        """Test unknown command."""
        response = self.client.get('/move_arm?cmd=invalid_command')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertEqual(data['message'], 'Unknown command')
    
    def test_missing_parameters(self):
        """Test commands with missing parameters."""
        # Test by_angle without motor parameter
        response = self.client.get('/move_arm?cmd=by_angle&angle=10')
        
        # Should result in an error (either 400 or 500 status, or error in JSON)
        self.assertTrue(response.status_code >= 400 or 
                       (response.status_code == 200 and 
                        json.loads(response.data).get('status') == 'error'))
    
    def test_arm_running_stop_behavior(self):
        """Test that running arm is stopped before new commands."""
        self.mock_is_arm_running.return_value = True
        
        response = self.client.get('/move_arm?cmd=motor_paaty_up')
        data = json.loads(response.data)
        
        # Should still succeed after stopping
        self.assertEqual(data['status'], 'ok')
        # Verify stop_arm was called
        self.mock_stop_arm.assert_called_once()
    
    def test_movement_failure_return_codes(self):
        """Test handling of non-zero return codes from movements."""
        self.mock_start_arm_and_wait.return_value = 1  # Failure code
        
        response = self.client.get('/move_arm?cmd=by_angle&motor=pontto&angle=10&check_safety=1')
        data = json.loads(response.data)
        
        self.assertEqual(data['status'], 'error')
        self.assertIn('Function returned with exit code 1', data['message'])


class TestAPIHelpers(unittest.TestCase):
    """Test helper functions and edge cases."""
    
    def setUp(self):
        """Set up test fixtures."""
        main.app.config['TESTING'] = True
        self.client = main.app.test_client()
    
    def test_float_parameter_parsing(self):
        """Test that float parameters are parsed correctly."""
        with patch('main.arm'), patch('main.shared') as mock_shared, patch('main.start_arm_and_wait') as mock_wait:
            mock_wait.return_value = 0
            # Set up shared state
            mock_shared.theta_1 = 45.0
            mock_shared.theta_2 = -30.0
            mock_shared.delta_r = 500.0
            
            # Test various float formats
            test_cases = [
                ('10.5', 10.5),
                ('0.1', 0.1),
                ('-5.7', -5.7),
                ('100', 100.0),  # Integer should be converted to float
            ]
            
            for param_value, expected_value in test_cases:
                with self.subTest(param_value=param_value):
                    with patch('main.forward_kinematics') as mock_fk, \
                         patch('main.inverse_kinematics') as mock_ik, \
                         patch('main.choose_solution') as mock_choose:
                        
                        # Set up proper return values
                        mock_fk.return_value = [[300, 200, 600]]
                        mock_ik.return_value = [(45.0, -30.0, 500.0)]
                        mock_choose.return_value = (45.0, -30.0, 500.0)
                        
                        response = self.client.get(f'/move_arm?cmd=by_angle&motor=pontto&angle={param_value}&check_safety=0')
                        data = json.loads(response.data)
                        
                        # Should not error on float parsing
                        self.assertNotEqual(data['status'], 'error')
    
    def test_boolean_parameter_parsing(self):
        """Test that boolean parameters are parsed correctly."""
        with patch('main.arm'), patch('main.shared') as mock_shared, patch('main.start_arm_and_wait') as mock_wait:
            mock_wait.return_value = 0
            # Set up shared state
            mock_shared.theta_1 = 45.0
            mock_shared.theta_2 = -30.0
            mock_shared.delta_r = 500.0
            
            # Test boolean conversion from string
            test_cases = [
                ('1', True),
                ('0', False),
            ]
            
            for param_value, expected_bool in test_cases:
                with self.subTest(param_value=param_value):
                    with patch('main.forward_kinematics') as mock_fk, \
                         patch('main.inverse_kinematics') as mock_ik, \
                         patch('main.choose_solution') as mock_choose:
                        
                        # Set up proper return values
                        mock_fk.return_value = [[300, 200, 600]]
                        mock_ik.return_value = [(45.0, -30.0, 500.0)]
                        mock_choose.return_value = (45.0, -30.0, 500.0)
                        
                        response = self.client.get(f'/move_arm?cmd=by_angle&motor=pontto&angle=10&check_safety={param_value}')
                        data = json.loads(response.data)
                        
                        # Should not error on boolean parsing
                        self.assertNotEqual(data['status'], 'error')


class TestAPIIntegration(unittest.TestCase):
    """Integration tests for API functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        main.app.config['TESTING'] = True
        self.client = main.app.test_client()
    
    @patch('main.arm')
    @patch('main.shared')
    @patch('main.start_arm')
    @patch('main.start_arm_and_wait')
    @patch('main.is_arm_running')
    @patch('main.stop_arm')
    def test_api_workflow(self, mock_stop, mock_running, mock_wait, mock_start, mock_shared, mock_arm):
        """Test a complete API workflow: init -> move -> stop."""
        # Setup mocks
        mock_running.return_value = False
        mock_wait.return_value = 0
        mock_shared.theta_1 = 0.0
        mock_shared.theta_2 = 0.0
        mock_shared.delta_r = 500.0
        
        # Step 1: Initialize
        response = self.client.get('/move_arm?cmd=init')
        data = json.loads(response.data)
        self.assertEqual(data['status'], 'ok')
        
        # Step 2: Move to specific angles
        with patch('main.forward_kinematics') as mock_fk:
            mock_fk.return_value = [[300, 200, 600]]
            response = self.client.get('/move_arm?cmd=to_angles&theta_1=45&theta_2=-30&delta_r=550&check_safety=1')
            data = json.loads(response.data)
            self.assertEqual(data['status'], 'ok')
        
        # Step 3: Manual adjustment
        response = self.client.get('/move_arm?cmd=motor_paaty_up')
        data = json.loads(response.data)
        self.assertEqual(data['status'], 'ok')
        
        # Verify calls were made in sequence
        self.assertTrue(mock_wait.called)
        self.assertTrue(mock_start.called)
    
    def test_api_response_format(self):
        """Test that all API responses follow the expected format."""
        with patch('main.arm'), patch('main.shared'), patch('main.start_arm'):
            # Test various commands
            commands = [
                'motor_paaty_up',
                'pl',
                'mn',
                'init',
                'unknown_command'
            ]
            
            for cmd in commands:
                with self.subTest(command=cmd):
                    response = self.client.get(f'/move_arm?cmd={cmd}')
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.content_type, 'application/json')
                    
                    data = json.loads(response.data)
                    
                    # Check required fields
                    self.assertIn('status', data)
                    self.assertIn('message', data)
                    self.assertIn('step_size', data)
                    
                    # Check status values
                    self.assertIn(data['status'], ['ok', 'error'])
                    
                    # Check message is string
                    self.assertIsInstance(data['message'], str)
                    
                    # Check step_size is integer
                    self.assertIsInstance(data['step_size'], int)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
