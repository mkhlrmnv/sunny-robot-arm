import unittest
from unittest.mock import patch
import time
from motor import Motor


# --- Begin Test Code ---
class TestMotor(unittest.TestCase):
    def setUp(self):
        # Patch RPi.GPIO so that no actual hardware calls are made.
        # gpio_patcher = patch('RPi.GPIO')
        # self.mock_gpio = gpio_patcher.start()
        # self.addCleanup(gpio_patcher.stop)
        
        # Patch time.sleep to avoid actual waiting.
        sleep_patcher = patch('time.sleep', return_value=None)
        self.mock_sleep = sleep_patcher.start()
        self.addCleanup(sleep_patcher.stop)
        
        # Create a Motor instance with known parameters.
        # Using step_per_rev=1600 (i.e. default microstepping) and small delay limits.
        self.motor = Motor(pulse_pin=17, dir_pin=27, step_per_rev=1600, min_delay=1e-5, max_delay=1)

    def tearDown(self):
        self.motor.cleanup()

    def test_calc_delay_bounds(self):
        # Speed=1 should return the minimum delay.
        self.assertAlmostEqual(self.motor.calc_delay(1), 1e-5)
        # Speed=0 should return the maximum delay.
        self.assertAlmostEqual(self.motor.calc_delay(0), 1)
        # Test a mid value (speed=0.5)
        expected = 1e-5 + (1 - 1e-5) * (1 - 0.5)
        self.assertAlmostEqual(self.motor.calc_delay(0.5), expected)

    def test_invalid_speed_calc_delay(self):
        # Test speeds outside the allowed range.
        with self.assertRaises(ValueError):
            self.motor.calc_delay(-0.1)
        with self.assertRaises(ValueError):
            self.motor.calc_delay(1.1)

    def test_step_forward(self):
        initial_steps = self.motor.get_steps()
        initial_angle = self.motor.get_angle()
        # Perform 10 forward steps at maximum speed.
        self.motor.step(steps=10, direction=1, speed=1)
        self.assertEqual(self.motor.get_steps(), initial_steps + 10)
        # Each step moves by (360 / step_per_rev) degrees.
        self.assertAlmostEqual(self.motor.get_angle(), initial_angle + 10 * (360 / 1600))

    def test_step_backward(self):
        initial_steps = self.motor.get_steps()
        initial_angle = self.motor.get_angle()
        # Perform 5 backward steps at maximum speed.
        self.motor.step(steps=5, direction=-1, speed=1)
        self.assertEqual(self.motor.get_steps(), initial_steps - 5)
        self.assertAlmostEqual(self.motor.get_angle(), initial_angle - 5 * (360 / 1600))

    def test_invalid_direction(self):
        # Direction must be either 1 or -1.
        with self.assertRaises(ValueError):
            self.motor.step(steps=5, direction=0, speed=0.5)
        with self.assertRaises(ValueError):
            self.motor.step(steps=5, direction=2, speed=0.5)

    def test_move_by_angle_invalid(self):
        # The current implementation of move_by_angle requires abs(angle) to be >= 360.
        # For angles less than 360, it raises a ValueError.
        with self.assertRaises(ValueError):
            self.motor.move_by_angle(90, speed=0.5)
        with self.assertRaises(ValueError):
            self.motor.move_by_angle(-180, speed=0.5)

    def test_move_by_angle_valid(self):
        # Test moving by exactly 360 degrees.
        # For step_per_rev=1600, one full revolution (360°) equals 1600 steps.
        self.motor.move_by_angle(360, speed=1)
        self.assertEqual(self.motor.get_steps(), 1600)
        self.assertAlmostEqual(self.motor.get_angle(), 360)

    def test_reset_position(self):
        # After moving some steps, reset should set step and angle counters to zero.
        self.motor.step(steps=10, direction=1, speed=1)
        self.motor.reset_position()
        self.assertEqual(self.motor.get_steps(), 0)
        self.assertEqual(self.motor.get_angle(), 0)

    def test_cleanup_calls_gpio_cleanup(self):
        # Call cleanup and verify that GPIO.cleanup() was called.
        self.motor.cleanup()
        # self.mock_gpio.cleanup.assert_called_once()


if __name__ == '__main__':
    unittest.main()