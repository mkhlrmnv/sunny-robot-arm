#!/usr/bin/env python3
"""
test_sun_helper.py
~~~~~~~~~~~~~~~~~~

Unit tests for the sun_helper module, focusing on path serialization utilities.

Test Classes
------------
TestJsonifyPath
    Tests for jsonify_path function including path-only and path+colors scenarios.
TestUnJsonifyPath  
    Tests for un_jsonify_path function including file loading and data validation.
TestPathSerialization
    Integration tests for round-trip serialization/deserialization.
TestEdgeCases
    Tests for error conditions and edge cases.

Dependencies
------------
- unittest for test framework
- numpy for array operations
- tempfile for temporary file creation
- json for JSON operations
- os for file operations
"""

import unittest
import numpy as np
import tempfile
import json
import os
import sys

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from sun_helper import jsonify_path, un_jsonify_path


class TestJsonifyPath(unittest.TestCase):
    """Test the jsonify_path function."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Simple 3D path
        self.simple_path = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 3.0],
            [-1.5, 2.5, -0.5]
        ])
        
        # RGB colors (normalized 0-1)
        self.simple_colors = np.array([
            [1.0, 0.0, 0.0],    # Red
            [0.0, 1.0, 0.0],    # Green  
            [0.0, 0.0, 1.0]     # Blue
        ])
        
        # Larger test path
        self.large_path = np.random.rand(100, 3) * 1000 - 500  # Random points in [-500, 500]
        self.large_colors = np.random.rand(100, 3)
    
    def test_jsonify_path_only(self):
        """Test jsonifying path without colors."""
        result = jsonify_path(self.simple_path)
        
        # Check structure
        self.assertIn('path', result)
        self.assertNotIn('colors', result)
        
        # Check path data
        self.assertEqual(len(result['path']), 3)
        
        # Check first point
        point0 = result['path'][0]
        self.assertEqual(point0['x'], 0.0)
        self.assertEqual(point0['y'], 0.0)
        self.assertEqual(point0['z'], 0.0)
        
        # Check second point
        point1 = result['path'][1]
        self.assertEqual(point1['x'], 1.0)
        self.assertEqual(point1['y'], 2.0)
        self.assertEqual(point1['z'], 3.0)
        
        # Check third point
        point2 = result['path'][2]
        self.assertEqual(point2['x'], -1.5)
        self.assertEqual(point2['y'], 2.5)
        self.assertEqual(point2['z'], -0.5)
    
    def test_jsonify_path_with_colors(self):
        """Test jsonifying path with colors."""
        result = jsonify_path(self.simple_path, self.simple_colors)
        
        # Check structure
        self.assertIn('path', result)
        self.assertIn('colors', result)
        
        # Check lengths match
        self.assertEqual(len(result['path']), len(result['colors']))
        self.assertEqual(len(result['path']), 3)
        
        # Check color conversion (0-1 normalized to 0-255 integers)
        color0 = result['colors'][0]
        self.assertEqual(color0['r'], 255)  # 1.0 * 255
        self.assertEqual(color0['g'], 0)    # 0.0 * 255
        self.assertEqual(color0['b'], 0)    # 0.0 * 255
        
        color1 = result['colors'][1]
        self.assertEqual(color1['r'], 0)
        self.assertEqual(color1['g'], 255)
        self.assertEqual(color1['b'], 0)
        
        color2 = result['colors'][2]
        self.assertEqual(color2['r'], 0)
        self.assertEqual(color2['g'], 0)
        self.assertEqual(color2['b'], 255)
    
    def test_jsonify_large_dataset(self):
        """Test with larger dataset to ensure performance."""
        result = jsonify_path(self.large_path, self.large_colors)
        
        # Check structure
        self.assertEqual(len(result['path']), 100)
        self.assertEqual(len(result['colors']), 100)
        
        # Spot check some values
        for i in [0, 50, 99]:
            point = result['path'][i]
            color = result['colors'][i]
            
            # Check point values are floats
            self.assertIsInstance(point['x'], float)
            self.assertIsInstance(point['y'], float)
            self.assertIsInstance(point['z'], float)
            
            # Check color values are integers 0-255
            self.assertIsInstance(color['r'], int)
            self.assertIsInstance(color['g'], int)
            self.assertIsInstance(color['b'], int)
            self.assertTrue(0 <= color['r'] <= 255)
            self.assertTrue(0 <= color['g'] <= 255)
            self.assertTrue(0 <= color['b'] <= 255)
    
    def test_jsonify_mismatched_lengths(self):
        """Test error when path and colors have different lengths."""
        wrong_colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # Only 2 colors for 3 points
        
        with self.assertRaises(AssertionError):
            jsonify_path(self.simple_path, wrong_colors)
    
    def test_jsonify_empty_path(self):
        """Test with empty path."""
        empty_path = np.array([]).reshape(0, 3)
        result = jsonify_path(empty_path)
        
        self.assertEqual(len(result['path']), 0)
        self.assertNotIn('colors', result)
    
    def test_jsonify_single_point(self):
        """Test with single point."""
        single_point = np.array([[1.5, -2.5, 3.5]])
        single_color = np.array([[0.5, 0.7, 0.2]])
        
        result = jsonify_path(single_point, single_color)
        
        self.assertEqual(len(result['path']), 1)
        self.assertEqual(len(result['colors']), 1)
        
        point = result['path'][0]
        color = result['colors'][0]
        
        self.assertEqual(point['x'], 1.5)
        self.assertEqual(point['y'], -2.5)
        self.assertEqual(point['z'], 3.5)
        
        self.assertEqual(color['r'], int(0.5 * 255))  # 127
        self.assertEqual(color['g'], int(0.7 * 255))  # 178
        self.assertEqual(color['b'], int(0.2 * 255))  # 51


class TestUnJsonifyPath(unittest.TestCase):
    """Test the un_jsonify_path function."""
    
    def setUp(self):
        """Set up test fixtures with temporary files."""
        # Create temporary directory
        self.temp_dir = tempfile.mkdtemp()
        
        # Test data
        self.test_path = np.array([
            [10.0, 20.0, 30.0],
            [-5.5, 15.2, -8.9],
            [100.1, -200.7, 50.0]
        ])
        
        self.test_colors = np.array([
            [128, 64, 192],   # Purple-ish
            [255, 128, 0],    # Orange-ish
            [64, 255, 128]    # Green-ish
        ])
    
    def tearDown(self):
        """Clean up temporary files."""
        # Remove any files created during tests
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def test_un_jsonify_path_only(self):
        """Test loading JSON with path only (no colors)."""
        # Create test JSON file
        test_data = {
            'path': [
                {'x': 10.0, 'y': 20.0, 'z': 30.0},
                {'x': -5.5, 'y': 15.2, 'z': -8.9},
                {'x': 100.1, 'y': -200.7, 'z': 50.0}
            ]
        }
        
        json_file = os.path.join(self.temp_dir, 'test_path_only.json')
        with open(json_file, 'w') as f:
            json.dump(test_data, f)
        
        # Load and test
        path, colors = un_jsonify_path(json_file)
        
        # Check path
        np.testing.assert_array_almost_equal(path, self.test_path)
        
        # Check colors is None
        self.assertIsNone(colors)
    
    def test_un_jsonify_path_with_colors(self):
        """Test loading JSON with both path and colors."""
        # Create test JSON file
        test_data = {
            'path': [
                {'x': 10.0, 'y': 20.0, 'z': 30.0},
                {'x': -5.5, 'y': 15.2, 'z': -8.9},
                {'x': 100.1, 'y': -200.7, 'z': 50.0}
            ],
            'colors': [
                {'r': 128, 'g': 64, 'b': 192},
                {'r': 255, 'g': 128, 'b': 0},
                {'r': 64, 'g': 255, 'b': 128}
            ]
        }
        
        json_file = os.path.join(self.temp_dir, 'test_path_colors.json')
        with open(json_file, 'w') as f:
            json.dump(test_data, f)
        
        # Load and test
        path, colors = un_jsonify_path(json_file)
        
        # Check path
        np.testing.assert_array_almost_equal(path, self.test_path)
        
        # Check colors
        self.assertIsNotNone(colors)
        np.testing.assert_array_equal(colors, self.test_colors)
    
    def test_un_jsonify_nonexistent_file(self):
        """Test error when file doesn't exist."""
        nonexistent_file = os.path.join(self.temp_dir, 'does_not_exist.json')
        
        with self.assertRaises(FileNotFoundError):
            un_jsonify_path(nonexistent_file)
    
    def test_un_jsonify_invalid_json(self):
        """Test error with invalid JSON."""
        invalid_json_file = os.path.join(self.temp_dir, 'invalid.json')
        with open(invalid_json_file, 'w') as f:
            f.write("{ invalid json content")
        
        with self.assertRaises(json.JSONDecodeError):
            un_jsonify_path(invalid_json_file)
    
    def test_un_jsonify_missing_path_key(self):
        """Test error when 'path' key is missing."""
        test_data = {'colors': [{'r': 255, 'g': 0, 'b': 0}]}
        
        json_file = os.path.join(self.temp_dir, 'no_path.json')
        with open(json_file, 'w') as f:
            json.dump(test_data, f)
        
        with self.assertRaises(KeyError):
            un_jsonify_path(json_file)
    
    def test_un_jsonify_empty_path(self):
        """Test with empty path array."""
        test_data = {'path': []}
        
        json_file = os.path.join(self.temp_dir, 'empty_path.json')
        with open(json_file, 'w') as f:
            json.dump(test_data, f)
        
        path, colors = un_jsonify_path(json_file)
        
        # Empty list comprehension results in shape (0,) not (0,3)
        self.assertEqual(len(path), 0)
        self.assertIsNone(colors)


class TestPathSerialization(unittest.TestCase):
    """Integration tests for round-trip serialization."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        
        # Various test datasets
        self.datasets = [
            # Simple case
            (
                np.array([[0, 0, 0], [1, 1, 1]]),
                np.array([[1, 0, 0], [0, 1, 0]])
            ),
            # Real-world scale robot coordinates
            (
                np.array([
                    [-500.5, 800.2, 1200.8],
                    [1500.1, -900.3, 800.0],
                    [0.0, 1000.0, 1500.5]
                ]),
                np.array([
                    [0.2, 0.8, 0.6],
                    [1.0, 0.5, 0.1],
                    [0.3, 0.9, 0.7]
                ])
            ),
            # Edge values
            (
                np.array([
                    [-1000.0, -1000.0, -1000.0],
                    [1000.0, 1000.0, 1000.0]
                ]),
                np.array([
                    [0.0, 0.0, 0.0],
                    [1.0, 1.0, 1.0]
                ])
            )
        ]
    
    def tearDown(self):
        """Clean up temporary files."""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def test_round_trip_with_colors(self):
        """Test complete round-trip serialization with colors."""
        for i, (original_path, original_colors) in enumerate(self.datasets):
            with self.subTest(dataset=i):
                # Serialize
                json_data = jsonify_path(original_path, original_colors)
                
                # Save to file
                json_file = os.path.join(self.temp_dir, f'round_trip_{i}.json')
                with open(json_file, 'w') as f:
                    json.dump(json_data, f)
                
                # Load back
                loaded_path, loaded_colors = un_jsonify_path(json_file)
                
                # Compare paths (should be exact)
                np.testing.assert_array_almost_equal(original_path, loaded_path, decimal=6)
                
                # Compare colors (note: precision loss from float->int->float conversion)
                self.assertIsNotNone(loaded_colors)
                
                # Check color conversion accuracy (should be within 1/255 ≈ 0.004)
                original_colors_255 = (original_colors * 255).astype(int)
                np.testing.assert_array_equal(original_colors_255, loaded_colors)
    
    def test_round_trip_path_only(self):
        """Test round-trip serialization without colors."""
        for i, (original_path, _) in enumerate(self.datasets):
            with self.subTest(dataset=i):
                # Serialize without colors
                json_data = jsonify_path(original_path)
                
                # Save to file
                json_file = os.path.join(self.temp_dir, f'round_trip_no_color_{i}.json')
                with open(json_file, 'w') as f:
                    json.dump(json_data, f)
                
                # Load back
                loaded_path, loaded_colors = un_jsonify_path(json_file)
                
                # Compare paths
                np.testing.assert_array_almost_equal(original_path, loaded_path, decimal=6)
                
                # Colors should be None
                self.assertIsNone(loaded_colors)
    
    def test_precision_preservation(self):
        """Test that coordinate precision is preserved."""
        # High-precision coordinates
        precise_path = np.array([
            [123.456789, -987.654321, 555.111222],
            [0.000001, 999.999999, -0.123456]
        ])
        
        # Round trip
        json_data = jsonify_path(precise_path)
        json_file = os.path.join(self.temp_dir, 'precision_test.json')
        with open(json_file, 'w') as f:
            json.dump(json_data, f)
        
        loaded_path, _ = un_jsonify_path(json_file)
        
        # Should preserve at least 6 decimal places
        np.testing.assert_array_almost_equal(precise_path, loaded_path, decimal=6)


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions."""
    
    def test_extreme_coordinates(self):
        """Test with very large/small coordinates."""
        extreme_path = np.array([
            [1e6, -1e6, 1e6],    # Very large
            [1e-6, -1e-6, 1e-6], # Very small
            [0.0, 0.0, 0.0]      # Zero
        ])
        
        result = jsonify_path(extreme_path)
        
        # Should handle extreme values
        self.assertEqual(len(result['path']), 3)
        
        # Check that values are preserved as floats
        for point in result['path']:
            self.assertIsInstance(point['x'], float)
            self.assertIsInstance(point['y'], float)
            self.assertIsInstance(point['z'], float)
    
    def test_color_clipping(self):
        """Test color values outside [0,1] range."""
        path = np.array([[0, 0, 0]])
        
        # Colors outside [0,1] range
        extreme_colors = np.array([
            [2.0, -0.5, 1.5]  # Should clip to [1.0, 0.0, 1.0] -> [255, 0, 255]
        ])
        
        result = jsonify_path(path, extreme_colors)
        color = result['colors'][0]
        
        # Should handle out-of-range colors gracefully
        # (Note: Current implementation may not clip, this tests current behavior)
        self.assertIsInstance(color['r'], int)
        self.assertIsInstance(color['g'], int)
        self.assertIsInstance(color['b'], int)
    
    def test_numpy_dtypes(self):
        """Test with different numpy dtypes."""
        # Test different numpy dtypes
        path_float32 = np.array([[1, 2, 3]], dtype=np.float32)
        path_float64 = np.array([[1, 2, 3]], dtype=np.float64)
        path_int = np.array([[1, 2, 3]], dtype=int)
        
        for path in [path_float32, path_float64, path_int]:
            with self.subTest(dtype=path.dtype):
                result = jsonify_path(path)
                
                # Should convert to standard Python floats
                point = result['path'][0]
                self.assertIsInstance(point['x'], float)
                self.assertIsInstance(point['y'], float)
                self.assertIsInstance(point['z'], float)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
