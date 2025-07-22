#!/usr/bin/env python3
"""
run_tests.py
~~~~~~~~~~~~

Test runner for the sunny-robot-arm project.

This script sets up the proper Python path and runs the kinematics tests.

Usage:
    python test/run_tests.py                              # Run kinematics unit tests
    python test/run_tests.py --verbose                    # Verbose output  
    python test/run_tests.py --integration               # Run integration tests
    python test/run_tests.py --sun-helper                # Run sun_helper utility tests
    python test/run_tests.py --all                       # Run all test modules
    python test/run_tests.py --specific TestForwardKinematics  # Run specific test class
    python test/run_tests.py --method test_forward_kinematics_home_position  # Run specific test method
"""

import sys
import os
import argparse
import unittest

# Add the src directory to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_path = os.path.join(project_root, 'src')
sys.path.insert(0, src_path)

def main():
    parser = argparse.ArgumentParser(description='Run tests for sunny-robot-arm')
    parser.add_argument('--verbose', '-v', action='store_true', 
                       help='Verbose output')
    parser.add_argument('--specific', '-s', type=str,
                       help='Run specific test class (e.g., TestForwardKinematics)')
    parser.add_argument('--method', '-m', type=str,
                       help='Run specific test method (e.g., test_forward_kinematics_home_position)')
    parser.add_argument('--integration', '-i', action='store_true',
                       help='Run integration tests instead of unit tests')
    parser.add_argument('--all', '-a', action='store_true',
                       help='Run all test modules (kinematics, sun_helper, integration)')
    parser.add_argument('--sun-helper', action='store_true',
                       help='Run sun_helper utility tests')
    
    args = parser.parse_args()
    
    # Import test modules
    try:
        if args.all:
            # Import all test modules and create combined suite
            import test_kinematics_and_safety
            import test_sun_helper
            import test_integration_kinematics
            
            loader = unittest.TestLoader()
            combined_suite = unittest.TestSuite()
            
            combined_suite.addTests(loader.loadTestsFromModule(test_kinematics_and_safety))
            combined_suite.addTests(loader.loadTestsFromModule(test_sun_helper))
            combined_suite.addTests(loader.loadTestsFromModule(test_integration_kinematics))
            
            print("✓ Successfully imported all test modules")
            
            # Run combined suite
            verbosity = 2 if args.verbose else 1
            runner = unittest.TextTestRunner(verbosity=verbosity)
            result = runner.run(combined_suite)
            
            return 0 if result.wasSuccessful() else 1
            
        elif args.integration:
            import test_integration_kinematics
            test_module = test_integration_kinematics
            print("✓ Successfully imported integration test module")
        elif args.sun_helper or (args.specific and 'SunHelper' in args.specific):
            import test_sun_helper
            test_module = test_sun_helper
            print("✓ Successfully imported sun_helper test module")
        else:
            import test_kinematics_and_safety
            test_module = test_kinematics_and_safety
            print("✓ Successfully imported kinematics unit test module")
    except ImportError as e:
        print(f"✗ Failed to import test modules: {e}")
        print("\nMake sure you have all dependencies installed:")
        print("pip install numpy scipy matplotlib pvlib pandas")
        print("\nAnd make sure you're running from the project root directory")
        return 1
    
    # Create test suite
    loader = unittest.TestLoader()
    
    if args.specific:
        # Run specific test class
        try:
            test_class = getattr(test_module, args.specific)
            suite = loader.loadTestsFromTestCase(test_class)
        except AttributeError:
            print(f"✗ Test class '{args.specific}' not found")
            return 1
    elif args.method:
        # Run specific test method
        try:
            suite = loader.loadTestsFromName(args.method, test_module)
        except AttributeError:
            print(f"✗ Test method '{args.method}' not found")
            return 1
    else:
        # Run all tests in the module
        suite = loader.loadTestsFromModule(test_module)
    
    # Run tests
    verbosity = 2 if args.verbose else 1
    runner = unittest.TextTestRunner(verbosity=verbosity)
    result = runner.run(suite)
    
    # Return appropriate exit code
    return 0 if result.wasSuccessful() else 1

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
