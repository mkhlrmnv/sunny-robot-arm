# test/__init__.py

import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

# Import test modules explicitly (optional)
# from .test_motor import TestMotor
# from .test_arm import TestArm