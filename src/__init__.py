# src/__init__.py

# Explicitly expose modules for easier imports
from .linear_rail import LinearRail
from .spinning_joints import SpinningJoints
from .cooling import FanController
from .arm import Arm
from .helper import Vector, Joints, forward_kinematics, inverse_kinematics
from .cooling import FanController

# Optional: Define package metadata
__version__ = "1.0"
__author__ = "Mikhail"