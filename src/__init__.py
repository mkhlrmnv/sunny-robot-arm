# src/__init__.py

# Explicitly expose modules for easier imports
from .linear_rail import LinearRail
from .spinning_joints import SpinningJoints
from .cooling import FanController
from .arm import Arm
from .kinematics_and_safety import *
from .cooling import FanController
from .sun_helper import *
from .lamp import Lamp
from config import *

# Optional: Define package metadata
__version__ = "1.0"
__author__ = "Mikhail"