import numpy as np

class Vector:
    def __init__(self, x, y, z):
        self.x = x 
        self.y = y
        self.z = z
    def __eq__(self, other):
        """Overrides the == operator for Vector comparison."""
        if isinstance(other, Vector):  # Ensure 'other' is also a Vector
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False  # If 'other' is not a Vector, return False
    
    def __str__(self):
        """Returns a string representation of the Vector."""
        return f"Vector(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

class Joints:
    def __init__(self, theta1, theta2):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta1_deg = np.rad2deg(theta1)
        self.theta2_deg = np.rad2deg(theta2)

    def __eq__(self, other):
        if isinstance(other, Joints):
            return self.theta1 == other.theta1 and self.theta2 == other.theta2
        return False

    def __str__(self):
        return f"Joints(theta1={self.theta1:.2f} rad, theta2={self.theta2:.2f} rad, " \
               f"theta1_deg={self.theta1_deg:.2f}°, theta2_deg={self.theta2_deg:.2f}°)"