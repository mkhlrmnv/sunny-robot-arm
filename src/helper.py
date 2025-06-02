import numpy as np
from scipy.spatial.transform import Rotation as R

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
    

# helper function to compute inverse kinematics
def inverse_kinematics(x, y, z,
                       link_rise=100,    # first 100 mm up in Z
                       dx=57.5+107,      # total X before pitch joint
                       dy0=830,          # total Y before pitch joint
                       link_length=950,  # final link length
                       eps=1e-6):
    """
    Returns (theta1_deg, theta2_deg) to reach (x,y,z).
    Raises ValueError if the point is out of reach.
    """
    # 1) Undo the base Z translation
    d_z = z - link_rise
    if abs(d_z) > link_length + eps:
        raise ValueError(f"Target out of vertical reach: |z-100| = {abs(d_z):.3f} > 950")

    theta2 = np.arcsin(d_z / link_length)  # principal solution in [–90°, +90°]

    # 2) Compute the “effective Y” for Pivot+end in the XY-plane:
    #      R_proj = dy0 + link_length * cos(θ₂)
    R_proj = dy0 + link_length * np.cos(theta2)

    #    Pivot’s local bearing (in joint-2 frame) is:
    #      γ = atan2(R_proj, dx)
    gamma = np.arctan2(R_proj, dx)

    #    World bearing of the end-point is:
    #      α = atan2(y, x).
    alpha = np.arctan2(y, x)

    #    So θ₁ = α - γ:
    theta1 = alpha - gamma

    # 3) Convert both to degrees, and wrap θ₁ into [0,360) if you prefer:
    theta1_deg = (np.degrees(theta1) + 360) % 360
    theta2_deg = np.degrees(theta2)
    return theta1_deg, theta2_deg

def forward_kinematics(theta1_deg, theta2_deg,
                       link_rise=100,      # first Z offset (mm)
                       dx1=57.5,           # first X offset (mm)
                       dx2=107,            # second X offset (mm)
                       dy0=830,            # Y offset before pitch (mm)
                       link_length=950):   # final link length (mm)
    """
    Compute the (x, y, z) position of the end-effector given:
      - theta1_deg: rotation about Z at the base (in degrees)
      - theta2_deg: rotation about X at the second joint (in degrees)
    Returns a length-3 numpy array [x, y, z] in mm.
    """

    # 1) Build T0_1 = translate +link_rise along Z
    T0_1 = np.eye(4)
    T0_1[2, 3] = link_rise

    # 2) Build T1_2 = rotate about Z by theta1
    T1_2 = np.eye(4)
    T1_2[:3, :3] = R.from_euler('z', theta1_deg, degrees=True).as_matrix()

    # 3) Build T2_3 = translate +dx1 along X
    T2_3 = np.eye(4)
    T2_3[0, 3] = dx1

    # 4) Build T3_4 = translate +dx2 along X, +dy0 along Y
    T3_4 = np.eye(4)
    T3_4[0, 3] = dx2
    T3_4[1, 3] = dy0

    # 5) Build T4_5 = rotate about X by theta2
    T4_5 = np.eye(4)
    T4_5[:3, :3] = R.from_euler('x', theta2_deg, degrees=True).as_matrix()

    # 6) Build T5_6 = translate +link_length along Y
    T5_6 = np.eye(4)
    T5_6[1, 3] = link_length

    # 7) Accumulate all transforms
    transforms = [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6]
    T_cum = np.eye(4)
    for T in transforms:
        T_cum = T_cum @ T

    # 8) Apply to the origin [0,0,0,1]^T
    end_homog = T_cum @ np.array([0.0, 0.0, 0.0, 1.0])
    return end_homog[:3]   # only x, y, z

# Example usage:
if __name__ == "__main__":
    th1, th2 = inverse_kinematics(100, 100, 100)
    print(f"θ₁ = {th1:.1f}°, θ₂ = {th2:.1f}°")
    position = forward_kinematics(th1, th2)
    print(f"End-effector position: {position}")