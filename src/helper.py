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
                       T_base=[[1, 0, 0, 925.39], [0, 1, 0, -219.38], [0, 0, 1, 0], [0, 0, 0, 1]],
                       link_rise=100,
                       dx1=57.5,
                       dx2=107,
                       dy0=830,
                       link_length=950,
                       eps=1e-6):
    """
    Computes (theta1_deg, theta2_deg) to reach (x, y, z) in world coordinates,
    using a simplified robot model with fixed base and no rail motion.
    """
    # 1. Transform target point to local robot frame
    T_inv = np.linalg.inv(T_base)
    target_world = np.array([x, y, z, 1])
    target_local = T_inv @ target_world
    x_l, y_l, z_l = target_local[:3]

    # 2. Adjust for Z offset (link_rise)
    z_eff = z_l - link_rise
    if abs(z_eff) > link_length + eps:
        raise ValueError(f"Target Z offset {z_eff:.2f} exceeds link length {link_length}")

    # 3. Compute theta2 from Z component
    theta2_rad = np.arcsin(z_eff / link_length)

    # 4. Compute horizontal projection to base of pitch joint
    R_proj = dy0 + link_length * np.cos(theta2_rad)
    gamma = np.arctan2(R_proj, dx1 + dx2)
    alpha = np.arctan2(y_l, x_l)
    theta1_rad = alpha - gamma

    # 5. Convert to degrees
    theta1_deg = (np.degrees(theta1_rad) + 360) % 360
    theta2_deg = np.degrees(theta2_rad)

    return theta1_deg, theta2_deg

def forward_kinematics(theta1_deg, theta2_deg, delta_r, 
                       theta_r=137.9,      # angle of the rails
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

    T_base = np.eye(4)
    T_base[0, 3] = 925.39
    T_base[1, 3] = -219.38

    T0_1 = np.eye(4)
    T0_1[:3, :3] = R.from_euler('z', theta_r, degrees=True).as_matrix()

    T1_2 = np.eye(4)
    T1_2[1, 3] = delta_r 

    # 0 -> 1
    T2_3 = np.eye(4)
    T2_3[2, 3] = link_rise

    # 1 -> 2
    T3_4 = np.eye(4)
    T3_4[:3, :3] = R.from_euler('z', theta1_deg-theta_r, degrees=True).as_matrix()

    # 2 -> 3
    T4_5 = np.eye(4)
    T4_5[0, 3] = dx1

    # 3 -> 4
    T5_6 = np.eye(4)
    T5_6[1, 3] = dy0

    T6_7 = np.eye(4)
    T6_7[0, 3] = dx2

    T7_8 = np.eye(4)
    T7_8[:3, :3] = R.from_euler('x', theta2_deg, degrees=True).as_matrix()

    T8_9 = np.eye(4)
    T8_9[1, 3] = link_length

    transforms = [T_base, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8, T8_9]

    # 3) Accumulate and collect points
    T_cum = np.eye(4)
    points = [T_cum @ np.array([0,0,0,1])]
    for T in transforms:
        T_cum = T_cum @ T
        points.append(T_cum @ np.array([0,0,0,1]))

    # strip off the homogeneous 1’s
    points = np.array(points)[:,:3]

    return points[1:]

# Example usage:
if __name__ == "__main__":
    th1, th2 = inverse_kinematics(100, 100, 100)
    print(f"θ₁ = {th1:.1f}°, θ₂ = {th2:.1f}°")
    position = forward_kinematics(th1, th2)
    print(f"End-effector position: {position}")