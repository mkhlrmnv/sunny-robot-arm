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
                    theta_r=137.9,     # rail orientation angle (deg)
                    link_rise=100,
                    rail_limits=(0, 2000),
                    dx1=57.5,
                    dx2=107,
                    dy0=830,
                    link_length=950,
                    eps=1e-6, 
                    verbal=False):
    """
    Compute the inverse kinematics for a robotic arm to reach a point (x, y, z).
    Returns the joint angles (theta1, theta2) in degrees and the delta_r for the rail.
    """

    # Step 1: Bring world point into robot base frame
    T_inv = np.linalg.inv(T_base)
    p_world = np.array([x, y, z, 1])
    p_local = T_inv @ p_world
    x_l, y_l, z_l = p_local[:3]

    # Step 2: Rotate point into rail frame (rail lies along +Y direction)
    Rz = R.from_euler('z', theta_r, degrees=True).as_matrix()
    p_rail = Rz.T @ np.array([x_l, y_l, z_l])
    x_r, y_r, z_r = p_rail

    # Step 3: From z_r, solve for theta2
    z_eff = z_r - link_rise
    if abs(z_eff) > link_length + eps:
        raise ValueError(f"Z offset {z_eff:.2f} exceeds vertical reach {link_length}")

    theta2_rad = np.arcsin(z_eff / link_length)
    arm_reach_y = dy0 + link_length * np.cos(theta2_rad)   # horizontal reach
    arm_reach_x = dx1 + dx2

    arm_reach = np.sqrt(arm_reach_x**2 + arm_reach_y**2)
    
    # Step 4: Arm base must lie on a circle of radius `arm_reach` centered at (x_r, y_r)
    # We intersect this circle with the rail line (x=0, y varies)

    # Circle center
    cx, cy = x_r, y_r
    r = arm_reach

    # Intersect circle (x - cx)^2 + (y - cy)^2 = r^2
    # with line x = 0
    # → (0 - cx)^2 + (y - cy)^2 = r^2w
    # → cx^2 + (y - cy)^2 = r^2
    # → (y - cy)^2 = r^2 - cx^2
    rhs = r**2 - cx**2
    if rhs < 0:
        raise ValueError("No real intersection — point unreachable horizontally.")

    y_candidates = [cy + np.sqrt(rhs), cy - np.sqrt(rhs)]
    
    # Filter candidates to be within rail limits
    y_candidates = [y for y in y_candidates if rail_limits[0] <= y <= rail_limits[1]]
    if not y_candidates:
        raise ValueError("No valid rail intersection within limits.")
    
    # now we need to compute theta 1 => from which we can get how much delta_y i caused
    # by arm and how much is by rail 

    # Step 5: Now compute wrist point (arm base) in rail frame
    wrist_x = 0
    wrist_y = y_candidates[0]

    dx = x_r - wrist_x
    dy = y_r - wrist_y
    alpha = np.arctan2(dy, dx)

    R_proj = dy0 + link_length * np.cos(theta2_rad)
    gamma = np.arctan2(R_proj, dx1 + dx2)

    theta1_rad = alpha - gamma

    # Step 6: Convert to degrees
    theta1_deg = (np.degrees(theta1_rad) + 360 + theta_r) % 360
    theta2_deg = np.degrees(theta2_rad)

    if verbal:
        print("Inverse Kinematics Debug Info:")
        print("\tx, y, z:", x, y, z)
        print("\tarm_reach", arm_reach)
        print("\tcx, cy", cx, cy)
        print("\tarm reach", arm_reach)
        print("\ty_candidades", y_candidates)
        print("\ty_candidates after filtering", y_candidates[0])

    return theta1_deg, theta2_deg, delta_r

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