import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

verbal = False

# Define a function to apply a transformation matrix to a point (homogeneous coordinates)
def transform_point(point, T):
    return T @ point

# Define the initial position (in homogeneous coordinates)
initial_position = np.array([0, 0, 0, 1]).T
theta_1= 137
theta_2 = 45 # TODO: correct this

# 0 -> 1
T0_1 = np.eye(4)
T0_1[2, 3] = 100

# 1 -> 2
T1_2 = np.eye(4)
T1_2[:3, :3] = R.from_euler('z', theta_1, degrees=True).as_matrix()

# 2 -> 3
T2_3 = np.eye(4)
T2_3[0, 3] = 57.5

# 3 -> 4
T3_4 = np.eye(4)
T3_4[1, 3] = 830

T4_5 = np.eye(4)
T4_5[0, 3] = 107

T5_6 = np.eye(4)
T5_6[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

T6_7 = np.eye(4)
T6_7[1, 3] = 950

transforms = [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7]

# 3) Accumulate and collect points
T_cum = np.eye(4)
points = [T_cum @ np.array([0,0,0,1])]
for T in transforms:
    T_cum = T_cum @ T
    points.append(T_cum @ np.array([0,0,0,1]))

# strip off the homogeneous 1’s
points = np.array(points)[:,:3]

# 4) Plot
fig = plt.figure(figsize=(8,6))
ax  = fig.add_subplot(111, projection='3d')
ax.plot(points[:,0], points[:,1], points[:,2], 'r-o')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('Robot Arm Forward Kinematics')
ax.set_xlim([-1600, 1600])
ax.set_ylim([-1600, 1600])
ax.set_zlim([-1600, 1600])
plt.show()


def inverse_kinematics(x, y, z, 
                       link_rise=100,    # first Z offset
                       link_xy=(57.5+107, 830),  # combined X,Y before pitch
                       link_length=950,  # final link
                       eps=1e-6):
    """
    Returns (theta1_deg, theta2_deg) to reach (x,y,z).
    Raises ValueError if the point is out of reach.
    """
    # 1) Undo the base Z translation
    z1 = z - link_rise

    # 2) Solve for base yaw θ1
    theta1 = np.arctan2(y, x)
    Rz_inv = R.from_euler('z', -theta1, degrees=False).as_matrix()
    p1 = Rz_inv.dot(np.array([x, y, z1]))  # now expressed in joint-2 frame

    # 3) Subtract fixed XY offsets before the pitch joint
    dx, dy = link_xy
    planar = p1 - np.array([dx, dy, 0])
    y_hat, z_hat = planar[1], planar[2]

    # 4) Check reachability in the Y–Z plane
    dist = np.hypot(y_hat, z_hat)
    if dist > link_length + eps:
        raise ValueError(f"Target out of reach: distance {dist:.1f} mm > {link_length} mm")

    # 5) Solve for pitch θ2
    #    We want the 950 mm link, originally along +Y, to land on (y_hat, z_hat):
    #      y_link = link_length * cos(theta2)
    #      z_link = -link_length * sin(theta2)
    #    so
    theta2 = np.arctan2(z_hat, y_hat)

    return np.degrees(theta1), np.degrees(theta2)


# --- Example usage ---
try:
    th1, th2 = inverse_kinematics(300, 200, 400)
    print(f"θ₁ = {th1:.1f}°, θ₂ = {th2:.1f}°")

    initial_position = np.array([0, 0, 0, 1]).T
    theta_1= th1
    theta_2 = th2

    # 0 -> 1
    T0_1 = np.eye(4)
    T0_1[2, 3] = 100

    # 1 -> 2
    T1_2 = np.eye(4)
    T1_2[:3, :3] = R.from_euler('z', theta_1, degrees=True).as_matrix()

    # 2 -> 3
    T2_3 = np.eye(4)
    T2_3[0, 3] = 57.5

    # 3 -> 4
    T3_4 = np.eye(4)
    T3_4[1, 3] = 830

    T4_5 = np.eye(4)
    T4_5[0, 3] = 107

    T5_6 = np.eye(4)
    T5_6[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

    T6_7 = np.eye(4)
    T6_7[1, 3] = 950

    transforms = [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7]

    # 3) Accumulate and collect points
    T_cum = np.eye(4)
    points = [T_cum @ np.array([0,0,0,1])]
    for T in transforms:
        T_cum = T_cum @ T
        points.append(T_cum @ np.array([0,0,0,1]))

    # strip off the homogeneous 1’s
    points = np.array(points)[:,:3]

    # 4) Plot
    fig = plt.figure(figsize=(8,6))
    ax  = fig.add_subplot(111, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], 'r-o')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Robot Arm Forward Kinematics')
    ax.set_xlim([-1600, 1600])
    ax.set_ylim([-1600, 1600])
    ax.set_zlim([-1600, 1600])
    plt.show()
    
except ValueError as e:
    print("IK error:", e)
