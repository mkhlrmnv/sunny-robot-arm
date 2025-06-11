import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

verbal = False

# Define the initial position (in homogeneous coordinates)
delta_r = 300
theta_r = 0
theta_1= 40
theta_2 = -45 # TODO: correct this

T_base = np.eye(4)
T_base[0, 3] = 0 # 925.39
T_base[1, 3] = 0 # -219.38

T0_1 = np.eye(4)
T0_1[:3, :3] = R.from_euler('z', theta_r, degrees=True).as_matrix()

T1_2 = np.eye(4)
T1_2[1, 3] = delta_r

# 0 -> 1
T2_3 = np.eye(4)
T2_3[2, 3] = 100

# 1 -> 2
T3_4 = np.eye(4)
T3_4[:3, :3] = R.from_euler('z', theta_1-theta_r, degrees=True).as_matrix()

# 2 -> 3
T4_5 = np.eye(4)
T4_5[0, 3] = 57.5

# 3 -> 4
T5_6 = np.eye(4)
T5_6[1, 3] = 830

T6_7 = np.eye(4)
T6_7[0, 3] = 107

T7_8 = np.eye(4)
T7_8[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

T8_9 = np.eye(4)
T8_9[1, 3] = 950

transforms = [T_base, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8, T8_9]

# 3) Accumulate and collect points
T_cum = np.eye(4)
points = [T_cum @ np.array([0,0,0,1])]
for T in transforms:
    T_cum = T_cum @ T
    points.append(T_cum @ np.array([0,0,0,1]))

# strip off the homogeneous 1’s
points = np.array(points)[:,:3]

points = points[1:]

print(points)

# 4) Plot
fig = plt.figure(figsize=(8,6))
ax  = fig.add_subplot(121, projection='3d')
ax.plot(points[:,0], points[:,1], points[:,2], 'r-o')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('Robot Arm Forward Kinematics')
ax.set_xlim([-1600, 1600])
ax.set_ylim([-1600, 1600])
ax.set_zlim([-1600, 1600])

from itertools import combinations

# Define the 8 corners of the box
box_corners = np.array([
    [0, 0, 0],
    [1820, 0, 0],
    [0, -1680, 0],
    [1820, -1680, 0],
    [0, 0, -1000],
    [1820, 0, -1000],
    [0, -1680, -1000],
    [1820, -1680, -1000]
])

# Define edges by listing pairs of points (12 box edges total)
edges = [
    (0, 1), (1, 3), (3, 2), (2, 0),  # bottom face
    (4, 5), (5, 7), (7, 6), (6, 4),  # top face
    (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
]

# Create line segments
lines = [(box_corners[start], box_corners[end]) for start, end in edges]
box = Line3DCollection(lines, colors='b', linewidths=1.5, linestyles='dashed')
ax.add_collection3d(box)
 
# plt.show()


def inverse_kinematics(x, y, z,
                    T_base=np.eye(4),
                    theta_r=0,     # rail orientation angle (deg)
                    link_rise=100,
                    rail_limits=(0, 2000),
                    dx1=57.5,
                    dx2=107,
                    dy0=830,
                    link_length=950,
                    eps=1e-6):
    print("x, y, z:", x, y, z)

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
    print("arm_reach", arm_reach)

    # Step 4: Arm base must lie on a circle of radius `arm_reach` centered at (x_r, y_r)
    # We intersect this circle with the rail line (x=0, y varies)

    # Circle center
    cx, cy = x_r, y_r
    r = arm_reach

    print("cx, cy", cx, cy)
    print("arm reach", arm_reach)

    # Intersect circle (x - cx)^2 + (y - cy)^2 = r^2
    # with line x = 0
    # → (0 - cx)^2 + (y - cy)^2 = r^2w
    # → cx^2 + (y - cy)^2 = r^2
    # → (y - cy)^2 = r^2 - cx^2
    rhs = r**2 - cx**2
    # if rhs < 0:
    #     raise ValueError("No real intersection — point unreachable horizontally.")

    y_candidates = [(2*cy+np.sqrt(4*cy**2-4*(cy**2-rhs))/2), (2*cy-np.sqrt(4*cy**2-4*(cy**2-rhs))/2)]
    print("y_candidades", y_candidates)


    # now we need to compute theta 1 => from which we can get how much delta_y i caused
    # by arm and how much is by rail 

    # Step 5: Now compute wrist point (arm base) in rail frame
    wrist_x = 0
    wrist_y = delta_r

    dx = x_r - wrist_x
    dy = y_r - wrist_y
    theta1_rad = np.arctan2(dy, dx)

    # Step 6: Convert to degrees
    theta1_deg = (np.degrees(theta1_rad) + 360) % 360
    theta2_deg = np.degrees(theta2_rad)

    return theta1_deg, theta2_deg, delta_r

# --- Example usage ---
try:
    th1, th2, dt_r = inverse_kinematics(points[-1, 0], points[-1, 1], points[-1, 2], T_base=T_base)
    print(f"θ₁ = {th1:.1f}°, θ₂ = {th2:.1f}°")

    # Define the initial position (in homogeneous coordinates)
    delta_r = dt_r
    theta_r = 137.9
    theta_1= th1
    theta_2 = th2 # TODO: correct this

    # T_base = np.eye(4)
    # T_base[0, 3] = 925.39
    # T_base[1, 3] = -219.38

    T0_1 = np.eye(4)
    T0_1[:3, :3] = R.from_euler('z', theta_r, degrees=True).as_matrix()

    T1_2 = np.eye(4)
    T1_2[1, 3] = delta_r

    # 0 -> 1
    T2_3 = np.eye(4)
    T2_3[2, 3] = 100

    # 1 -> 2
    T3_4 = np.eye(4)
    T3_4[:3, :3] = R.from_euler('z', theta_1-theta_r, degrees=True).as_matrix()

    # 2 -> 3
    T4_5 = np.eye(4)
    T4_5[0, 3] = 57.5

    # 3 -> 4
    T5_6 = np.eye(4)
    T5_6[1, 3] = 830

    T6_7 = np.eye(4)
    T6_7[0, 3] = 107

    T7_8 = np.eye(4)
    T7_8[:3, :3] = R.from_euler('x', theta_2, degrees=True).as_matrix()

    T8_9 = np.eye(4)
    T8_9[1, 3] = 950

    transforms = [T_base, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8, T8_9]

    # 3) Accumulate and collect points
    T_cum = np.eye(4)
    points = [T_cum @ np.array([0,0,0,1])]
    for T in transforms:
        T_cum = T_cum @ T
        points.append(T_cum @ np.array([0,0,0,1]))

    # strip off the homogeneous 1’s
    points = np.array(points)[:,:3]

    points = points[1:]

    print(points)

    # 4) Plot
    # fig = plt.figure(figsize=(8,6))
    ax  = fig.add_subplot(122, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], 'r-o')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Robot Arm Forward Kinematics')
    ax.set_xlim([-1600, 1600])
    ax.set_ylim([-1600, 1600])
    ax.set_zlim([-1600, 1600])

    from itertools import combinations

    # Define the 8 corners of the box
    box_corners = np.array([
        [0, 0, 0],
        [1820, 0, 0],
        [0, -1680, 0],
        [1820, -1680, 0],
        [0, 0, -1000],
        [1820, 0, -1000],
        [0, -1680, -1000],
        [1820, -1680, -1000]
    ])

    # Define edges by listing pairs of points (12 box edges total)
    edges = [
        (0, 1), (1, 3), (3, 2), (2, 0),  # bottom face
        (4, 5), (5, 7), (7, 6), (6, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
    ]

    # Create line segments
    lines = [(box_corners[start], box_corners[end]) for start, end in edges]
    box = Line3DCollection(lines, colors='b', linewidths=1.5, linestyles='dashed')
    ax.add_collection3d(box)

    plt.show()
    
except ValueError as e:
    print("IK error:", e)
