"""
kinematics_and_safety.py
~~~~~~~~~~~~~~~~~~~~~~~~~

Robotic arm kinematics and safety utilities.

This module provides:
  - Inverse kinematics for a 2-DOF arm on a linear rail,
  - Forward kinematics to compute end-effector points,
  - Collision checks against predefined safety boxes,
  - Utilities to draw the robot and safety zones using matplotlib.

Dependencies:
  - numpy, scipy.spatial.transform.Rotation
  - matplotlib (3D), mpl_toolkits.mplot3d.art3d.Line3DCollection
  - pvlib (for solar path), pandas
  - sun_helper.get_sun_path, alt_to_color
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import pvlib
import pandas as pd
import os
import json

from config import *
from sun_helper import get_sun_path, alt_to_color, jsonify_path, un_jsonify_path


# -----------------------------------------------------------------------------
# Inverse Kinematics
# -----------------------------------------------------------------------------

def inverse_kinematics(x: float,
                       y: float, 
                       z: float,
                       T_base: list[list[float]] = BASE_TRANSFORM_MATRIX,
                       theta_r: float = RAIL_ANGLE,             # rail orientation (deg)
                       link_rise: float = PONTTO_Z_OFFSET,      # vertical offset of first joint (mm)
                       rail_limits: tuple[float, float] = (RAIL_MIN_LIMIT, 
                                                           RAIL_MAX_LIMIT),
                       dx1: float = PONTTO_X_OFFSET,            # x-offset first link (mm)
                       dx2: float = PAATY_X_OFFSET,             # x-offset second link (mm)
                       dy0: float = PONTTO_ARM_LENGTH,          # first arm length (mm)
                       link_length: float = PAATY_ARM_LENGTH,   # second arm length (mm)  
                       eps: float = 1e-6, 
                       check_reachability: bool = True,
                       check_safety: bool = True,
                       verbal: bool = False) -> list[tuple[float,float,float]]:
    """
    Solve for joint angles and rail position to reach (x, y, z).

    Returns a list of feasible solutions: (theta1_deg, theta2_deg, delta_r).
    Raises ValueError if no solution.
    """

     # 1) Transform world coordinates into robot base frame
    T_inv = np.linalg.inv(T_base)
    p_world = np.array([x, y, z, 1])
    p_local = T_inv @ p_world
    x_l, y_l, z_l = p_local[:3]

    # 2) Rotate into rail coordinate (rail along +Y)
    Rz = R.from_euler('z', theta_r, degrees=True).as_matrix()
    p_rail = Rz.T @ np.array([x_l, y_l, z_l])
    x_r, y_r, z_r = p_rail

    # 3) Compute effective z by subtracting link rise
    z_eff = z_r - link_rise
    if check_reachability and abs(z_eff) > link_length + eps:
        raise ValueError(f"Z offset {z_eff:.2f} exceeds vertical reach {link_length}")

    # horizontal distance for circle of radius = link_length
    horiz = np.sqrt(max(link_length**2 - z_eff**2, 0.0))

    # two possible elbow configurations
    theta_2a_rad = np.arctan2(z_eff,  horiz)
    theta_2b_rad = np.arctan2( z_eff, -horiz)   # elbow-up

    solutions: list[tuple[float,float,float]] = []

    # fixed geometry
    arm_reach_x = dx1 + dx2

    if verbal:
        print("\n-- Inverse Kinematics Debug --")
        print(f"\tInput target      : x={x}, y={y}, z={z}")
        print(f"\tBase‐frame coords : x_l={x_l:.2f}, y_l={y_l:.2f}, z_l={z_l:.2f}")
        print(f"\tRail‐frame coords : x_r={x_r:.2f}, y_r={y_r:.2f}, z_r={z_r:.2f}")
        print(f"\tz_eff={z_eff:.2f}, horiz={horiz:.2f}")
        print(f"\tθ2 candidates     : {np.degrees(theta_2a_rad):.2f}°, {np.degrees(theta_2b_rad):.2f}°")

    for theta_2 in (theta_2a_rad, theta_2b_rad):
        if verbal:
            print(f"\n\tTesting θ2 = {np.degrees(theta_2):.2f}°:")

        # y-position of wrist: two intersections of circle and x-plane
        arm_reach_y = dy0 + link_length * np.cos(theta_2)
        r = np.hypot(arm_reach_x, arm_reach_y)
        rhs = r**2 - x_r**2
        if rhs < 0: 
            continue        # no intersection -> this theta_2 isn't valid solution

        if verbal:
            print(f"\t  arm_reach_y={arm_reach_y:.2f}, r={r:.2f}, rhs={rhs:.2f}")
        

        for sign in (+1, -1):
            y_wrist = y_r + sign * np.sqrt(rhs)

            # check rail limits
            if verbal:
                print(f"\t  y_wrist[{sign:+}] = {y_wrist:.2f}", end="")

            if not (rail_limits[0] <= y_wrist <= rail_limits[1]):
                continue        # y_wrist is outside the limits -> y_wrist isn't valid

            if verbal: print(" ✓ within limits")

            # angle from base to wrist projection
            dx = x_r - 0
            dy = y_r - y_wrist
            alpha = np.arctan2(dy, dx)
            gamma = np.arctan2(arm_reach_y, dx1 + dx2)
            theta_1 = alpha - gamma

            # convert and wrap angles into degrees domain
            theta_1_deg = (((np.degrees(theta_1) + theta_r) - (PONTTO_MOTOR_MIN_ANGLE)) % 360) + (PONTTO_MOTOR_MIN_ANGLE)
            theta_2_deg = ((np.degrees(theta_2) + 180) % 360) - 180

            if verbal:
                print(f"\t    α={np.degrees(alpha):.2f}°, γ={np.degrees(gamma):.2f}°")
                print(f"\t    raw θ1={np.degrees(theta_1):.2f}°, wrapped θ1={theta_1_deg:.2f}°")
                print(f"\t    wrapped θ2={theta_2_deg:.2f}°, delta_r={y_wrist:.2f}")

            sol = (theta_1_deg, theta_2_deg, y_wrist)

            # optional safety filter
            if check_safety:
                if check_solutions_safety(sol, all_boxes):
                    solutions.append(sol)
                    if verbal:
                        print(f"\t    ✓ Valid solution: {sol}")
            else:
                solutions.append(sol)

    if not solutions:
        raise ValueError("No valid IK solution within limits")
    
    if verbal:
        print("\n\tFinal IK Solutions (θ1°, θ2°, rail δ):")
        for idx, (th1, th2, dr) in enumerate(solutions, start=1):
            print(f"\t  #{idx}: θ1 = {th1:.2f}°, θ2 = {th2:.2f}°, δ = {dr:.2f}")

    return solutions


# -----------------------------------------------------------------------------
# Choose closest solution
# -----------------------------------------------------------------------------

def choose_solution(solutions: list[tuple[float,float,float]], 
                    current_state: list[tuple[float,float,float]]
                    ) -> tuple[float,float,float]:
    """
    Pick the IK solution minimizing sum of absolute joint + rail changes.
    """
    c_th1, c_th2, c_dr = current_state
    best = min(solutions,
               key=lambda sol: abs(sol[0]-c_th1) + abs(sol[1]-c_th2) + abs(sol[2]-c_dr))
    return best


# -----------------------------------------------------------------------------
# Forward Kinematics
# -----------------------------------------------------------------------------

def forward_kinematics(theta1_deg: float, 
                       theta2_deg: float, 
                       delta_r: float, 
                       theta_r: float = RAIL_ANGLE,             # angle of the rails
                       link_rise: float = PONTTO_Z_OFFSET,      # first Z offset (mm)
                       dx1: float = PONTTO_X_OFFSET,            # first X offset (mm)
                       dx2: float = PAATY_X_OFFSET,             # second X offset (mm)
                       dy0: float = PONTTO_ARM_LENGTH,          # Y offset before pitch (mm)
                       link_length: float = PAATY_ARM_LENGTH    # final link length (mm)
                    ) -> np.array:
    """
    Compute XYZ positions of each joint (and end-effector) in world frame.
        - theta1_deg: rotation about Z at the base (in degrees)
        - theta2_deg: rotation about X at the second joint (in degrees)
        - delta_r: rail slide from the base (in mm)

    Returns an (N x 3) array of points along kinematic chain. AKA returns all points of the robot
    """

    # assemble transforms
    transforms = []

    # base translation
    Tb = BASE_TRANSFORM_MATRIX
    transforms.append(Tb)

    # rail rotation
    T0 = np.eye(4); T0[:3,:3] = R.from_euler('z', theta_r, True).as_matrix()
    transforms.append(T0)

    # rail slide
    T1 = np.eye(4); T1[1,3] = delta_r
    transforms.append(T1)

    # link rise / pontto rise
    T2 = np.eye(4); T2[2,3] = link_rise
    transforms.append(T2)

    # pontto twist
    T3 = np.eye(4); T3[:3,:3] = R.from_euler('z', theta1_deg-theta_r, True).as_matrix()
    transforms.append(T3)

    # first joint / pontto x-offset
    T4 = np.eye(4); T4[0,3] = dx1
    transforms.append(T4)

    # translation to second link base / to paaty
    T5 = np.eye(4); T5[1,3] = dy0
    transforms.append(T5)
    T6 = np.eye(4); T6[0,3] = dx2
    transforms.append(T6)

    # second joint / paaty pitch 
    T7 = np.eye(4); T7[:3,:3] = R.from_euler('x', theta2_deg, True).as_matrix()
    transforms.append(T7)

    # translations to end of the second arm
    T8 = np.eye(4); T8[1,3] = link_length
    transforms.append(T8)

    # accumulate all points
    Tcum = np.eye(4)
    points = []
    for T in transforms:
        Tcum = Tcum @ T
        points.append(Tcum @ np.array([0,0,0,1]))

    # strip off the homogeneous 1’s
    pts = np.array(points)[:,:3]

    # return all expect the first points as it always in the origin
    return pts[1:]


# -----------------------------------------------------------------------------
# Collision & Safety Checks
# -----------------------------------------------------------------------------

# Definition of the 8 corner the kontti -> container on top of which robot is
kontti_box_corners = np.array([
    [0, 0, -1],
    [1820, 0, -1],
    [0, -1680, -1],
    [1820, -1680, -1],
    [0, 0, -1000],
    [1820, 0, -1000],
    [0, -1680, -1000],
    [1820, -1680, -1000]
])

# Definition of the 8 corner first of safety box that is 1.2m away from the kontti in x-direction
safety_box_1_corners = np.array([
    [-2000, 1200, -1000],
    [-2000, 2000, -1000],
    [1820, 1200, -1000],
    [1820, 2000, -1000],
    [-2000, 1200, 1000],
    [-2000, 2000, 1000],
    [1820, 1200, 1000],
    [1820, 2000, 1000]
])

# Definition of the 8 corner first of safety box that is 1m away from the kontti in negative y-direction
safety_box_2_corners = np.array([
    [-2000, 1000, -1000],
    [-1100, 1000, -1000],
    [-2000, -1680, -1000],
    [-1100, -1680, -1000],
    [-2000, 1000, 1000],
    [-1100, 1000, 1000],
    [-2000, -1680, 1000],
    [-1100, -1680, 1000]
])

# Definition of the array that stores all required boxes
all_boxes = [kontti_box_corners, safety_box_1_corners, safety_box_2_corners]


# Define edges by listing pairs of points (12 box edges total)
edges: list[tuple[int, int]] = [
    (0, 1), (1, 3), (3, 2), (2, 0),  # bottom face
    (4, 5), (5, 7), (7, 6), (6, 4),  # top face
    (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
]


def edge_crosses_box(points: np.array, box_corners: np.array) -> bool:
    """Check if any segment between consecutive points intersects the box."""

    # Get box min/max bounds
    min_corner = np.min(box_corners, axis=0)
    max_corner = np.max(box_corners, axis=0)

    def segment_intersects_box(p1, p2):
        # Liang-Barsky algorithm for line-box intersection in 3D
        d = p2 - p1
        tmin = 0.0
        tmax = 1.0

        for i in range(3):  # x, y, z
            if abs(d[i]) < 1e-8:
                if p1[i] < min_corner[i] or p1[i] > max_corner[i]:
                    return False  # Line parallel and outside slab
            else:
                ood = 1.0 / d[i]
                t1 = (min_corner[i] - p1[i]) * ood
                t2 = (max_corner[i] - p1[i]) * ood
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
                if tmin > tmax:
                    return False  # No intersection
        return True

    # Check each edge of the robot arm
    for i in range(len(points) - 1):
        if segment_intersects_box(points[i], points[i + 1]):
            return True

    return False


def check_solutions_safety(solutions: tuple[float,float,float], 
                           safety_boxes: list[np.array] = all_boxes
                        ) -> bool:
    """Return True if FK path for sol does not intersect any safety box."""
    pts = forward_kinematics(*solutions)
    return not any(edge_crosses_box(pts, box) for box in safety_boxes)


# -----------------------------------------------------------------------------
# Visualization Helpers
# -----------------------------------------------------------------------------

def draw_box(box_corners: np.array, 
             edges: list[tuple[int, int]], 
             ax, 
             color: str = 'b', 
             linestyle: str = 'dashed', 
             linewidth: float = 1.5) -> None:
    
    """Draw the 12 edges of a box on a 3D Axes."""
    lines = [(box_corners[start], box_corners[end]) for start, end in edges]
    line_collection = Line3DCollection(lines, colors=color, linewidths=linewidth, linestyles=linestyle)
    ax.add_collection3d(line_collection)


def draw_all_safety_boxes(ax) -> None:
    """Draw all predefined safety boxes"""
    draw_box(kontti_box_corners, edges=edges, ax=ax,  color='g')

    for box in (safety_box_1_corners, safety_box_2_corners):
        draw_box(box, edges=edges, ax=ax, color='r')


def draw_robot(ax, points: np.array, color: str = 'g'):
    ax.plot(points[:,0], points[:,1], points[:,2], f'{color}-o')


def plot_path(ax, path,
             color='blue',
             marker='.',
             linestyle='-',
             label="Sun's path"):
    
    ax.plot(path[:, 0], path[:, 1], path[:, 2], marker=marker, linestyle=linestyle,
             color=color, label=label)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')


# Example usage:
if __name__ == "__main__":

    # sols = inverse_kinematics(48.90380303, -2467.93789029, -561.66937223)

    import time

    start_time = time.time()
    # suns_path, unreachable_points, _ = get_sun_path(R=1700)
    finish_time = time.time()
    print(f"Sun path computed in {finish_time - start_time:.2f} seconds")
    test_path = np.array([[377.04045089,  391.49348017, 1115.], [-968.47394546, -372.88213625,  662.5]])
    # test_path = np.load('paths/test_path.npy')

    json_res = jsonify_path(test_path)
    file_path = os.path.join(os.path.dirname(__file__), "..", "paths", "test_path.json")
    with open(file_path, "w") as f:
        json.dump(json_res, f, indent=4)


    # np.save('paths/finish_sun_path.json', suns_path)

    counter = 0
    # sols = inverse_kinematics(*suns_path[34], verbal=False)
    sols = inverse_kinematics(*test_path[0], verbal=True)

    # print("safe", check_solutions_safety(sols[0], all_boxes)

    # th1, th2, dr = [30, 30, 400]
    th1, th2, dr = sols[0]

    points = forward_kinematics(th1, th2, dr)

    print(points[-1])

    print(f"Points: {points[-1]}")

    for box in all_boxes:
        if edge_crosses_box(points, box):
            print("POINTS IS IN THE BOOOOOOX")

    fig = plt.figure(figsize=(8,6))
    ax_1 = fig.add_subplot(111, projection='3d')
    draw_all_safety_boxes(ax_1)


    plot_path(ax_1, test_path, linestyle='None')
    # plot_path(ax_1, suns_path)
    # plot_path(ax_1, unreachable_points, color='red', label='unreachable')
    
    draw_robot(ax_1, points=points)

    plt.show()
