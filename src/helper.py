import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import pvlib
import pandas as pd

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
                    rail_limits=(-0.1, 718),
                    dx1=57.5,
                    dx2=107,
                    dy0=830,
                    link_length=950,
                    eps=1e-6, 
                    check_reachability=True,
                    check_safety=True,
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

    if check_reachability and abs(z_eff) > link_length + eps:
        raise ValueError(f"Z offset {z_eff:.2f} exceeds vertical reach {link_length}")

    horiz = np.sqrt(max(link_length**2 - z_eff**2, 0.0))

    # solving two possible theta2 angles (a and b)
    theta_2a_rad = np.arctan2(z_eff,  horiz)
    theta_2b_rad = np.arctan2( z_eff, -horiz)   # elbow-up

    solutions = []

    cx, cy = x_r, y_r
    arm_reach_x = dx1 + dx2

    if verbal:
        print("\n-- Inverse Kinematics Debug --")
        print(f"\tInput target      : x={x}, y={y}, z={z}")
        print(f"\tBase‐frame coords : x_l={x_l:.2f}, y_l={y_l:.2f}, z_l={z_l:.2f}")
        print(f"\tRail‐frame coords : x_r={x_r:.2f}, y_r={y_r:.2f}, z_r={z_r:.2f}")
        print(f"\tCircle center      : cx={cx:.2f}, cy={cy:.2f}")
        print(f"\tz_eff={z_eff:.2f}, horiz={horiz:.2f}")
        print(f"\tθ2 candidates     : {np.degrees(theta_2a_rad):.2f}°, {np.degrees(theta_2b_rad):.2f}°")

    for theta_2 in (theta_2a_rad, theta_2b_rad):

        if verbal:
            print(f"\n\tTesting θ2 = {np.degrees(theta_2):.2f}°:")

        arm_reach_y = dy0 + link_length * np.cos(theta_2)
        r = np.hypot(arm_reach_x, arm_reach_y)

        
        rhs = r**2 - cx**2
        if rhs < 0: 
            continue

        if verbal:
            print(f"\t  arm_reach_y={arm_reach_y:.2f}, r={r:.2f}, rhs={rhs:.2f}")
        
        for sign in (+1, -1):
            y_wrist = cy + sign * np.sqrt(rhs)

            if verbal:
                print(f"\t  y_wrist[{sign:+}] = {y_wrist:.2f}", end="")

            if not (rail_limits[0] <= y_wrist <= rail_limits[1]):
                continue

            if verbal: print(" ✓ within limits")

            dx = x_r - 0
            dy = y_r - y_wrist
            alpha = np.arctan2(dy, dx)
            gamma = np.arctan2(arm_reach_y, dx1 + dx2)
            theta_1 = alpha - gamma

            # wrap into [-180,180]
            theta_1_deg = (((np.degrees(theta_1) + theta_r) - (-172)) % 360) + (-172)
            theta_2_deg = ((np.degrees(theta_2) + 180) % 360) - 180

            if verbal:
                print(f"\t    α={np.degrees(alpha):.2f}°, γ={np.degrees(gamma):.2f}°")
                print(f"\t    raw θ1={np.degrees(theta_1):.2f}°, wrapped θ1={theta_1_deg:.2f}°")
                print(f"\t    wrapped θ2={theta_2_deg:.2f}°, delta_r={y_wrist:.2f}")

            sol = (theta_1_deg, theta_2_deg, y_wrist)

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



def choose_solution(solutions, current_state):
    # TODO: check if this works + make it cleaner
    c_th1, c_th2, c_dr = current_state

    best_sol = solutions[0]
    best_distance = 1e10

    if len(solutions) == 1:
        return solutions[0]

    for sol in solutions[0:]:
        th1, th2, dr = sol

        new_distance = abs(th1 - c_th1) + abs(th2 - c_th2) + abs(dr - c_dr)

        if new_distance < best_distance:
            best_distance = new_distance
            best_sol = sol

    return best_sol



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

# Safety Box 1
safety_box_1_corners = np.array([
    [-2000, 1000, -1000],
    [-2000, 2000, -1000],
    [1820, 1000, -1000],
    [1820, 2000, -1000],
    [-2000, 1000, 1000],
    [-2000, 2000, 1000],
    [1820, 1000, 1000],
    [1820, 2000, 1000]
])

# Safety Box 2
safety_box_2_corners = np.array([
    [-2000, 1000, -1000],
    [-1000, 1000, -1000],
    [-2000, -1680, -1000],
    [-1000, -1680, -1000],
    [-2000, 1000, 1000],
    [-1000, 1000, 1000],
    [-2000, -1680, 1000],
    [-1000, -1680, 1000]
])

test_box = np.array([
    [0, 0, -1000],
    [0, 100, -1000],
    [-100, 0, -1000],
    [-100, 100, -1000],
    [0, 0, 1000],
    [0, 100, 1000],
    [-100, 0, 1000],
    [-100, 100, 1000]
])

all_boxes = [kontti_box_corners, safety_box_1_corners, safety_box_2_corners] #, test_box]

    # Define edges by listing pairs of points (12 box edges total)
edges = [
    (0, 1), (1, 3), (3, 2), (2, 0),  # bottom face
    (4, 5), (5, 7), (7, 6), (6, 4),  # top face
    (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
]

def check_solutions_safety(solutions, safety_boxes=all_boxes):

    """
    Check if the given solutions are safe, i.e., they do not cross any safety boxes.
    
    Args:
        solutions (list): List of tuples containing (theta1, theta2, delta_r).
        safety_boxes (list): List of numpy arrays representing safety box corners.
        
    Returns:
        bool: True if all solutions are safe, False otherwise.
    """
    points = forward_kinematics(*solutions)
    for box in safety_boxes:
        if edge_crosses_box(points, box):
            return False
    return True

def draw_box(box_corners, edges, ax, color='b', linestyle='dashed', linewidth=1.5):
    lines = [(box_corners[start], box_corners[end]) for start, end in edges]
    line_collection = Line3DCollection(lines, colors=color, linewidths=linewidth, linestyles=linestyle)
    ax.add_collection3d(line_collection)


def draw_all_safety_boxes(ax):
    draw_box(kontti_box_corners, edges=edges, ax=ax,  color='r')

    for box in (safety_box_1_corners, safety_box_2_corners, test_box):
        draw_box(box, edges=edges, ax=ax, color='r')

def draw_robot(ax, points, color='g'):
    ax.plot(points[:,0], points[:,1], points[:,2], f'{color}-o')

def point_in_box(point, box):
    """
    Returns True if a single 3-D point lies inside an axis-aligned box.

    Parameters
    ----------
    point : array-like of shape (3,)
        The candidate point [x, y, z].
    box   : ndarray of shape (8, 3)
        The eight corner vertices of the box.

    Notes
    -----
    The function works for any ordering of the eight vertices; it
    first finds the box’s component-wise min/max, then checks:

        box_min <= point <= box_max   (element-wise)

    """
    point = np.asarray(point)
    box_min = box.min(axis=0)
    box_max = box.max(axis=0)

    return np.all(point >= box_min) and np.all(point <= box_max)

def edge_crosses_box(points, box_corners):
    """
    Check if any segment between points crosses the safety box.
    
    Args:
        points (np.array): Nx3 array of robot points.
        box_corners (np.array): 8x3 array of safety box corner points.
        
    Returns:
        bool: True if any edge crosses the box, False otherwise.
    """

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

def get_sun_path(R=1700,
             max_iteration=1000,
             latitude=60.1699, 
             longitude=24.9384,
             timezone = 'Europe/Helsinki'):

    # 1) Define timespan
    # Here we define from the day start to the end, cause we
    # will get rid off the night time in step 2
    times = pd.date_range(
        end  ='2025-06-21 23:59',   
        start='2025-06-21 00:00',   
        freq='10min',
        tz=timezone
    )

    # 2) Compute solar position (altitude & azimuth) and 
    # remove the night time
    location = pvlib.location.Location(latitude, longitude, timezone)
    solpos = location.get_solarposition(times)
    solpos = solpos.loc[solpos['apparent_elevation'] > 0, :] #remove night time => remove all where altitude is below 0
    alt = solpos['apparent_elevation']  # degrees above horizon
    az  = solpos['azimuth']             # degrees clockwise from North

    # 3) Convert to unit‐sphere Cartesian for 3D plotting
    #    X axis → East, Y → North, Z → Up
    x = R * np.cos(np.radians(alt)) * np.sin(np.radians(az + 135)) + (1820/2)
    y = R * np.cos(np.radians(alt)) * np.cos(np.radians(az + 135)) - (1680/2)
    z = R * np.sin(np.radians(alt))

    # 4) Shift the path
    # First shift it to start and end in the center of the box (kontti),
    # then shift it so that none of the points are in the box

    req_x = -800
    req_y = 800
    req_z = -850

    # Shift to the center
    x_shift = req_x - x.iloc[int(len(x)/2)]
    y_shift = req_y - y.iloc[int(len(y)/2)]
    z_shift = req_z - np.min([z.iloc[0], z.iloc[len(z)-1]])

    x += x_shift
    y += y_shift
    z += z_shift

    # Shift to outside of the box
    x_shift_2 = 0 - x.iloc[0]
    y_shift_2 = 0 - y.iloc[-1]

    x += x_shift_2
    y += y_shift_2

    # stack for future steps
    sun_dirs = np.stack((x, y, z), axis=1)

    counter = 0
    unreachable_points = []

    # for loop through all the points
    for i in range(len(sun_dirs)):

        # try if they are reachable
        try: 
            inverse_kinematics(*sun_dirs[i], check_safety=False, verbal=False)

        # if not, bring point closer until it is reachable
        except ValueError:
            unreachable_points.append(sun_dirs[i].copy())
            counter += 1

            # loop at max 1000 times
            for j in range(max_iteration):
                
                # create new candidate with modified R
                new_x = (R-j) * np.cos(np.radians(alt.iloc[i])) * np.sin(np.radians(az.iloc[i] + 135)) + (1820/2)
                new_y = (R-j) * np.cos(np.radians(alt.iloc[i])) * np.cos(np.radians(az.iloc[i] + 135)) - (1680/2)
                new_z = (R-j) * np.sin(np.radians(alt.iloc[i]))

                # Shift it
                new_x += x_shift
                new_y += y_shift
                new_z += z_shift
                new_x += x_shift_2
                new_y += y_shift_2

                candidate = np.array([new_x, new_y, new_z])

                # Try if it is reachable
                try:
                    inverse_kinematics(*candidate, check_safety=False, verbal=False)
                    sun_dirs[i] = candidate
                    break
                
                # if not continue looping until it is
                except ValueError:
                    continue 
            
    # breakpoint()
    print(f"{counter} points are unreachable out of {len(sun_dirs)}")

    # making sure that none of the points are inside the safety boxes
    for i in range(len(sun_dirs)):
        if sun_dirs[i][0] < -1000:
            sun_dirs[i][0] = -999
        if sun_dirs[i][0] > 1820:
            sun_dirs[i][0] = 1819
        if sun_dirs[i][1] < -1680:
            sun_dirs[i][1] = -1679
        if sun_dirs[i][1] > 1000:
            sun_dirs[i][1] = 999

    return sun_dirs, np.array(unreachable_points)

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

    # suns_path, unreachable_points = get_sun_path(R=1700)
    # test_path = np.array([[-335.454, -195.317, 1000], [413.476, 497.258, 1000]])
    test_path = np.load('paths/test_path.npy')


    # sols = inverse_kinematics(*suns_path[int(len(suns_path)/2)], verbal=True)
    sols = inverse_kinematics(*test_path[1], verbal=True)

    # print("safe", check_solutions_safety(sols[0], all_boxes)

    # th1, th2, dr = [45, 90, 500]
    th1, th2, dr = sols[0]

    points = forward_kinematics(th1, th2, dr)

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
