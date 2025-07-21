"""
sun_helper.py
~~~~~~~~~~~~~

Utilities for computing the sun's path and coloring for a robotic-arm simulation.

Functions
---------
alt_to_color
    Map solar altitude (°) to approximate RGB colors via color-temperature conversion.
get_sun_path
    Generate and adjust a series of 3D sun-direction points for one day,
    clamp them to the robot's workspace, and record unreachable positions.
jsonify_path
    Convert an (N×3) path array (and optional colors) into JSON-friendly dict.
un_jsonify_path
    Load a JSON file produced by jsonify_path back into numpy arrays.

Dependencies
------------
  - numpy, pandas, pvlib for solar geometry
  - matplotlib for color normalization
  - kinematics_and_safety.inverse_kinematics, forward_kinematics for reach checks
  - config constants defining workspace limits and transforms
"""

import numpy as np
import pvlib
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import json
import os

from config import (
    BASE_TRANSFORM_MATRIX, RAIL_ANGLE,
    SAFE_ZONE_X_CORD, SAFE_ZONE_Y_CORD,
    PONTTO_Z_OFFSET, PAATY_ARM_LENGTH,
    PONTTO_X_OFFSET, PAATY_X_OFFSET,
    RAIL_MIN_LIMIT, RAIL_MAX_LIMIT,
    PONTTO_MOTOR_MIN_ANGLE,
    PONTTO_ARM_LENGTH
)

def alt_to_color(alt: pd.Series, 
                 min_temp: int = 2000, 
                 max_temp: int = 6500
                 ) -> np.array:
    """
    Convert solar altitude angles to RGB colors by mapping to correlated color temperature.

    Parameters
    ----------
    alt : pd.Series
        Solar elevation angles (degrees) above horizon for each time point.
    min_temp, max_temp : int
        Bounds for color temperature mapping (Kelvin).

    Returns
    -------
    colors : (N,3) array
        RGB tuples normalized 0–1 for each altitude.
    """
    
    # Scale altitudes to color temperatures between min_temp and max_temp
    max_alt = alt.max()
    temps = min_temp + (alt / max_alt) * (max_temp - min_temp)

    def cct_to_rgb(cct_kelvin: float) -> tuple[float,float,float]:
        """
        Convert a color temperature (Kelvin) to an RGB triple (0–1).
        Uses empirical formulas from Tanner Helland.
        """
        temp = cct_kelvin / 100.0

        # Red channel
        if temp <= 66:
            red = 255.0
        else:
            red = 329.698727446 * ((temp - 60) ** -0.1332047592)

        # Green channel
        if temp <= 66:
            green = 99.4708025861 * np.log(temp) - 161.1195681661
        else:
            green = 288.1221695283 * ((temp - 60) ** -0.0755148492)

        # Blue channel
        if temp >= 66:
            blue = 255.0
        elif temp <= 19:
            blue = 0.0
        else:
            blue = 138.5177312231 * np.log(temp - 10) - 305.0447927307

        # Clip
        red   = np.clip(red,   0, 255)
        green = np.clip(green, 0, 255)
        blue  = np.clip(blue,  0, 255)

        # Normalize to 0–1 for matplotlib
        return red/255.0, green/255.0, blue/255.0

     # Vectorized conversion for all temperatures
    colors = np.array([cct_to_rgb(t) for t in temps])
    return colors


def get_sun_path(R: int = 1700,
             latitude: float = 60.1699, 
             longitude: float = 24.9384,
             timezone: str = 'Europe/Helsinki'
             ) -> tuple[np.array, np.array, np.array]:
    """
    Compute the sun's 3D positions over a single day, transform them into the
    robot's coordinate frame, clamp to workspace limits, and collect unreachable
    points.

    Parameters
    ----------
    R : float
        Radius of the virtual celestial sphere (for plotting scale).
    latitude, longitude : float
        Observer's geographic coordinates.
    timezone : str
        IANA timezone for date localization.

    Returns
    -------
    sun_dirs : np.array
        N×3 array of robot-frame [x, y, z] sun directions (after workspace shift).
    unreachable_pts : np.array
        M×3 array of original sun_dirs that IK reported as unreachable.
    colors : np.array
        Nx3 array [r, g, b] of the estimated suns color for each points .
    """
    
    # Necessary imports for inverse kinematics and forward kinematics
    # made here to avoid circular imports
    from kinematics_and_safety import inverse_kinematics, forward_kinematics

    # 1) Build datetime index for the day at 10-min intervals
    times = pd.date_range(
        end  ='2025-06-21 23:59',   
        start='2025-06-21 00:00',   
        freq='10min',
        tz=timezone
    )

    # 2) Compute solar positions and drop nighttime (elevation ≤ 0°)
    location = pvlib.location.Location(latitude, longitude, timezone)
    solpos = location.get_solarposition(times)
    # keep only daytime
    solpos = solpos.loc[solpos['apparent_elevation'] > 0, :]
    alt = solpos['apparent_elevation']  
    az  = solpos['azimuth']

    print("alt type", type(alt))

    # map altitude → RGB color
    colors = alt_to_color(alt)

    # 3) Convert to unit‐sphere Cartesian for 3D plotting
    #    X axis → East, Y → North, Z → Up
    x = R * np.cos(np.radians(alt)) * np.sin(np.radians(az + 135)) # + (1820/2)
    y = R * np.cos(np.radians(alt)) * np.cos(np.radians(az + 135)) # - (1680/2)
    z = R * np.sin(np.radians(alt))

    # 4) Shift path to align its midpoint with the overcrossing point 
    # of the safety boxes + offset of 300mm

    center_point = [-SAFE_ZONE_X_CORD + 300, 
                    SAFE_ZONE_Y_CORD - 300, 
                    -PAATY_ARM_LENGTH + PONTTO_Z_OFFSET]
    
    mid_idx = len(x)//2
    # compute translation offsets so midpoint lands at center_pt
    x_shift = center_point[0] - x.iloc[mid_idx]
    y_shift = center_point[1] - y.iloc[mid_idx] 
    z_shift = center_point[2] - np.min([z.iloc[0], z.iloc[len(z)-1]])
    x += x_shift; y += y_shift; z += z_shift

    # 5) Shift path so the all points are outside the "kontti" box
    x_shift_2 = 0 - x.iloc[0]
    y_shift_2 = 0 - y.iloc[-1]
    x += x_shift_2; y += y_shift_2

    # Assemble N×3 array of 3D points
    sun_dirs = np.stack((x, y, z), axis=1)

    # trim first/last few as they are usually to close to the 
    # "kontti" and cannot be reached safely
    sun_dirs =  sun_dirs[8:-6]
    colors = colors[8:-6]


    # if point is in of the safety box, then set it to the edge of the box

    # 6) If the points is inside safety boxes -> outside of the working area
    # shift it into the limits
    for i, (xi, yi, zi) in enumerate(sun_dirs):
        xi = np.clip(xi, -1000+1, 1820-1)
        yi = np.clip(yi, -1680+1, 1100-1)
        sun_dirs[i,0], sun_dirs[i,1] = xi, yi

    counter = 0
    unreachable_points = []

    # 7) Test each point with IK; collect unreachable, and adjust the to be reachable
    for i in range(len(sun_dirs)):
        try: 
            # only checking, we ignore returned solutions here
            inverse_kinematics(*sun_dirs[i], check_safety=True, verbal=False)

        # if not, then ...
        except ValueError:
            unreachable_points.append(sun_dirs[i].copy())
            counter += 1
            # print("counter ", counter)
            # print(f"Point {i} is unreachable: {sun_dirs[i]}")

            # collect original coordinates
            x, y, z = sun_dirs[i]

            # translate them into a base + rail coordinates

            # base
            T_base=BASE_TRANSFORM_MATRIX
            T_inv = np.linalg.inv(T_base)
            p_world = np.array([x, y, z, 1])
            p_local = T_inv @ p_world
            x_l, y_l, z_l = p_local[:3]

            # rail
            from scipy.spatial.transform import Rotation as R
            Rz = R.from_euler('z', RAIL_ANGLE, degrees=True).as_matrix()
            p_rail = Rz.T @ np.array([x_l, y_l, z_l])
            x_r, y_r, z_r = p_rail

            # solve for how much high is the point above the first joint
            z_eff = z_r - PONTTO_Z_OFFSET

            # assume that arm is too long / short and use theta_2 from previous point
            theta_2_deg = inverse_kinematics(*sun_dirs[i-1])[0][1]
            theta_2 = np.radians(theta_2_deg)

            # Two constants, that indicate the x and y offset of the arm, if arm is alligned 
            # with y-axis 
            arm_reach_x = PONTTO_X_OFFSET + PAATY_X_OFFSET
            arm_reach_y = z_eff / np.tan(theta_2) + PONTTO_ARM_LENGTH

            # total reach of the arm in 2d world (x, y)
            r = np.hypot(arm_reach_x, arm_reach_y)
            
            # rename the points 
            # (cx, cy) is the 2D coordinate of required end point
            cx, cy = x_r, y_r

            # calculate the d = required arm reach in 2d plane
            if cy > RAIL_MIN_LIMIT and cy < RAIL_MAX_LIMIT:
                # Case 1: End points y cord is in the rail limits, so we will assume that arm will try
                # to reach the point from closes point that is on the y-axis, resulting in d being
                # equal to end points x cord
                d = cx
            elif cy <= RAIL_MIN_LIMIT:
                # Case 2: End points y cord is smaller then rail min limit, meaning that robot 
                # will have to reach point from rails min position (0, 0), to the d is hypot between
                # origin and the end point
                d = np.hypot(cx, cy)
            elif cy >= RAIL_MAX_LIMIT:
                # Case 3: End points y cord is bigger then rail max limit, meaning that robot 
                # will have to reach point from rails maximum position (0, MAX_LIM), to the d is hypot between
                # max pos and the end point
                d = np.sqrt(cx**2 + (RAIL_MAX_LIMIT-cy)**2)

            # If required reach is larger then actual reach of the robot, we will shift the
            # end point closer to be reachable
            if d > r:
                s  = r / d                     # 0 < s < 1
                cx = s * cx
                cy = s * cy
                
                # (optional) log or store how much we shortened:
                shrink_mm = d - r
                print(f"   → shortened ray by {shrink_mm:.1f} mm to stay reachable")

            # everything under is copied from IK function is kinematics_and_safety.py
            # so for more explanation look there
            rhs = r**2 - cx**2

            if rhs < 0: 
                print("ERRPR: rhs < 0")
                continue
                
            for sign in (+1, -1):
                y_wrist = cy + sign * np.sqrt(rhs)

                print(f"\t  y_wrist[{sign:+}] = {y_wrist:.2f}", end="")

                if not (RAIL_MIN_LIMIT <= y_wrist <= RAIL_MAX_LIMIT):
                    print("\n")
                    continue

                print(" ✓ within limits")

                dx = x_r - 0
                dy = y_r - y_wrist
                alpha = np.arctan2(dy, dx)
                gamma = np.arctan2(arm_reach_y, arm_reach_x)
                theta_1 = alpha - gamma

                # wrap into [-180,180]
                theta_1_deg = (((np.degrees(theta_1) + RAIL_ANGLE) - (PONTTO_MOTOR_MIN_ANGLE)) % 360) + (PONTTO_MOTOR_MIN_ANGLE)


                print(f"\t    α={np.degrees(alpha):.2f}°, γ={np.degrees(gamma):.2f}°")
                print(f"\t    raw θ1={np.degrees(theta_1):.2f}°, wrapped θ1={theta_1_deg:.2f}°")
                print(f"\t    wrapped θ2={theta_2_deg:.2f}°, delta_r={y_wrist:.2f}")

                sol = (theta_1_deg, theta_2_deg, y_wrist)

                # calculate the new adjusted end point and replace the unreachable with it
                end_point = forward_kinematics(*sol)
                sun_dirs[i] = end_point[-1]
            
    # report count of unreachable
    print(f"{counter} points are unreachable out of {len(sun_dirs)}")

    return sun_dirs, np.array(unreachable_points), colors


def jsonify_path(path: np.array, 
                 colours: np.ndarray | None = None
                 ) -> dict:
    """
    Convert path and optional RGB colors into JSON-serializable dict.

    Returns {'path':[{'x':..,'y':..,'z':..},...], 'colors':[{'r':..,'g':..,'b':..},...]}
    """

    result = {'path': [{ 'x':float(x),'y':float(y),'z':float(z) } for x,y,z in path]}

    if colours is not None:
        assert len(path) == len(colours), "Path and colours must have the same length"
        result['colors'] = [
            {'r':int(r*255),'g':int(g*255),'b':int(b*255)}
            for r,g,b in colours
        ]

    return result

def un_jsonify_path(json_file: str) -> tuple[np.ndarray, np.ndarray | None]:
    """
    Load JSON file from jsonify_path into numpy arrays.

    Returns (path_array, colors_array or None).
    """
    with open(json_file) as f:
        data = json.load(f)
    path = np.array([[p['x'],p['y'],p['z']] for p in data['path']])
    colors = None
    if 'colors' in data:
        colors = np.array([[c['r'],c['g'],c['b']] for c in data['colors']])
    return path, colors
    

if __name__ == "__main__":

    sun_dirs, unreachable_points, colors = get_sun_path()
    
    res = jsonify_path(sun_dirs, colors)

    print(res)

    file_path = os.path.join(os.path.dirname(__file__), "..", "paths", "sun_path.json")

    with open(file_path, "w") as f:
        json.dump(res, f, indent=4)

    sun_dirs2, colors2 = un_jsonify_path(file_path)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sun_dirs2[:, 0], sun_dirs2[:, 1], sun_dirs2[:, 2], c=colors2, s=10)
    plt.show()
    '''

    orig_file_path = os.path.join(os.path.dirname(__file__), '..', 'paths', 'test2_path.npy')

    path = np.load(orig_file_path, allow_pickle=True)

    res = jsonify_path(path)

    file_path = os.path.join(os.path.dirname(__file__), "..", "paths", "test2_path.json")

    with open(file_path, "w") as f:
        json.dump(res, f, indent=4)

    sun_dirs2, colors2 = un_jsonify_path(file_path)

    if colors2 is None:
        colors2 = 'b'
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sun_dirs2[:, 0], sun_dirs2[:, 1], sun_dirs2[:, 2], c=colors2, s=10)
    plt.show()
    '''

