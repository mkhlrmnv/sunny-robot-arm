import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

def calculate_declination(day_of_year):
    """Calculate the solar declination angle (δ) in radians."""
    return np.radians(23.44) * np.sin(2 * np.pi * (day_of_year - 81) / 365)

def calculate_solar_position(latitude, declination, hour_angle):
    """
    Calculate the altitude (h) and azimuth (A) of the sun.
    latitude: Observer's latitude in radians.
    declination: Solar declination in radians.
    hour_angle: Hour angle in radians.
    Returns altitude (h) and azimuth (A) in radians.
    """
    altitude = np.arcsin(
        np.sin(latitude) * np.sin(declination) + np.cos(latitude) * np.cos(declination) * np.cos(hour_angle)
    )
    azimuth = np.arctan2(
        -np.cos(declination) * np.sin(hour_angle),
        np.cos(latitude) * np.sin(declination) - np.sin(latitude) * np.cos(declination) * np.cos(hour_angle)
    )
    return altitude, azimuth

def spherical_to_cartesian(altitude, azimuth):
    """Convert spherical coordinates to Cartesian (x, y, z)."""
    x = np.cos(altitude) * np.cos(azimuth)
    y = np.cos(altitude) * np.sin(azimuth)
    z = np.sin(altitude)
    return x, y, z

def sun_path(latitude, day_of_year):
    """
    Calculate the 3D path of the sun for a given day of the year at a specific latitude.
    latitude: Observer's latitude in degrees.
    day_of_year: Day of the year (1 to 365).
    """
    latitude_rad = np.radians(latitude)
    declination = calculate_declination(day_of_year)
    
    hours = np.linspace(0, 24, 100)  # Simulate over 24 hours
    hour_angles = np.radians((hours - 12) * 15)  # Convert time to hour angle (in radians)

    # Compute sun positions
    positions = []
    for hour_angle in hour_angles:
        altitude, azimuth = calculate_solar_position(latitude_rad, declination, hour_angle)
        x, y, z = spherical_to_cartesian(altitude, azimuth)
        if z >= 0:
            x += 0.6
            positions.append((x, y, z))
    
    return np.array(positions)

# Helsinki's parameters
latitude = 60.1699
day_of_year = datetime.now().timetuple().tm_yday  # Current day of the year

# Get the sun's path
positions = sun_path(latitude, day_of_year)

# Plotting the path in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, z
x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

# Plot the path
ax.plot(x, y, z, label="Sun's Path", color='orange')
ax.scatter([0], [0], [0], color='blue', label="Observer (Helsinki)")  # Observer's location

# Set labels
ax.set_xlabel("X (East-West)")
ax.set_ylabel("Y (North-South)")
ax.set_zlabel("Z (Altitude)")
ax.set_title("3D Sun Path Over Helsinki")
ax.legend()
plt.show()