import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def draw_cylinder(ax, radius, height, center=(0, 0), num_points=50):
    """
    Draws a 3D cylinder.
    
    Parameters:
    ax: The 3D axis to draw on.
    radius: Radius of the cylinder.
    height: Height of the cylinder.
    center: (x, y) coordinates of the cylinder's center.
    num_points: Number of points used to generate the surface.
    """
    # Generate cylinder coordinates
    theta = np.linspace(0, 2 * np.pi, num_points)
    z = np.linspace(0, height, num_points)
    theta_grid, z_grid = np.meshgrid(theta, z)

    # Parametric equations for the cylinder surface
    x = center[0] + radius * np.cos(theta_grid)
    y = center[1] + radius * np.sin(theta_grid)
    z = 163 + z_grid

    # Plot the surface of the cylinder
    ax.plot_surface(x, y, z, color='cyan', alpha=0.6)

    # Optional: Draw top and bottom faces
    ax.plot(x[0], y[0], z[0], color='blue')  # Bottom edge
    ax.plot(x[-1], y[-1], z[-1], color='blue')  # Top edge

# Plotting the cylinder
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw a cylinder with radius 1, height 2, and centered at (0, 0)
draw_cylinder(ax, radius=0.1, height=0.1, center=(-0.045, -74.57))

# Set labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('3D Cylinder (Lieriö)')

# Set the axis limits

# Show the plot
plt.show()