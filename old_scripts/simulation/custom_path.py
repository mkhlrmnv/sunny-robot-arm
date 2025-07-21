import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_rectangle_3d(p1, p2, width):
    """
    Calculate the coordinates of a rectangle in 3D based on two center points.
    
    Parameters:
    p1 (tuple): First center point (x, y, z).
    p2 (tuple): Second center point (x, y, z).
    width (float): Width of the rectangle (perpendicular to the line between p1 and p2).

    Returns:
    list: List of four corner points of the rectangle [(x1, y1, z1), ..., (x4, y4, z4)].
    """
    p1 = np.array(p1)
    p2 = np.array(p2)

    # Calculate the vector between p1 and p2
    direction = p2 - p1
    direction_length = np.linalg.norm(direction)

    if direction_length == 0:
        raise ValueError("The two points must be different to define a rectangle.")
    
    # Normalize the direction vector
    direction = direction / direction_length

    # Find a perpendicular vector in 3D
    if direction[0] == 0 and direction[1] == 0:  # If the direction is along the Z-axis
        perpendicular = np.array([1, 0, 0])
    else:
        perpendicular = np.cross(direction, [0, 0, 1])
        perpendicular /= np.linalg.norm(perpendicular)

    # Half-width offset for both sides
    half_width = width / 2

    # Calculate the four corners of the rectangle
    corner1 = p1 + half_width * perpendicular
    corner2 = p1 - half_width * perpendicular
    corner3 = p2 - half_width * perpendicular
    corner4 = p2 + half_width * perpendicular

    return [corner1, corner2, corner3, corner4]

def draw_rectangle_3d(ax, corners):
    """
    Draw the rectangle in a 3D plot.
    
    Parameters:
    ax: The 3D axis to draw on.
    corners: List of four corner points of the rectangle.
    """
    corners = np.array(corners)

    # Close the rectangle by appending the first corner at the end
    corners = np.vstack([corners, corners[0]])

    # Plot the rectangle edges
    ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], color='red', linewidth=2)

    # Scatter the corner points
    ax.scatter(corners[:, 0], corners[:, 1], corners[:, 2], color='blue', s=50)

# Example usage
p1 = (-0.045, -0.7457, 1.63)
p2 = (-0.56, -1.3541, 1.63)
width = 0.10

# Calculate rectangle corners
rectangle_corners = calculate_rectangle_3d(p1, p2, width)

# Plot the 3D rectangle
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw the rectangle in 3D
draw_rectangle_3d(ax, rectangle_corners)

# Set axis labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('3D Rectangle Visualization')

# Set axis limits for better visualization

plt.show()