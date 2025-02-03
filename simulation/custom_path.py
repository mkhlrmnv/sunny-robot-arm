import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define points from your image (converted to meters if needed)
points = {
    "A": np.array([0.8091, 0.775, 1.13]),
    "B": np.array([0.91, -0.7193, 2.13]),
    "C": np.array([0.7524, -0.8522, 2.13]),
    "D": np.array([-0.56, -1.3541, 1.13]),
    "E": np.array([0.5195, 0.0293, 1.63]),
    "F": np.array([-0.0405, -0.7457, 1.63])
}

# Function to draw an arc between three points (center, start, end)
def draw_arc(ax, center, start, end, num_points=100):
    center = np.array(center)
    start = np.array(start) - center
    end = np.array(end) - center

    # Calculate normal vector to the plane of the arc
    normal = np.cross(start, end)
    normal /= np.linalg.norm(normal)

    # Calculate angle between start and end vectors
    angle = np.arccos(np.dot(start, end) / (np.linalg.norm(start) * np.linalg.norm(end)))

    # Generate arc points
    t = np.linspace(0, angle, num_points)
    arc_points = np.array([
        center + np.cos(theta) * start + np.sin(theta) * np.cross(normal, start)
        for theta in t
    ])

    # Plot the arc
    ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2], color='black', linewidth=2)

# Plotting the 3D visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot points
for name, coord in points.items():
    ax.scatter(*coord, label=name)
    ax.text(*coord, name, fontsize=10, color='red')

# Draw arcs
draw_arc(ax, points["E"], points["A"], points["B"])
draw_arc(ax, points["F"], points["C"], points["D"])

# Connect other points with straight lines (if needed)
ax.plot([points["A"][0], points["B"][0]], [points["A"][1], points["B"][1]], [points["A"][2], points["B"][2]], color='blue', linestyle='dashed')
ax.plot([points["C"][0], points["D"][0]], [points["C"][1], points["D"][1]], [points["C"][2], points["D"][2]], color='blue', linestyle='dashed')

# Set labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title("3D Paths Visualization")
ax.legend()

# Display the plot
plt.show()