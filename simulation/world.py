import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class BoxWithRailVisualizer:
    def __init__(self, width, height, depth, rail_width, figure=None):
        self.width = width
        self.height = height
        self.depth = depth
        self.rail_width = rail_width
        self.figure = figure if figure else plt.figure()  # Use provided figure or create a new one
        self.box_vertices = []
        self.rail_vertices = []
        self.box_faces = []
        self.rail_faces = []

        # Points for drawing arcs
        self.arc_points = {
            "A": np.array([0.8091, 0.775, 1.13]),
            "B": np.array([0.91, -0.7193, 2.13]),
            "C": np.array([0.7524, -0.8522, 2.13]),
            "D": np.array([-0.56, -1.3541, 1.13]),
            "E": np.array([0.5195, 0.0293, 1.63]),
            "F": np.array([-0.0405, -0.7457, 1.63])
        }

    def calculate_box_vertices(self):
        half_width = self.width / 2
        half_height = self.height
        half_depth = self.depth / 2

        # Define box vertices
        self.box_vertices = [
            [-half_width, -half_depth, 0], [half_width, -half_depth, 0],
            [half_width, half_depth, 0], [-half_width, half_depth, 0],
            [-half_width, -half_depth, half_height], [half_width, -half_depth, half_height],
            [half_width, half_depth, half_height], [-half_width, half_depth, half_height]
        ]

        # Define box faces
        self.box_faces = [
            [self.box_vertices[0], self.box_vertices[1], self.box_vertices[2], self.box_vertices[3]],  # Bottom face
            [self.box_vertices[4], self.box_vertices[5], self.box_vertices[6], self.box_vertices[7]],  # Top face
            [self.box_vertices[0], self.box_vertices[1], self.box_vertices[5], self.box_vertices[4]],  # Front face
            [self.box_vertices[2], self.box_vertices[3], self.box_vertices[7], self.box_vertices[6]],  # Back face
            [self.box_vertices[1], self.box_vertices[2], self.box_vertices[6], self.box_vertices[5]],  # Right face
            [self.box_vertices[0], self.box_vertices[3], self.box_vertices[7], self.box_vertices[4]],  # Left face
        ]

    def calculate_rail_vertices(self):
        rail_height = self.height / 2 + (self.height / 2 - 1.48)

        # Define rail vertices
        self.rail_vertices = [
            [0, -0.775, self.height], [-0.081, -0.7165, self.height],
            [0.56, 0, self.height], [0.479, 0.0585, self.height],  # Bottom
            [0, -0.775, self.height + self.rail_width], [-0.081, -0.7165, self.height + self.rail_width],
            [0.56, 0, self.height + self.rail_width], [0.479, 0.0585, self.height + self.rail_width]  # Top
        ]

        # Define rail faces
        self.rail_faces = [
            [self.rail_vertices[0], self.rail_vertices[1], self.rail_vertices[3], self.rail_vertices[2]],  # Bottom face
            [self.rail_vertices[4], self.rail_vertices[5], self.rail_vertices[7], self.rail_vertices[6]],  # Top face
            [self.rail_vertices[0], self.rail_vertices[1], self.rail_vertices[5], self.rail_vertices[4]],  # Front face
            [self.rail_vertices[2], self.rail_vertices[3], self.rail_vertices[7], self.rail_vertices[6]],  # Back face
            [self.rail_vertices[0], self.rail_vertices[2], self.rail_vertices[6], self.rail_vertices[4]],  # Left face
            [self.rail_vertices[1], self.rail_vertices[3], self.rail_vertices[7], self.rail_vertices[5]]   # Right face
        ]

    def draw_arc(self, ax, center, start, end, num_points=100):
        """Draw an arc between three points in 3D space."""
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

    def draw_cylinder(self, ax, radius, height, center=(0, 0), num_points=50):
        """Draw a cylinder in 3D space."""
        theta = np.linspace(0, 2 * np.pi, num_points)
        z = np.linspace(0, height, num_points)
        theta_grid, z_grid = np.meshgrid(theta, z)

        # Parametric equations for the cylinder surface
        x = center[0] + radius * np.cos(theta_grid)
        y = center[1] + radius * np.sin(theta_grid)
        z = self.height + 0.1 + z_grid  # Adjust the z-offset if needed

        # Plot the cylinder surface
        ax.plot_surface(x, y, z, color='cyan', alpha=0.6)

    def plot(self):
        # Calculate vertices and faces
        self.calculate_box_vertices()
        self.calculate_rail_vertices()

        # Create an axis in the provided figure
        ax = self.figure.add_subplot(111, projection='3d')

        # Add the box and rail to the plot
        ax.add_collection3d(Poly3DCollection(self.box_faces, facecolors='pink', linewidths=1, edgecolors='black', alpha=0.2))
        ax.add_collection3d(Poly3DCollection(self.rail_faces, facecolors='steelblue', linewidths=1, edgecolors='black', alpha=0.9))

        # Plot points and arcs
        for name, coord in self.arc_points.items():
            ax.scatter(*coord, label=name)
            ax.text(*coord, name, fontsize=10, color='red')

        # Draw arcs
        self.draw_arc(ax, self.arc_points["E"], self.arc_points["A"], self.arc_points["B"])
        self.draw_arc(ax, self.arc_points["F"], self.arc_points["C"], self.arc_points["D"])

        # Draw a sample cylinder
        self.draw_cylinder(ax, radius=0.1, height=0.1, center=(-0.045, -0.7457))

        # Set labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('3D Box with Rail, Arcs, and Cylinder')
        ax.legend()

# Example usage
figure = plt.figure(figsize=(10, 10))
visualizer = BoxWithRailVisualizer(width=1.12, height=1.38, depth=1.55, rail_width=0.10, figure=figure)
visualizer.plot()

plt.show()