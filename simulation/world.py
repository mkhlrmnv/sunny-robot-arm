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

        # Points for drawing arcs and rectangle
        self.arc_points = {
            "A": np.array([0.8091, 0.775, 1.13]),
            "B": np.array([0.91, -0.7193, 2.13]),
            "C": np.array([0.7524, -0.8522, 2.13]),
            "D": np.array([-0.56, -1.3541, 1.13]),  # Base point for rectangle
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

    def calculate_rectangle_3d(self, p1, p2, width):
        """
        Calculate the coordinates of a rectangle in 3D based on two points.
        
        Parameters:
        p1 (tuple): First center point (x, y, z).
        p2 (tuple): Second point (adjusted to have the same height as p1).
        width (float): Width of the rectangle.

        Returns:
        list: List of four corner points of the rectangle.
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

    def draw_3d_box(self, ax, rectangle1, rectangle2):
        """
        Draw a 3D box given two rectangles in 3D space.
        
        Parameters:
        ax: The 3D axis to draw on.
        rectangle1: List of four corner points for the first rectangle.
        rectangle2: List of four corner points for the second rectangle.
        """
        # Define the box faces by connecting corresponding points from both rectangles
        faces = [
            [rectangle1[0], rectangle1[1], rectangle2[1], rectangle2[0]],  # Front face
            [rectangle1[1], rectangle1[2], rectangle2[2], rectangle2[1]],  # Side face
            [rectangle1[2], rectangle1[3], rectangle2[3], rectangle2[2]],  # Back face
            [rectangle1[3], rectangle1[0], rectangle2[0], rectangle2[3]],  # Side face
            [rectangle1[0], rectangle1[1], rectangle1[2], rectangle1[3]],  # Top face
            [rectangle2[0], rectangle2[1], rectangle2[2], rectangle2[3]]   # Bottom face
        ]

        # Draw each face
        box_faces = Poly3DCollection(faces, color='green', alpha=0.5, edgecolor='black')
        ax.add_collection3d(box_faces)

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
        self.draw_cylinder(ax, radius=0.1, height=0.1, center=(-0.045, -0.7457))

        # Plot points and arcs
        for name, coord in self.arc_points.items():
            ax.scatter(*coord, label=name)
            ax.text(*coord, name, fontsize=10, color='red')

        # Draw arcs
        self.draw_arc(ax, self.arc_points["E"], self.arc_points["A"], self.arc_points["B"])
        self.draw_arc(ax, self.arc_points["F"], self.arc_points["C"], self.arc_points["D"])

        # Calculate and draw a rectangle
        p1 = np.array([self.arc_points["F"][0], self.arc_points["F"][1], 1.38 + 0.2])  # Center of the circle
        p2 = np.array([self.arc_points["D"][0], self.arc_points["D"][1], p1[2]])  # Adjusted point with same height
        rectangle1 = self.calculate_rectangle_3d(p1, p2, width=0.1)

        # Calculate and draw the second rectangle
        p1_new = np.array([-0.0405, -0.7457, p1[2] + 0.1])
        p2_new = np.array([self.arc_points["D"][0], self.arc_points["D"][1], p1_new[2]])
        rectangle2 = self.calculate_rectangle_3d(p1_new, p2_new, width=0.1)

        # Draw the 3D box between the two rectangles
        self.draw_3d_box(ax, rectangle1, rectangle2)

        # Set labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('3D Box with Rail, Arcs, and Rectangle')
        ax.legend()

# Example usage
figure = plt.figure(figsize=(8, 8))
visualizer = BoxWithRailVisualizer(width=1.12, height=1.38, depth=1.55, rail_width=0.10, figure=figure)
visualizer.plot()

plt.show()