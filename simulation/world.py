import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class BoxWithRailVisualizer:
    def __init__(self, width, height, depth, rail_width, num_of_point=50):
        """
        Initializes the simulation world with specified dimensions and parameters.

        Args:
            width (float): The width of the simulation world.
            height (float): The height of the simulation world.
            depth (float): The depth of the simulation world.
            rail_width (float): The width of the rail in the simulation world.
            num_of_point (int, optional): The number of points for drawing arcs and rectangles. Defaults to 50.

        Attributes:
            width (float): The width of the simulation world.
            height (float): The height of the simulation world.
            depth (float): The depth of the simulation world.
            rail_width (float): The width of the rail in the simulation world.
            box_vertices (list): List to store vertices of the box.
            rail_vertices (list): List to store vertices of the rail.
            box_faces (list): List to store faces of the box.
            rail_faces (list): List to store faces of the rail.
            arm1_length (float): Length of the first arm.
            amr2_length (float): Length of the second arm.
            num_of_points (int): Number of points for drawing arcs and rectangles.
            arc_points (dict): Dictionary containing points for drawing arcs and rectangles.
        """
        self.width = width
        self.height = height
        self.depth = depth
        self.rail_width = rail_width
        self.box_vertices = []
        self.rail_vertices = []
        self.box_faces = []
        self.rail_faces = []
        self.arm1_length = 0.8
        self.amr2_length = 0.5
        self.num_of_points = num_of_point

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
        """
        Calculates the vertices and faces of a box based on its width, height, and depth.

        This method computes the coordinates of the eight vertices of a box and stores them in the 
        `self.box_vertices` attribute. It also defines the six faces of the box using these vertices 
        and stores them in the `self.box_faces` attribute.

        The vertices are calculated assuming the box is centered at the origin (0, 0, 0) with the 
        bottom face lying on the XY plane.

        Attributes:
            self.box_vertices (list): A list of eight vertices, each represented as a list of three 
                                      coordinates [x, y, z].
            self.box_faces (list): A list of six faces, each represented as a list of four vertices.
        """
        """Calculates main box vertices"""
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
        """
        Calculate the vertices and faces of the rail.

        This method defines the vertices and faces of the rail based on the height
        and rail width attributes of the object. The vertices are defined in a 
        3D space, and the faces are defined using these vertices to form the 
        bottom, top, front, back, left, and right faces of the rail.

        Attributes:
            self.rail_vertices (list): A list of vertices defining the rail in 3D space.
            self.rail_faces (list): A list of faces defined by the vertices, forming the rail.
        """
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

    def draw_3d_box(self, ax, p1, p2, p1_new, p2_new):
        """Draw two lines in 3D space between specified points."""
        # First line: from p1 to p2
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='green', linewidth=2)

        # Second line: from p1_new to p2_new
        ax.plot([p1_new[0], p2_new[0]], [p1_new[1], p2_new[1]], [p1_new[2], p2_new[2]], color='green', linewidth=2)


    def draw_arc(self, ax, center, start, end, num_points=100):
        """
        Draw an arc between three points in 3D space.

        Parameters:
        ax (matplotlib.axes._subplots.Axes3DSubplot): The 3D axes to plot the arc on.
        center (array-like): The center point of the arc.
        start (array-like): The starting point of the arc.
        end (array-like): The ending point of the arc.
        num_points (int, optional): The number of points to generate along the arc. Default is 100.

        Returns:
        None
        """
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
        """
        Draw a cylinder in 3D space.

        Parameters:
        ax (matplotlib.axes._subplots.Axes3DSubplot): The 3D axis to plot the cylinder on.
        radius (float): The radius of the cylinder.
        height (float): The height of the cylinder.
        center (tuple, optional): The (x, y) coordinates of the center of the cylinder's base. Default is (0, 0).
        num_points (int, optional): The number of points to generate for the cylinder's surface. Default is 50.

        Returns:
        None
        """
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

    def calculate_arc_points(self, center, start, end):
        """
        Calculate points along an arc between three points in 3D space.

        Parameters:
        center (array): Center point of the arc.
        start (array): Starting point of the arc.
        end (array): Ending point of the arc.
        num_points (int): Number of points to calculate on the arc.

        Returns:
        np.array: Array of points on the arc.
        """
        center = np.array(center)
        start = np.array(start) - center
        end = np.array(end) - center

        # Calculate normal vector to the plane of the arc
        normal = np.cross(start, end)
        normal /= np.linalg.norm(normal)

        # Calculate angle between start and end vectors
        angle = np.arccos(np.dot(start, end) / (np.linalg.norm(start) * np.linalg.norm(end)))

        # Generate points on the arc
        t = np.linspace(0, angle, self.num_of_points)
        arc_points = np.array([
            center + np.cos(theta) * start + np.sin(theta) * np.cross(normal, start)
            for theta in t
        ])

        return arc_points

    def calculate_dynamic_lines(self, start_point, end_point):
        """
        Calculate the dynamic lines for the robot arm based on the start and end points.

        This function computes the intermediate and final positions of a robot arm's segments
        given the start and end points in 3D space. It ensures that the lengths of the arm
        segments are correct.

        Parameters:
        start_point (list or tuple): The starting coordinates [x, y, z] of the robot arm.
        end_point (list or tuple): The ending coordinates [x, y, z] of the robot arm.

        Returns:
        tuple: A tuple containing the start point, middle point, and real end point as lists.

        Raises:
        AssertionError: If the calculated lengths of the arm segments do not match the expected lengths.
        """
        try:
            sx, sy, sz = start_point[0], start_point[1], start_point[2]
            ex, ey, ez = end_point[0], end_point[1], end_point[2]

            delta_y = ey - sy
            delta_x = ex - sx

            theta = np.atan(delta_y / delta_x)

            delta_z = ez - sz

            delta_xy = np.sqrt(self.amr2_length**2 - delta_z**2)

            x1 = sx - (np.cos(theta) * self.arm1_length)
            y1 = sy - (np.sin(theta) * self.arm1_length)

            x2 = x1 - (np.cos(theta) * delta_xy)
            y2 = y1 - (np.sin(theta) * delta_xy)
            if delta_x > 0:
                x1 = sx + (np.cos(theta) * self.arm1_length)
                y1 = sy + (np.sin(theta) * self.arm1_length)
                x2 = x1 + (np.cos(theta) * delta_xy)
                y2 = y1 + (np.sin(theta) * delta_xy)

            middle_point = [x1, y1, sz]
            real_end = np.array([x2, y2, ez])

            # Checks that arms are correct leght
            assert round(np.linalg.norm(middle_point - real_end), 2) == self.amr2_length
            assert round(np.linalg.norm(start_point - middle_point), 2) == self.arm1_length

            return start_point, middle_point, real_end
        except AssertionError:
            print(f"One of the arms was wrong size with start point: {start_point} and end point: {end_point}")

    def plot(self, index):
        """
        Plots a 3D visualization of the box, rail, arcs, and dynamic lines.

        Args:
            index (int): The index of the point to plot.

        The function performs the following steps:
        1. Calculates the vertices and faces of the box and rail.
        2. Creates a 3D axis in the provided figure.
        3. Calculates and plots arc points.
        4. Determines the point and circle center based on the index.
        5. Adds the box and rail to the plot.
        6. Plots the points and arcs.
        7. Draws the arcs.
        8. Calculates dynamic points for the lines.
        9. Plots the dynamic lines.
        10. Sets labels and title for the plot.
        """
        # Calculate vertices and faces
        self.calculate_box_vertices()
        self.calculate_rail_vertices()

        # Create an axis in the provided figure
        plt.figure(index, figsize=(18, 9))
        ax = plt.axes(projection='3d')

        # Calculate and plot arc points
        arc1_points = self.calculate_arc_points(self.arc_points["E"], self.arc_points["A"], self.arc_points["B"])
        arc2_points = self.calculate_arc_points(self.arc_points["F"], self.arc_points["C"], self.arc_points["D"])

        if index < self.num_of_points:
            point = arc1_points[index]
            circle_center = (0.5195, 0.0293)
            start_point = self.arc_points["E"]
        elif (index - self.num_of_points) < self.num_of_points:
            point = arc2_points[index - self.num_of_points]
            circle_center = (-0.045, -0.7457)
            start_point = self.arc_points["F"]
        else:
            return

        # Add the box and rail to the plot
        ax.add_collection3d(Poly3DCollection(self.box_faces, facecolors='pink', linewidths=1, edgecolors='black', alpha=0.2))
        ax.add_collection3d(Poly3DCollection(self.rail_faces, facecolors='steelblue', linewidths=1, edgecolors='black', alpha=0.9))
        self.draw_cylinder(ax, radius=0.1, height=0.1, center=circle_center)

        # Plot points and arcs
        for name, coord in self.arc_points.items():
            ax.scatter(*coord, label=name)
            ax.text(*coord, name, fontsize=10, color='red')

        # Draw arcs
        self.draw_arc(ax, self.arc_points["E"], self.arc_points["A"], self.arc_points["B"])
        self.draw_arc(ax, self.arc_points["F"], self.arc_points["C"], self.arc_points["D"])

        # Plot the arcs
        ax.scatter(arc1_points[:, 0], arc1_points[:, 1], arc1_points[:, 2], color='black', s=10, label='Arc 1 Points')
        ax.scatter(arc2_points[:, 0], arc2_points[:, 1], arc2_points[:, 2], color='black', s=10, label='Arc 2 Points')

        # Define the starting point dynamically
        # start_point = self.arc_points["F"]

        # Calculate dynamic points for the lines
        p1, p2, final = self.calculate_dynamic_lines(start_point, point)

        # Plot the dynamic lines
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='green', linewidth=2, label='Arm 1')
        ax.plot([p2[0], final[0]], [p2[1], final[1]], [p2[2], final[2]], color='blue', linewidth=2, label='Arm 2')

        # Set labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('3D Box with Rail, Arcs, and Rectangle')
        ax.legend()

# Example usage
visualizer = BoxWithRailVisualizer(width=1.12, height=1.38, depth=1.55, rail_width=0.10)

for i in range(visualizer.num_of_points*2):
    visualizer.plot(i)
    plt.show()
    plt.close()