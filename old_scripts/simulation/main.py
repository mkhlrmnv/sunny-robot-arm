import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotation_matrix_z(theta):
    """Rotation matrix around the Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotation_matrix_y(theta):
    """Rotation matrix around the Y-axis."""
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def translation_matrix(dx, dy, dz):
    """Translation matrix."""
    return np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2, L1, L2):
    """
    Computes the position of the robot arm's segments and end effector.
    theta1: Rotation of first joint (around Z-axis).
    theta2: Rotation of second joint (around Y-axis).
    L1: Length of first arm segment.
    L2: Length of second arm segment.
    """
    # Base to first joint (rotate around Z-axis)
    T1 = rotation_matrix_z(theta1)
    # First joint to second joint (translate along X and rotate around Y)
    T2 = T1 @ translation_matrix(L1, 0, 0) @ rotation_matrix_y(theta2)
    # Second joint to end effector (translate along X)
    T3 = T2 @ translation_matrix(L2, 0, 0)

    # Extract positions
    origin = np.array([0, 0, 0])
    joint1 = T1[:3, 3]
    joint2 = T2[:3, 3]
    end_effector = T3[:3, 3]

    return origin, joint1, joint2, end_effector

def plot_robot_arm(positions):
    """Plots the robot arm in 3D space."""
    origin, joint1, joint2, end_effector = positions

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the segments
    ax.plot([origin[0], joint1[0]], [origin[1], joint1[1]], [origin[2], joint1[2]], 'bo-', label="Segment 1")
    ax.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], [joint1[2], joint2[2]], 'ro-', label="Segment 2")
    ax.plot([joint2[0], end_effector[0]], [joint2[1], end_effector[1]], [joint2[2], end_effector[2]], 'go-', label="End Effector")

    # Set labels and limits
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_xlim(-L1-L2, L1+L2)
    ax.set_ylim(-L1-L2, L1+L2)
    ax.set_zlim(-L1-L2, L1+L2)
    ax.legend()
    plt.show()

# Arm parameters
L1 = 3  # Length of first segment
L2 = 2  # Length of second segment

# Joint angles
theta1 = np.radians(45)  # Rotation around Z-axis
theta2 = np.radians(30)  # Rotation around Y-axis

# Compute positions
positions = forward_kinematics(theta1, theta2, L1, L2)

# Plot the robot arm
plot_robot_arm(positions)