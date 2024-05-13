import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt


def rotational_affine(axis: str, angle: float, degrees=True) -> np.ndarray:
    if axis not in 'xyz':
        raise ValueError(f"Invalid axis: {axis}")
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = R.from_euler(axis, angle, degrees=degrees).as_matrix()
    return rotation_matrix


def translational_affine(axis: str, displacement: float) -> np.ndarray:
    """Calculates a translational affine matrix for a specific displacement."""
    if axis not in ["tx", "ty", "tz"]:
        raise ValueError(f"Invalid axis: {axis}")

    transform = np.eye(4)
    if axis == 'tx':
        transform[0, 3] = displacement
    elif axis == 'ty':
        transform[1, 3] = displacement
    elif axis == 'tz':
        transform[2, 3] = displacement
    return transform


def chained_rotations(angles: list | tuple, degrees=True) -> np.ndarray:
    """
        Calculates the resulting affine matrix from a sequence of rotations.

        Args:
            angles (list): List of rotation specifications.
                              - Each rotation can be a list [x, y, z] (Euler angles in degrees).
                              - Or a tuple ('axis', angle) (axis and angle in degrees).
                              - Or a string 'axis' and a float angle.
                              - Or a float angle for a rotation around x, y, z.

        Returns:
            np.ndarray: The combined rotation matrix.
            :param degrees:
            :param angles:
        """
    axis_rotation = np.eye(4)
    if isinstance(angles[0], list):
        rotations = R.from_euler('xyz', angles, degrees=degrees).as_matrix()
        for rotation in rotations:
            axis_rotation[:3, :3] = axis_rotation[:3, :3] @ rotation
    elif isinstance(angles[0], tuple):
        for axis, angle in angles:
            axis_rotation = axis_rotation @ rotational_affine(axis, angle, degrees=degrees)
    elif isinstance(angles[0], str) and len(angles) == 2:
        axis_rotation = axis_rotation @ rotational_affine(angles[0], angles[1], degrees=degrees)
    elif isinstance(angles[0], float | int) and len(angles) == 3:
        axis_rotation[:3, :3] = R.from_euler('xyz', angles, degrees=degrees).as_matrix()
    else:
        raise ValueError("Invalid rotation specification.")
    return axis_rotation


class Link:
    def __init__(self, axis, translational_offset=0.0, initial_frame_rotation: list | tuple = None, initial_state=0.0,
                 degrees=True):
        """
        Base class for robot links.

        Args:
            axis (str): 'x', 'y', 'z' for rotation or 'tx', 'ty', 'tz' for translation.
            translational_offset (float or list): Offset along the link's axis.
            initial_frame_rotation (list): Euler angles for initial frame rotation (x, y, z).
            initial_state (float): Initial angle (radians) or displacement (units).
        """
        self.degrees = degrees
        self.axis = axis
        self.state = initial_state
        self.transform = np.eye(4)  # Initialize as identity matrix

        self.translational_offset = translational_offset
        self.initial_frame_rotation = initial_frame_rotation
        self.update_transform()

    def update_transform(self):
        """Abstract method to be implemented by subclasses."""
        pass

    def set_state(self, new_state):
        """Set a new state for the link."""
        self.state = new_state
        self.update_transform()

    def axis_transform(self, degrees=True):
        # Apply the local rotation to the link's transform
        axis_rotation = np.eye(4)
        if self.initial_frame_rotation:
            axis_rotation = chained_rotations(self.initial_frame_rotation, degrees=degrees)

        axis_translation = translational_affine('tx', self.translational_offset)
        return axis_translation @ axis_rotation


class RotationalLink(Link):
    def update_transform(self):
        """Update the transformation matrix for a rotational link."""
        local_rotation = rotational_affine(self.axis, self.state, self.degrees)

        # Apply the local rotation to the link's transform
        self.transform = self.axis_transform(self.degrees) @ local_rotation


class TranslationalLink(Link):
    def update_transform(self):
        """Update the transformation matrix for a translational link."""
        local_translation = translational_affine(self.axis, self.state)

        self.transform = self.axis_transform(self.degrees) @ local_translation


if __name__ == "__main__":
    # Example 1: No rotation, translational offset of 1
    link1 = TranslationalLink('tz', initial_state=1.0, translational_offset=1)

    # Example 2: 90-degree rotation on x, translational offset of 1, initial rotations of 90 on x and y
    link2 = RotationalLink('x', initial_state=45, translational_offset=2,
                           initial_frame_rotation=[('z', 90), ('y', -90)])


    def plot_link(link, ax):
        """Visualize a link in 3D."""
        origin = [0, 0, 0]
        end_point = link.transform[:3, :3]@link.transform[:3, 3]
        ax.plot([origin[0], end_point[0]], [origin[1], end_point[1]], [origin[2], end_point[2]], '-o')


    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 2])
    ax.set_ylim([-1, 2])
    ax.set_zlim([-1, 2])

    plot_link(link1, ax)  # Plot the first link
    plot_link(link2, ax)  # Plot the second link

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('Link Visualization')
    plt.show()
