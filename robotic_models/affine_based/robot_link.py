import numpy as np
from scipy.spatial.transform import Rotation as R


class Link:
    def __init__(self, axis, initial_state=0.0):
        """
        Base class for robot links.

        Args:
            axis (str): 'x', 'y', 'z' for rotation or 'tx', 'ty', 'tz' for translation.
            initial_state (float): Initial angle (radians) or displacement (units).
        """
        self.axis = axis
        self.state = initial_state
        self.transform = np.eye(4)  # Initialize as identity matrix
        self.update_transform()

    def update_transform(self):
        """Abstract method to be implemented by subclasses."""
        pass

    def set_state(self, new_state):
        """Set a new state for the link."""
        self.state = new_state
        self.update_transform()


class RotationalLink(Link):
    def update_transform(self):
        """Update the transformation matrix for a rotational link."""
        rotation_matrix = None
        if self.axis == 'x':
            rotation_matrix = R.from_euler('x', self.state).as_matrix()
        elif self.axis == 'y':
            rotation_matrix = R.from_euler('y', self.state).as_matrix()
        elif self.axis == 'z':
            rotation_matrix = R.from_euler('z', self.state).as_matrix()

        if rotation_matrix is not None:
            self.transform[:3, :3] = rotation_matrix


class TranslationalLink(Link):
    def update_transform(self):
        """Update the transformation matrix for a translational link."""
        if self.axis == 'tx':
            self.transform[0, 3] = self.state
        elif self.axis == 'ty':
            self.transform[1, 3] = self.state
        elif self.axis == 'tz':
            self.transform[2, 3] = self.state



