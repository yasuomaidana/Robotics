import numpy as np
import matplotlib.patches as patches
from numpy import array
from matplotlib.axes import Axes
from common.affine import affine_matrix_from_rotation_and_translation, rotational_affine


class OmniWheel:
    def __init__(self, distance_from_robot_frame=(0, 0), orientation=0, diameter=1, width=0.5, velocity=0):
        self.distance_from_robot_frame = distance_from_robot_frame
        self.orientation = orientation
        self.diameter = diameter
        self.width = width
        self.velocity = velocity

    def plot(self, ax: Axes, robot_position=array([0, 0]), robot_orientation=0, color='black'):
        robot_rotation = rotational_affine('z', robot_orientation)[:2, :2]
        wheel_position: np.ndarray[float, float] = robot_rotation @ self.distance_from_robot_frame + robot_position
        wheel_rotation = self.orientation + robot_orientation

        wheel_patch = patches.Rectangle(
            (wheel_position[0] - self.diameter / 2, wheel_position[1] - self.width / 2),
            self.diameter, self.width, angle=wheel_rotation, color=color, rotation_point="center")

        ax.add_patch(wheel_patch)

    def get_affine_matrix(self):
        return affine_matrix_from_rotation_and_translation(self.orientation, np.array(self.distance_from_robot_frame))

    def calculate_velocity(self, robot_angular_velocity):
        wheel_radius = self.diameter / 2
        wheel_velocity = robot_angular_velocity * wheel_radius
        return wheel_velocity

    def plot_velocity(self, ax: Axes, robot_position=array([0, 0]), robot_orientation=0, color='orange'):
        robot_rotation = rotational_affine('z', robot_orientation)[:2, :2]
        wheel_position: np.ndarray[float, float] = robot_rotation @ self.distance_from_robot_frame + robot_position

        vx, vy = rotational_affine('z', robot_orientation)[:2, :2] @ self.get_velocity_components(self.velocity)

        ax.quiver(wheel_position[0], wheel_position[1],
                  vx, vy, scale=1 if vx == vy == 0 else None, color=color)

    def get_velocity_components(self, robot_angular_velocity):
        wheel_velocity = self.calculate_velocity(robot_angular_velocity)
        affine_matrix = self.get_affine_matrix()
        wheel_direction = affine_matrix[:2, 0]
        rotational = rotational_affine('z', 90)[:2, :2] @ wheel_direction * wheel_velocity
        return rotational
