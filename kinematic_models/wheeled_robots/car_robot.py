import numpy as np
from numpy import array, ndarray, cos, sin, tan, zeros, deg2rad, rad2deg
from matplotlib.axes import Axes
import matplotlib.patches as patches
from numpy.linalg import inv


class AckermannCar:
    def __init__(self, state: ndarray = None, wheelbase=2.5, width=1.5, length=4.5, degrees=True):
        if state is None:
            state = zeros(6)
        self.state: ndarray = state
        self.wheelbase = wheelbase  # distance between front and rear axles
        self.width = width  # car width
        self.length = length  # car length
        self.steering_angle = 0  # current steering angle (radians)
        self.velocity = 0  # current velocity (m/s)
        self.degrees = degrees
        self.state[2] = self.yaw

    def kinematic_matrix(self, front: bool = False):
        yaw = self.state[2]
        angle = yaw + front * (deg2rad(self.steering_angle) if self.degrees else self.steering_angle)
        return array([[cos(angle), -sin(angle), 0],
                      [sin(angle), cos(angle), 0],
                      [0, 0, 1]])

    def get_robot_velocity(self, velocity: float = None, steering_angle: float = None):
        if velocity is None:
            velocity = self.velocity
        if steering_angle is None:
            steering_angle = self.steering_angle
        else:
            steering_angle = deg2rad(steering_angle) if self.degrees else steering_angle
        return array([velocity, 0, velocity * tan(steering_angle) / self.wheelbase])

    def body_velocity_from_state(self, velocities: ndarray = None, front: bool = False):
        if velocities is None:
            velocities = self.state[3:6]
        return inv(self.kinematic_matrix(front)) @ velocities

    def front_velocity(self, velocity: float = None, steering_angle: float = None):
        return self.kinematic_matrix(front=True) @ self.get_robot_velocity(velocity, steering_angle)

    def back_velocity(self, velocity, steering_angle):
        return self.kinematic_matrix(front=False) @ self.get_robot_velocity(velocity, steering_angle)

    def center_velocity(self, velocity, steering_angle):
        return (self.front_velocity(velocity, steering_angle) + self.back_velocity(velocity, steering_angle)) / 2

    def update(self, velocity: float, steering_angle: float, dt: float):
        self.steering_angle = steering_angle
        self.velocity = velocity
        v_robot = self.get_robot_velocity(velocity, steering_angle)
        # Update car's position and orientation
        v_body = self.body_velocity_from_state(v_robot)
        self.state[0:3] += dt * self.kinematic_matrix() @ v_robot
        self.state[3:6] = v_body
        self.state[2] = np.mod(self.state[2], 2 * np.pi)

    def draw(self, ax: Axes):
        # Car body
        body_x = float(self.state[0]) - self.length / 2
        body_y = float(self.state[1]) - self.width / 2
        body_rect = patches.Rectangle((body_x, body_y), self.length, self.width, angle=np.rad2deg(self.state[2]),
                                      rotation_point="center",
                                      color="lightblue")
        ax.add_patch(body_rect)

        # Front and rear axle lines
        ax.plot([self.state[0], self.state[0] + self.wheelbase * np.cos(self.state[2])],
                [self.state[1], self.state[1] + self.wheelbase * np.sin(self.state[2])],
                color="black")

        # Steering angle visualization (optional)
        steering_line_x = self.state[0] + self.wheelbase * np.cos(self.state[2])
        steering_line_y = self.state[1] + self.wheelbase * np.sin(self.state[2])
        ax.plot([steering_line_x, steering_line_x + 0.5 * np.cos(self.state[2] + self.steering_angle)],
                [steering_line_y, steering_line_y + 0.5 * np.sin(self.state[2] + self.steering_angle)],
                color="red", linestyle="dashed")

    @property
    def yaw(self):
        """Yaw angle in radians or degrees, depending on configuration."""
        return rad2deg(self.state[2]) if self.degrees else self.state[2]
