import numpy as np
from numpy import array, ndarray, cos, sin, tan, zeros, linspace
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from numpy.linalg import inv


class AckermannCar:
    def __init__(self, state: ndarray = zeros(6), wheelbase=2.5, width=1.5, length=4.5):
        self.state = state
        self.wheelbase = wheelbase  # distance between front and rear axles
        self.width = width  # car width
        self.length = length  # car length
        self.steering_angle = 0  # current steering angle (radians)
        self.velocity = 0  # current velocity (m/s)

    def kinematic_matrix(self, front: bool = False):
        yaw = self.state[2]
        angle = yaw + front * self.steering_angle
        return array([[cos(angle), -sin(angle), 0],
                      [sin(angle), cos(angle), 0],
                      [0, 0, 1]])

    def body_velocity_from_state(self, velocities: ndarray, front: bool = False):
        return inv(self.kinematic_matrix(front)) @ velocities

    def front_velocity(self, velocity, steering_angle):
        vx = velocity * np.cos(self.yaw + steering_angle)
        vy = velocity * np.sin(self.yaw + steering_angle)
        omega = velocity * np.tan(steering_angle) / self.wheelbase
        return vx, vy, omega

    def back_velocity(self, velocity, steering_angle):
        vx = velocity * np.cos(self.yaw)
        vy = velocity * np.sin(self.yaw)
        omega = velocity * np.tan(steering_angle) / self.wheelbase
        return vx, vy, omega

    def center_velocity(self, velocity, steering_angle):
        front_vx, front_vy, front_omega = self.front_velocity(velocity, steering_angle)
        back_vx, back_vy, back_omega = self.back_velocity(velocity, steering_angle)
        vx = (front_vx + back_vx) / 2
        vy = (front_vy + back_vy) / 2
        omega = (front_omega + back_omega) / 2
        return vx, vy, omega

    def update(self, velocity, steering_angle, dt):
        self.steering_angle = steering_angle
        v = array([velocity, 0, velocity * np.tan(steering_angle) / self.wheelbase])
        # Update car's position and orientation
        v_body = self.body_velocity_from_state(v)
        self.state[0:3] += dt * self.kinematic_matrix() @ v
        self.state[3:6] = v_body

    def draw(self, ax):
        # Car body
        body_x = self.x - (self.length / 2) * np.cos(self.yaw)
        body_y = self.y - (self.length / 2) * np.sin(self.yaw)
        body_rect = patches.Rectangle((body_x, body_y), self.length, self.width, angle=np.rad2deg(self.yaw),
                                      color="lightblue")
        ax.add_patch(body_rect)

        # Front and rear axle lines
        ax.plot([self.x, self.x + self.wheelbase * np.cos(self.yaw)],
                [self.y, self.y + self.wheelbase * np.sin(self.yaw)],
                color="black")

        # Steering angle visualization (optional)
        steering_line_x = self.x + self.wheelbase * np.cos(self.yaw)
        steering_line_y = self.y + self.wheelbase * np.sin(self.yaw)
        ax.plot([steering_line_x, steering_line_x + 0.5 * np.cos(self.yaw + self.steering_angle)],
                [steering_line_y, steering_line_y + 0.5 * np.sin(self.yaw + self.steering_angle)],
                color="red", linestyle="dashed")



