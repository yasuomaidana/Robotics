from matplotlib import patches
from matplotlib.axes import Axes
from numpy import radians, cos, sin, ndarray

from omnidirectional_robot import OmnidirectionalRobot
from omni_wheel import OmniWheel


class RadialOmnidirectionalRobot(OmnidirectionalRobot):
    def __init__(self, num_wheels, radius, wheel_width=None, wheel_diameter=0.5, axis_rotation=0,
                 velocity_color='orange', wheel_color='black', motor_velocities: ndarray | list = None):
        super().__init__(num_wheels, velocity_color, wheel_color, motor_velocities)
        self.radius = radius
        self.axis_rotation = axis_rotation
        self.wheel_width = wheel_width or radius / 2
        self.wheel_diameter = wheel_diameter
        self.generate_wheels()

    def generate_wheels(self):
        angle_step = 360 / self.num_wheels
        for i in range(self.num_wheels):
            angle = i * angle_step + self.axis_rotation
            x = self.radius * cos(radians(angle))
            y = self.radius * sin(radians(angle))
            wheel = OmniWheel(distance_from_robot_frame=(x, y), orientation=angle, diameter=self.wheel_diameter,
                              width=self.wheel_width)
            self.add_wheel(wheel)

    def plot(self, ax: Axes, wheel_color=None, edge_color='black'):
        circle = patches.Circle(self.position, self.radius, edgecolor=edge_color, facecolor='none')
        ax.add_patch(circle)
        super().plot(ax, wheel_color)

    def plot_velocity(self, ax: Axes, wheel_color=None, velocity_color=None, global_velocity="green"):
        circle = patches.Circle(self.position, self.radius, edgecolor='black', facecolor='none')
        ax.add_patch(circle)
        super().plot_velocity(ax, wheel_color, velocity_color, global_velocity)
