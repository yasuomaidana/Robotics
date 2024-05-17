class OmnidirectionalRobot:
    def __init__(self, num_wheels, velocity_color='orange', wheel_color='black'):
        if num_wheels < 3:
            raise ValueError("Omnidirectional robot must have at least 3 wheels")
        self.num_wheels = num_wheels
        self.wheels = []
        self.velocity_color = velocity_color
        self.wheel_color = wheel_color

    def add_wheel(self, wheel):
        if len(self.wheels) < self.num_wheels:
            self.wheels.append(wheel)
        else:
            raise ValueError("All wheels have been added")

    def plot(self, ax):
        ax.set_aspect('equal')
        for wheel in self.wheels:
            wheel.plot(ax, color=self.wheel_color)

    def plot_velocity(self, robot_angular_velocity, ax):
        ax.set_aspect('equal')
        for wheel in self.wheels:
            wheel.plot(ax, color=self.wheel_color)
            wheel.plot_velocity(ax, robot_angular_velocity, color=self.velocity_color)
