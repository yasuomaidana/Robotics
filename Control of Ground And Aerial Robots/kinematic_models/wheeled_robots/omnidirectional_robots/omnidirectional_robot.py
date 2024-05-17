from matplotlib import pyplot as plt


class OmnidirectionalRobot:
    def __init__(self, num_wheels):
        if num_wheels < 3:
            raise ValueError("Omnidirectional robot must have at least 3 wheels")
        self.num_wheels = num_wheels
        self.wheels = []

    def add_wheel(self, wheel):
        if len(self.wheels) < self.num_wheels:
            self.wheels.append(wheel)
        else:
            raise ValueError("All wheels have been added")

    def plot(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')

        for wheel in self.wheels:
            wheel.plot(ax)

        plt.show()

    def plot_velocity(self, robot_angular_velocity):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')

        for wheel in self.wheels:
            wheel.plot(ax)
            wheel.plot_velocity(ax, robot_angular_velocity)

        plt.show()
