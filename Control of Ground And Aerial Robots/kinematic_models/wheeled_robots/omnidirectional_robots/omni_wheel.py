import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches


class OmniWheel:
    def __init__(self, diameter, width, distance_from_robot_frame, orientation):
        self.diameter = diameter
        self.width = width
        self.distance_from_robot_frame = distance_from_robot_frame
        self.orientation = orientation

    def plot(self):
        fig, ax = plt.subplots()
        wheel_patch = patches.Rectangle((self.distance_from_robot_frame, -self.width / 2), self.diameter, self.width,
                                        angle=self.orientation, color='black')
        ax.add_patch(wheel_patch)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect('equal')
        plt.show()
