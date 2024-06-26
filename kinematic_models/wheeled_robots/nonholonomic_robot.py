from matplotlib.axes import Axes
from numpy import ndarray, zeros, deg2rad, cos, sin, eye

from kinematic_models.wheeled_robots.car_robot import AckermannCar


class NonholonomicRobot(AckermannCar):
    def __init__(self, state: ndarray = None, wheelbase=2.5, width=1.5, length=4.5, degrees=True,
                 arm_offset: ndarray = None, arm_length: float = 1.0, arm_angle: float = 0):
        super().__init__(state, wheelbase, width, length, degrees)
        if arm_offset is None:
            arm_offset = zeros(2)
        self.arm_offset = arm_offset
        self.arm_length = arm_length
        self.arm_angle = deg2rad(arm_angle) if self.degrees else arm_angle

    @property
    def transformed_arm_offset(self):
        return self.kinematic_matrix()[:2, :2] @ self.arm_offset

    def arm_kinematic_matrix(self):
        """Calculates the arm's kinematic matrix relative to the car's base frame."""
        yaw = self.state[2]
        arm_angle_rad = self.arm_angle

        kinematic_matrix = eye(4)

        kinematic_matrix[:2, :2] = self.kinematic_matrix()[:2, :2]

        kinematic_matrix[0, 2] = -self.arm_length * sin(yaw + arm_angle_rad)
        kinematic_matrix[1, 2] = self.arm_length * cos(yaw + arm_angle_rad)
        kinematic_matrix[0:2, 3] = self.transformed_arm_offset

        return kinematic_matrix

    def draw(self, ax: Axes):
        super().draw(ax)

        # Calculate arm end points
        arm_angle_rad = self.arm_angle

        # Calculate the offset point relative to the car's reference frame
        offset_x, offset_y = self.transformed_arm_offset

        arm_base_x = self.state[0] + offset_x
        arm_base_y = self.state[1] + offset_y

        arm_end_x = arm_base_x + self.arm_length * cos(self.state[2] + arm_angle_rad)
        arm_end_y = arm_base_y + self.arm_length * sin(self.state[2] + arm_angle_rad)

        # Draw the arm
        ax.plot([arm_base_x, arm_end_x], [arm_base_y, arm_end_y], color="green", linewidth=2)
