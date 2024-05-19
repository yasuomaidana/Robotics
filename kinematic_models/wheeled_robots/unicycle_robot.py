from numpy import cos, sin


class UnicycleRobot:
    def __init__(self, wheel_distance, wheel_diameter):
        if wheel_distance <= 0:
            raise ValueError("Wheel distance must be a positive number")
        if wheel_diameter <= 0:
            raise ValueError("Wheel diameter must be a positive number")
        self.wheel_distance = wheel_distance
        self.wheel_diameter = wheel_diameter
        self.x = 0
        self.y = 0
        self.theta = 0

    def angular_velocity(self, motor_angular_velocities):
        if len(motor_angular_velocities) != 2:
            raise ValueError("Exactly two motor angular velocities are required")
        return (motor_angular_velocities[1] - motor_angular_velocities[0]) * (
                self.wheel_diameter / 2) / self.wheel_distance

    def linear_velocity(self, motor_angular_velocities):
        if len(motor_angular_velocities) != 2:
            raise ValueError("Exactly two motor angular velocities are required")
        return (motor_angular_velocities[0] + motor_angular_velocities[1]) / 2 * (self.wheel_diameter / 2)

    def update_position(self, motor_angular_velocities, time_step):
        if len(motor_angular_velocities) != 2:
            raise ValueError("Exactly two motor angular velocities are required")
        v = self.linear_velocity(motor_angular_velocities)
        w = self.angular_velocity(motor_angular_velocities)
        self.x += v * cos(self.theta) * time_step
        self.y += v * sin(self.theta) * time_step
        self.theta += w * time_step
