class UnicycleRobot:
    def __init__(self, wheel_distance, wheel_diameter):
        if wheel_distance <= 0:
            raise ValueError("Wheel distance must be a positive number")
        if wheel_diameter <= 0:
            raise ValueError("Wheel diameter must be a positive number")
        self.wheel_distance = wheel_distance
        self.wheel_diameter = wheel_diameter

    def angular_velocity(self, motor_angular_velocities):
        if len(motor_angular_velocities) != 2:
            raise ValueError("Exactly two motor angular velocities are required")
        return (motor_angular_velocities[1] - motor_angular_velocities[0]) * (self.wheel_diameter /2) / self.wheel_distance

    def linear_velocity(self, motor_angular_velocities):
        if len(motor_angular_velocities) != 2:
            raise ValueError("Exactly two motor angular velocities are required")
        return (motor_angular_velocities[0] + motor_angular_velocities[1]) / 2 * (self.wheel_diameter / 2)
