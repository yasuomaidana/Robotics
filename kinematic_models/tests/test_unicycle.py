import unittest
from unittest import TestCase
from kinematic_models.wheeled_robots.unicycle_robot import UnicycleRobot


class TestUnicycleRobot(TestCase):

    def setUp(self):
        self.robot = UnicycleRobot(1, 1)

    def test_angular_velocity_equal_velocities(self):
        self.assertEqual(self.robot.angular_velocity([5, 5]), 0)

    def test_angular_velocity_zero_velocity(self):
        self.assertEqual(self.robot.angular_velocity([0, 5]), 2.5)

    def test_angular_velocity_different_directions(self):
        self.assertEqual(self.robot.angular_velocity([-5, 5]), 5)

    def test_invalid_wheel_distance(self):
        with self.assertRaises(ValueError):
            robot = UnicycleRobot(-1, 1)

    def test_zero_wheel_distance(self):
        with self.assertRaises(ValueError):
            robot = UnicycleRobot(0, 1)

    def test_invalid_wheel_diameter(self):
        with self.assertRaises(ValueError):
            robot = UnicycleRobot(1, -1)

    def test_zero_wheel_diameter(self):
        with self.assertRaises(ValueError):
            robot = UnicycleRobot(1, 0)

    def test_linear_velocity_equal_velocities(self):
        self.assertEqual(self.robot.linear_velocity([5, 5]), 2.5)

    def test_linear_velocity_different_velocities(self):
        self.assertEqual(self.robot.linear_velocity([5, 10]), 3.75)

    def test_linear_velocity_different_directions(self):
        self.assertEqual(self.robot.linear_velocity([-5, 5]), 0)

    def test_angular_velocity_invalid_motor_velocities(self):
        with self.assertRaises(ValueError):
            self.robot.angular_velocity([5])

    def test_linear_velocity_invalid_motor_velocities(self):
        with self.assertRaises(ValueError):
            self.robot.linear_velocity([5])


if __name__ == '__main__':
    unittest.main()
