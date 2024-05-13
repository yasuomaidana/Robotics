from unittest import TestCase

from robotic_models.affine_based.robot_link import RobotLink


class TestRobotLink(TestCase):
    def test__validate_axis(self):
        self.assertRaises(ValueError, RobotLink, 'a')
