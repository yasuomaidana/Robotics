from unittest import TestCase

from robotic_models.affine_based.robot_link import Link


class TestRobotLink(TestCase):
    def test__validate_axis(self):
        self.assertRaises(ValueError, Link, 'a')
