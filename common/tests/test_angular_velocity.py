from common.angular_velocity import omega_from_v_r
from numpy import array
from unittest import TestCase


class Test(TestCase):
    def test_omega_from_v_r(self):
        v = array([2.0, 0.0])  # Linear velocity (m/s) along the x-axis
        r = array([0.0, 1.0])  # Radius vector (m) along the y-axis

        omega = omega_from_v_r(v, r)
        assert omega == -2.0, f"Expected -2.0, but got {omega}"
