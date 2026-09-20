import math

from manipulation_demo.interactive_target import quaternion_from_rpy
import pytest


def test_zero_rpy_is_identity_quaternion():
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


def test_half_turn_about_x_axis():
    qx, qy, qz, qw = quaternion_from_rpy(math.pi, 0.0, 0.0)
    assert qx == pytest.approx(1.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(0.0)
    assert qw == pytest.approx(0.0, abs=1e-12)
