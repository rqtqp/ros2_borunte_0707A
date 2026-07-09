"""calibration.py: sign/offset round-trip and soft limits (pure, no ROS)."""
import math

from borunte0707a_driver import calibration
from borunte0707a_driver.calibration import (
    NUM_JOINTS, SIGN, OFFSET_RAD, SOFT_LIMITS_DEG,
    controller_deg_to_urdf_rad, urdf_rad_to_controller_deg, within_soft_limits,
)


def test_round_trip_identity():
    axis = [12.3, -45.6, 78.9, -101.1, 55.5, -170.0]
    back = urdf_rad_to_controller_deg(controller_deg_to_urdf_rad(axis))
    assert all(abs(axis[i] - back[i]) < 1e-9 for i in range(NUM_JOINTS))


def test_round_trip_identity_reverse():
    q = [0.5, -1.0, 0.9, 2.0, -1.5, 3.0]
    back = controller_deg_to_urdf_rad(urdf_rad_to_controller_deg(q))
    assert all(abs(q[i] - back[i]) < 1e-9 for i in range(NUM_JOINTS))


def test_pinned_zero_maps_to_offsets_only():
    # Controller mechanical home (all axis-N == 0) must land on the URDF zero
    # up to the small measured offsets -- the authoritative pin-groove zero.
    q = controller_deg_to_urdf_rad([0.0] * NUM_JOINTS)
    assert all(abs(q[i] - OFFSET_RAD[i]) < 1e-12 for i in range(NUM_JOINTS))
    assert all(abs(o) < math.radians(0.75) for o in OFFSET_RAD)


def test_sign_map_matches_urdf_flips():
    assert SIGN == (1.0, -1.0, -1.0, 1.0, -1.0, -1.0)


def test_soft_limits_pass_and_violations():
    ok, violations = within_soft_limits([0.0] * NUM_JOINTS)
    assert ok and not violations
    bad = [0.0] * NUM_JOINTS
    bad[0] = SOFT_LIMITS_DEG[0][1] + 1.0   # J1 above upper
    bad[2] = SOFT_LIMITS_DEG[2][0] - 1.0   # J3 below lower
    ok, violations = within_soft_limits(bad)
    assert not ok
    assert [v[0] for v in violations] == [0, 2]
    idx, value, lo, hi = violations[0]
    assert value == bad[0] and (lo, hi) == SOFT_LIMITS_DEG[0]


def test_soft_limits_boundaries_inclusive():
    edge = [SOFT_LIMITS_DEG[i][1] for i in range(NUM_JOINTS)]
    ok, _ = within_soft_limits(edge)
    assert ok
