"""Controller <-> URDF joint calibration (sign + zero offset) and soft limits.

Single source of truth shared by the state publisher (controller -> URDF) and
the motion bridge (URDF -> controller), so telemetry and commands round-trip
consistently.

## Why a sign map exists

The URDF flips the rotation axis on joints 2, 3, 5, 6 (`axis -1 0 0`, `0 -1 0`),
while joint 1 (`0 0 1`) and joint 4 (`0 1 0`) share the controller's positive
direction. The vendor joint *ranges* confirm this: e.g. J2 vendor -125..85 deg
maps exactly onto the URDF limit -2.182..1.484 rad under a pure sign flip with
**zero** offset (see CLAUDE.md Joint Map + reference/HC1_DEBUG_REFERENCE.md).

    q_urdf_rad = SIGN[i] * radians(axis_deg[i]) + OFFSET_RAD[i]
    axis_deg[i] = SIGN[i] * degrees(q_urdf_rad - OFFSET_RAD[i])     (SIGN == +/-1)

SIGN was derived from the URDF axes + vendor-vs-URDF limit ranges. OFFSET was
then measured rigorously with `kin_calibrate` (least-squares fit of the URDF
forward kinematics to the controller's own TCP readout, `world-0..5`, over 11
poses): the offsets came out ~0 (|<0.7 deg| each, RMS fit 0.73 mm), confirming
the URDF zero pose DOES coincide with the controller's mechanical home. Both SIGN
and OFFSET_RAD are overridable as ROS parameters on the bridge so a field
re-calibration never requires a code change.

AUTHORITATIVE ZERO -- the arm has factory dowel/pin grooves on each joint;
seating the calibration pins locks the mechanical zero, which equals controller
axis-N == 0 AND the URDF zero pose (verified: pins seated -> all axis-N ~0 ->
model at q=0). Always begin any (re)calibration from pins-seated. An eyeballed
"home" is NOT a reliable reference -- it can be tens of degrees off on J2/J3/J5
and will send a sign/offset fit down the wrong path.

(The fit also recovered a -90 deg yaw between the URDF root frame and the
controller's Cartesian *world* frame, plus a ~17 mm tool offset -- those matter
only for Cartesian/world comparisons, not joint-space display or commands.)
"""

from __future__ import annotations

import math

NUM_JOINTS = 6

# Per-joint sign, controller axis-N -> URDF joint direction (see module docstring).
SIGN = (1.0, -1.0, -1.0, 1.0, -1.0, -1.0)

# Per-joint zero offset, radians, added in the controller->URDF direction.
# Measured by kin_calibrate (URDF FK fitted to the controller's TCP over 11
# poses, RMS 0.73 mm). All offsets are ~0 -- the URDF zero matches the mechanical
# home. J1/J4/J6 fixed at 0; J2/J3/J5 are the tiny solved residuals (<0.7 deg).
OFFSET_RAD = (0.0, 0.01134, -0.01206, 0.0, 0.01114, 0.0)

# Soft limits in *controller* degrees (axis-N space), from the factory
# commissioning log (reference/HC1_DEBUG_REFERENCE.md). Tighter than the hard
# limits; targets are validated against these before any AddRCC.
SOFT_LIMITS_DEG = (
    (-155.0, 155.0),   # J1 base
    (-105.0, 75.0),    # J2 shoulder
    (-45.0, 160.0),    # J3 elbow
    (-160.0, 160.0),   # J4 wrist pitch
    (-100.0, 100.0),   # J5 wrist roll
    (-340.0, 340.0),   # J6 wrist yaw
)


def controller_deg_to_urdf_rad(axis_deg, sign=SIGN, offset_rad=OFFSET_RAD):
    """Map a list of controller axis-N degrees to URDF joint radians."""
    return [
        sign[i] * math.radians(axis_deg[i]) + offset_rad[i]
        for i in range(NUM_JOINTS)
    ]


def urdf_rad_to_controller_deg(q_rad, sign=SIGN, offset_rad=OFFSET_RAD):
    """Map a list of URDF joint radians to controller axis-N degrees."""
    return [
        sign[i] * math.degrees(q_rad[i] - offset_rad[i])
        for i in range(NUM_JOINTS)
    ]


def within_soft_limits(axis_deg, limits=SOFT_LIMITS_DEG):
    """Return (ok, violations). violations: list of (index, value, lo, hi)."""
    violations = []
    for i in range(NUM_JOINTS):
        lo, hi = limits[i]
        if not (lo <= axis_deg[i] <= hi):
            violations.append((i, axis_deg[i], lo, hi))
    return (not violations, violations)
