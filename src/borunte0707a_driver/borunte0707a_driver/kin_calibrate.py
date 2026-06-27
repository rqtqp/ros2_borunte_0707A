"""Kinematic offset calibration against the controller's own TCP readout.

The HC1 reports the live tool pose in Cartesian (`world-0..5`: X,Y,Z mm,
orientation deg) from its factory-calibrated kinematics. We use that as ground
truth: collect (joint, TCP) pairs at several poses you jog to, then least-squares
solve the URDF joint OFFSETs (plus the unknown base<->world and tool<->flange
transforms) so the URDF forward kinematics reproduce the measured TCP positions.

This removes all eyeballing -- the offsets come out of the controller's own
geometry. Read-only here (we only query; you jog from the pendant).

Workflow:
    # jog to a pose on the pendant, then:
    ros2 run borunte0707a_driver kin_calibrate --collect poses.json
    # repeat for 8-12 varied poses (exercise J2/J3/J5, and some J4/J6), then:
    ros2 run borunte0707a_driver kin_calibrate --solve poses.json

Only J2/J3/J5 offsets are solved (J1/J4/J6 confirmed 0 on this arm); fit residual
is reported in mm. If residual is small (~a few mm), paste the printed OFFSET_RAD
into calibration.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os

from borunte0707a_driver.calibration import SIGN
from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client

# Resolved URDF chain (meters), root->tool0, from
# brtirus0707a_description/urdf/brtirus0707a.urdf.xacro (all joint rpy = 0).
# Each entry: (translation xyz, rotation axis) for joints 1..6.
URDF_JOINTS = [
    ((-0.064848, 0.0579775, 0.055131), (0.0, 0.0, 1.0)),
    ((0.057, 0.0499935, 0.241322), (-1.0, 0.0, 0.0)),
    ((0.0065, -0.003601, 0.350042), (-1.0, 0.0, 0.0)),
    ((-0.062946, 0.101, 0.040925), (0.0, 1.0, 0.0)),
    ((0.0425, 0.262, 0.0), (-1.0, 0.0, 0.0)),
    ((-0.041, 0.1013, 0.000274), (0.0, -1.0, 0.0)),
]
# Joints whose offset we solve (0-indexed). J1/J4/J6 confirmed aligned (offset 0).
FREE_OFFSET_IDX = [1, 2, 4]


def _np():
    try:
        import numpy as np
        return np
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("kin_calibrate needs numpy (pip/apt install python3-numpy)") from exc


def _rot(np, axis, theta):
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c, s, C = math.cos(theta), math.sin(theta), 1 - math.cos(theta)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def fk(np, q):
    """URDF forward kinematics root->tool0. q: 6 joint angles (rad). Returns R,p."""
    R = np.eye(3)
    p = np.zeros(3)
    for i, (xyz, axis) in enumerate(URDF_JOINTS):
        p = p + R @ np.asarray(xyz, dtype=float)
        R = R @ _rot(np, axis, q[i])
    return R, p


def joints_from_axis(np, axis_deg, offset_rad):
    return [SIGN[i] * math.radians(axis_deg[i]) + offset_rad[i] for i in range(6)]


def _unpack(np, params):
    """params -> (offset_rad[6], R_base, t_base_mm, t_tool_m)."""
    from scipy.spatial.transform import Rotation
    offset = [0.0] * 6
    for k, idx in enumerate(FREE_OFFSET_IDX):
        offset[idx] = params[k]
    n = len(FREE_OFFSET_IDX)
    rotvec = params[n:n + 3]
    t_base = np.asarray(params[n + 3:n + 6])      # mm
    t_tool = np.asarray(params[n + 6:n + 9])      # m
    R_base = Rotation.from_rotvec(rotvec).as_matrix()
    return offset, R_base, t_base, t_tool


def predict_world(np, params, samples):
    offset, R_base, t_base, t_tool = _unpack(np, params)
    out = []
    for s in samples:
        q = joints_from_axis(np, s["axis"], offset)
        R_fk, p_fk = fk(np, q)
        p_root_mm = (p_fk + R_fk @ t_tool) * 1000.0
        out.append(R_base @ p_root_mm + t_base)
    return np.array(out)


def residuals(np, params, samples):
    pred = predict_world(np, params, samples)
    meas = np.array([s["world"][:3] for s in samples])
    return (pred - meas).reshape(-1)


def collect(client: HC1Client, path: str) -> None:
    addrs = [f"axis-{i}" for i in range(6)] + [f"world-{i}" for i in range(6)]
    data = client.query(addrs)
    sample = {
        "axis": [float(data[f"axis-{i}"]) for i in range(6)],
        "world": [float(data[f"world-{i}"]) for i in range(6)],
    }
    samples = []
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            samples = json.load(fh)
    samples.append(sample)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(samples, fh, indent=2)
    print(f"collected sample {len(samples)}: "
          f"axis={['%.1f' % a for a in sample['axis']]}  "
          f"TCP_mm={['%.0f' % w for w in sample['world'][:3]]}")
    print(f"-> {path}. Jog to a new pose and run --collect again "
          f"(aim for 8-12 varied poses).")


def solve(path: str) -> None:
    np = _np()
    from scipy.optimize import least_squares
    with open(path, encoding="utf-8") as fh:
        samples = json.load(fh)
    if len(samples) < 5:
        print(f"WARNING: only {len(samples)} samples; collect >=8 varied poses "
              f"for a reliable fit.")

    # Initial guess: current offsets, identity base rotation, tool at flange.
    from borunte0707a_driver.calibration import OFFSET_RAD
    p0 = [OFFSET_RAD[i] for i in FREE_OFFSET_IDX] + [0.0, 0.0, 0.0]
    # Seed t_base so the first sample's residual starts near zero.
    offset0 = list(OFFSET_RAD)
    R_fk0, p_fk0 = fk(np, joints_from_axis(np, samples[0]["axis"], offset0))
    t_base0 = np.array(samples[0]["world"][:3]) - p_fk0 * 1000.0
    p0 = p0 + list(t_base0) + [0.0, 0.0, 0.0]

    res = least_squares(
        lambda p: residuals(np, p, samples), p0, method="lm", max_nfev=20000
    )
    offset, R_base, t_base, t_tool = _unpack(np, res.x)
    err = residuals(np, res.x, samples).reshape(-1, 3)
    norms = np.linalg.norm(err, axis=1)

    print(f"\nfit over {len(samples)} poses | per-pose position error (mm):")
    for i, n in enumerate(norms):
        print(f"  pose {i + 1:2d}: {n:6.2f} mm")
    print(f"\nRMS position error: {math.sqrt((norms ** 2).mean()):.2f} mm  "
          f"(max {norms.max():.2f} mm)")
    print(f"base rotation (deg): "
          f"{['%.2f' % d for d in _rotvec_deg(np, res.x)]}")
    print(f"tool offset from flange (mm): {['%.1f' % (t * 1000) for t in t_tool]}")
    print("\nSolved OFFSET_RAD (paste into calibration.py):")
    print(f"  OFFSET_RAD = ({', '.join('%.5f' % o for o in offset)})")
    deg = [math.degrees(o) for o in offset]
    print(f"  # degrees: ({', '.join('%.2f' % d for d in deg)})")
    if norms.max() > 15:
        print("\nNOTE: max error > 15 mm -- add more varied poses (especially "
              "exercising J2/J3/J5), or check the URDF link lengths.")


def _rotvec_deg(np, params):
    n = len(FREE_OFFSET_IDX)
    rv = np.asarray(params[n:n + 3])
    return [math.degrees(v) for v in rv]


def main() -> None:
    env = load_env()
    parser = argparse.ArgumentParser(description="Cartesian kinematic offset calibration.")
    parser.add_argument("--collect", metavar="FILE", help="append current (joint,TCP) sample")
    parser.add_argument("--solve", metavar="FILE", help="fit offsets from collected samples")
    parser.add_argument("--ip", default=env.get("ROBOT_IP", ""))
    parser.add_argument("--port", type=int, default=int(env.get("REMOTE_MONITOR_PORT", "9760")))
    parser.add_argument("--timeout", type=float,
                        default=float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
    args = parser.parse_args()

    if args.solve:
        solve(args.solve)
    elif args.collect:
        if not args.ip:
            parser.error("no robot IP (set ROBOT_IP in .env or pass --ip)")
        collect(HC1Client(args.ip, args.port, args.timeout), args.collect)
    else:
        parser.error("use --collect FILE (per pose) then --solve FILE")


if __name__ == "__main__":
    main()
