"""Read-only sign/offset calibration helper. NO motion -- query only.

Two jobs, both safe to run anytime:

  monitor (default)  Live table of each joint's controller angle (axis-N deg)
                     and the URDF value the current calibration produces. Jog one
                     joint at a time from the PENDANT and watch the delta: the
                     URDF delta must have the direction & magnitude you expect.
                     This verifies SIGN.

  --capture-zero     Snapshot the current pose, assuming the arm is parked at the
                     URDF "home" (all joints 0, per the SRDF group_state). Prints
                     the OFFSET_RAD that makes the current axis-N map to URDF zero,
                     and whether it differs from the calibration in use. This
                     measures OFFSET.

Examples:
  python3 -m borunte0707a_driver.calibration_helper            # monitor (Ctrl-C)
  python3 -m borunte0707a_driver.calibration_helper --duration 20
  python3 -m borunte0707a_driver.calibration_helper --capture-zero
"""

from __future__ import annotations

import argparse
import math
import time

from borunte0707a_driver import calibration as cal
from borunte0707a_driver.calibration import NUM_JOINTS
from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client


def _read_axes(client: HC1Client):
    data = client.query([f"axis-{i}" for i in range(NUM_JOINTS)])
    return [float(data[f"axis-{i}"]) for i in range(NUM_JOINTS)]


def monitor(client: HC1Client, rate_hz: float, duration: float | None) -> None:
    """Print a line whenever a joint moves, showing the calibrated URDF delta.

    Each line shows the controller delta and the URDF delta the calibration
    produces. For joints with SIGN=-1 (J2/J3/J5/J6) the URDF deliberately moves
    OPPOSITE the controller -- that is the configured behaviour, not an error.
    The tool checks two things it actually can: that the right joint moved (index
    mapping) and that |URDF delta| == |controller delta| (1:1 scale; flags
    'SCALE?' otherwise). The SIGN itself is confirmed VISUALLY in RViz: jog the
    arm and confirm the model moves the same way.
    """
    period = 1.0 / rate_hz
    baseline = _read_axes(client)
    last_printed = list(baseline)
    print("Monitoring (read-only). Jog ONE joint at a time on the pendant.")
    print("Then confirm in RViz that the model moves the SAME way as the arm "
          "(that is the real sign check).\nCtrl-C to finish.\n")
    print(f"baseline axis_deg: {['%.2f' % a for a in baseline]}")
    start = time.monotonic()
    try:
        while duration is None or (time.monotonic() - start) < duration:
            axis = _read_axes(client)
            moved = [
                i for i in range(NUM_JOINTS)
                if abs(axis[i] - last_printed[i]) > 0.05
            ]
            if moved:
                q_now = cal.controller_deg_to_urdf_rad(axis)
                q_base = cal.controller_deg_to_urdf_rad(baseline)
                for i in moved:
                    d_ctrl = axis[i] - baseline[i]
                    d_urdf = math.degrees(q_now[i] - q_base[i])
                    sign = "+1" if cal.SIGN[i] > 0 else "-1, URDF opposite by design"
                    scale = "ok" if abs(abs(d_urdf) - abs(d_ctrl)) < 0.1 else "SCALE?"
                    print(f"  J{i + 1}: ctrl Δ{d_ctrl:+7.2f}  ->  URDF Δ{d_urdf:+7.2f}  "
                          f"sign {sign}  scale {scale}")
                last_printed = list(axis)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    print("\nDone. Index mapping + 1:1 scale checked here; confirm SIGN visually "
          "in RViz (model direction == arm direction).")


def capture_zero(client: HC1Client) -> None:
    """Compute OFFSET_RAD that maps the current (home) pose to URDF zero.

    NOTE: this is only valid if the URDF zero pose coincides with the
    controller's mechanical home. On this arm it does (kin_calibrate confirmed
    offsets ~0, RMS 0.73 mm), so at home this correctly reports ~0. For the
    authoritative offset fit use `kin_calibrate` (Cartesian TCP), not this.
    """
    axis = _read_axes(client)
    # q_urdf = sign*rad(axis) + offset == 0  ->  offset = -sign*rad(axis)
    offsets = [-cal.SIGN[i] * math.radians(axis[i]) for i in range(NUM_JOINTS)]
    print("Captured pose (assuming arm is at URDF home = all joints 0):\n")
    print(f"{'J':>2} {'axis_deg':>10} {'implied_offset_rad':>20} "
          f"{'current_offset':>16}  status")
    near_zero = True
    for i in range(NUM_JOINTS):
        cur = cal.OFFSET_RAD[i]
        delta = abs(offsets[i] - cur)
        ok = "ok" if abs(offsets[i]) < math.radians(0.5) else "NON-ZERO"
        if abs(offsets[i]) >= math.radians(0.5):
            near_zero = False
        print(f"J{i + 1:<1} {axis[i]:10.3f} {offsets[i]:20.5f} "
              f"{cur:16.5f}  {ok} (Δ{delta:.4f})")
    print()
    if near_zero:
        print("All implied offsets ~0: the controller home matches URDF zero. "
              "OFFSET_RAD=0 is confirmed; no change needed.")
    else:
        print("Non-zero offsets detected. To use them, pass to the bridge:")
        joined = ",".join(f"{o:.6f}" for o in offsets)
        print(f"  --ros-args -p offset_rad:=[{joined}]")
        print("or update OFFSET_RAD in calibration.py after sanity-checking.")


def main() -> None:
    env = load_env()
    parser = argparse.ArgumentParser(description="Read-only sign/offset calibration helper.")
    parser.add_argument("--ip", default=env.get("ROBOT_IP", ""))
    parser.add_argument("--port", type=int, default=int(env.get("REMOTE_MONITOR_PORT", "9760")))
    parser.add_argument("--timeout", type=float,
                        default=float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
    parser.add_argument("--rate", type=float, default=5.0, help="monitor poll rate (Hz)")
    parser.add_argument("--duration", type=float, default=None,
                        help="monitor seconds (default: until Ctrl-C)")
    parser.add_argument("--capture-zero", action="store_true",
                        help="snapshot current pose as URDF home and print offsets")
    args = parser.parse_args()

    if not args.ip:
        parser.error("no robot IP (set ROBOT_IP in .env or pass --ip)")

    client = HC1Client(args.ip, args.port, args.timeout)
    if args.capture_zero:
        capture_zero(client)
    else:
        monitor(client, args.rate, args.duration)


if __name__ == "__main__":
    main()
