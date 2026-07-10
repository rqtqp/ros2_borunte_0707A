"""Joint-space repeatability test: cycle the arm between two poses N times per
speed and measure the settled encoder positions at each arrival.

    ros2 run borunte0707a_driver repeatability_test -- \
        [--cycles N] [--speeds 10,20,30] [--duration 4.0] [--gate-topic T]

Run with the controller ACTIVE and an operator at the e-stop. Drives the
joint_trajectory_controller directly (single-point trajectories, arrival-
blocked, never overlapping), gated on a Bool health topic (fail-closed).
Sets the bridge's speed_pct per phase via the parameter service (needs the
runtime-settable speed_pct). Positions come from /joint_states = the
controller's own encoders (axis-N), so this measures encoder-level
repeatability, not external/metrology accuracy.

Prints per speed x endpoint: per-joint mean error vs commanded, spread
(max-min) and std across cycles, plus per-move travel time. Ends at home.

Measured 2026-07-10 (5 cycles @ 10/20/30%): worst-joint spread 0.000 deg at
every speed and endpoint; travel 25.3/16.4/13.5 s.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

NAMES = [f"brtirus0707a_joint_{i}" for i in range(1, 7)]
BRIDGE = "/borunte0707a_motion_bridge"

# Two multi-joint poses (URDF degrees), comfortably inside the soft limits.
POSE_A = [20.0, -8.0, 15.0, 5.0, -12.0, 10.0]
POSE_B = [-20.0, -3.0, 25.0, -5.0, -18.0, -10.0]
HOME = [0.0] * 6

ARRIVE_TOL_DEG = 0.8      # coarse arrival detector; settled sampling is finer
STABLE_EPS_DEG = 0.02     # "settled" = position changing less than this
SETTLE_SAMPLES = 15       # encoder samples averaged per measurement


class Rig:
    def __init__(self, node, gate_topic):
        self.n = node
        self.cur = {}
        node.create_subscription(JointState, "joint_states",
                                 lambda m: self.cur.update(zip(m.name, m.position)), 10)
        self.gate = {"ok": None, "t": 0.0}
        node.create_subscription(Bool, gate_topic,
                                 lambda m: self.gate.update(ok=bool(m.data), t=time.time()), 10)
        self.pub = node.create_publisher(
            JointTrajectory, "/brtirus0707a_controller/joint_trajectory", 10)
        self.param_cli = node.create_client(SetParameters, BRIDGE + "/set_parameters")

    def spin(self, sec):
        rclpy.spin_once(self.n, timeout_sec=sec)

    def positions_deg(self):
        return [math.degrees(self.cur[nm]) for nm in NAMES]

    def gate_ok(self):
        return self.gate["ok"] is True and (time.time() - self.gate["t"]) < 2.0

    def set_speed(self, pct):
        req = SetParameters.Request()
        req.parameters = [Parameter(name="speed_pct", value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(pct)))]
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=5)
        res = fut.result()
        if res is None or not res.results[0].successful:
            raise RuntimeError("failed to set speed_pct=%s: %s"
                               % (pct, None if res is None else res.results[0].reason))

    def move_and_measure(self, tgt_deg, duration):
        """Command one pose; block until settled; return (mean_deg[6], seconds)."""
        if not self.gate_ok():
            raise RuntimeError(f"BLOCKED: health gate = {self.gate['ok']}")
        jt = JointTrajectory()
        jt.joint_names = NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in tgt_deg]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        jt.points = [pt]
        t0 = time.time()
        self.pub.publish(jt)
        # 1) coarse arrival, 2) stillness, 3) averaged settled sample
        last = None
        still_since = None
        while time.time() - t0 < duration + 90:
            self.spin(0.1)
            pos = self.positions_deg()
            near = max(abs(pos[i] - tgt_deg[i]) for i in range(6)) < ARRIVE_TOL_DEG
            moving = last is not None and max(
                abs(pos[i] - last[i]) for i in range(6)) > STABLE_EPS_DEG
            last = pos
            if near and not moving:
                still_since = still_since or time.time()
                if time.time() - still_since > 1.5:
                    break
            else:
                still_since = None
        else:
            raise RuntimeError(f"move timed out (target {tgt_deg})")
        travel = time.time() - t0
        samples = []
        for _ in range(SETTLE_SAMPLES):
            self.spin(0.08)
            samples.append(self.positions_deg())
        mean = [statistics.fmean(s[i] for s in samples) for i in range(6)]
        return mean, travel


def report(tag, commanded, arrivals):
    print("  endpoint %s  (commanded %s), %d arrivals:"
          % (tag, [f"{c:.1f}" for c in commanded], len(arrivals)))
    for i in range(6):
        vals = [a[i] for a in arrivals]
        print("    J%d: mean err %+7.3f deg   spread %6.3f   std %6.3f"
              % (i + 1, statistics.fmean(vals) - commanded[i],
                 max(vals) - min(vals), statistics.pstdev(vals)))
    worst = max(max(a[i] for a in arrivals) - min(a[i] for a in arrivals) for i in range(6))
    print(f"    -> worst-joint spread: {worst:.3f} deg")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--speeds", default="10,20,30")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--gate-topic", default="arm_motion_ok")
    args = parser.parse_args(argv)
    speeds = [float(s) for s in args.speeds.split(",")]

    rclpy.init()
    node = rclpy.create_node("repeatability_test")
    rig = Rig(node, args.gate_topic)
    t0 = time.time()
    while (len(rig.cur) < 6 or rig.gate["ok"] is None) and time.time() - t0 < 8:
        rig.spin(0.1)
    if len(rig.cur) < 6:
        print("no /joint_states — arm stack down?")
        return 1
    if not rig.param_cli.wait_for_service(timeout_sec=5):
        print(f"no {BRIDGE}/set_parameters — old bridge?")
        return 1

    print("repeatability: %d cycles x %s%% speed, poses A=%s B=%s"
          % (args.cycles, speeds, POSE_A, POSE_B))
    for pct in speeds:
        rig.set_speed(pct)
        print(f"== speed {pct:g}% ==")
        rig.move_and_measure(POSE_A, args.duration)   # entry move, not measured
        at_a, at_b, times = [], [], []
        for k in range(args.cycles):
            m, t = rig.move_and_measure(POSE_B, args.duration)
            at_b.append(m)
            times.append(t)
            m, t = rig.move_and_measure(POSE_A, args.duration)
            at_a.append(m)
            times.append(t)
            print("  cycle %d/%d done (last move %.1fs)" % (k + 1, args.cycles, t))
        report("A", POSE_A, at_a)
        report("B", POSE_B, at_b)
        print("  travel time: mean %.1fs  min %.1fs  max %.1fs"
              % (statistics.fmean(times), min(times), max(times)))
    rig.move_and_measure(HOME, args.duration)
    print("parked home. done.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
