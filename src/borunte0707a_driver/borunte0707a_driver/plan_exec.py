"""MoveIt Plan+Execute to a joint-space goal — the validated full pipeline
(collision-aware OMPL plan -> trajectory stream -> motion_bridge AddRCC path).

    ros2 run borunte0707a_driver plan_exec -- j1 j2 j3 j4 j5 j6   (URDF deg)
    ros2 run borunte0707a_driver plan_exec -- home                (all zeros)
    ...append: --gate-topic <topic>   (default arm_motion_ok)

Run with the controller ACTIVE and an operator at the e-stop. Gated on a
Bool health topic (fail-closed: refuses to move if the gate is absent, stale,
or False). After MoveIt reports success it additionally BLOCKS until
/joint_states actually settles on the goal (MoveIt's "done" can lead the arm
by one AddRCC) — chained calls never overlap motions.

Goal-constraint tolerance is deliberately tight (0.001 rad): OMPL samples the
goal state anywhere inside it, so a loose value becomes terminal position
error (~0.5 deg at 0.01 rad, measured live).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

NAMES = [f"brtirus0707a_joint_{i}" for i in range(1, 7)]
GROUP = "arm"
VEL_SCALE = 0.25          # trajectory timing only; arm speed is the bridge's speed_pct
GOAL_TOL_RAD = 0.001
ARRIVE_TOL_DEG = 0.7
ARRIVE_TIMEOUT = 60.0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("target", nargs="+",
                        help="six URDF joint targets in degrees, or 'home'")
    parser.add_argument("--gate-topic", default="arm_motion_ok",
                        help="Bool health gate topic (fail-closed)")
    args = parser.parse_args(argv)
    if args.target == ["home"]:
        tgt_deg = [0.0] * 6
    elif len(args.target) == 6:
        tgt_deg = [float(a) for a in args.target]
    else:
        parser.error("expected 6 joint targets in degrees, or 'home'")
    tgt = [math.radians(d) for d in tgt_deg]

    rclpy.init()
    n = rclpy.create_node("plan_exec")
    cur = {}
    n.create_subscription(JointState, "joint_states",
                          lambda m: cur.update(zip(m.name, m.position)), 10)
    gate = {"ok": None, "t": 0.0}
    n.create_subscription(Bool, args.gate_topic,
                          lambda m: gate.update(ok=bool(m.data), t=time.time()), 10)
    t0 = time.time()
    while (len(cur) < 6 or gate["ok"] is None) and time.time() - t0 < 6:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(cur) < 6:
        print("no /joint_states — is the arm stack up?")
        return 1
    if gate["ok"] is not True or (time.time() - gate["t"]) > 2.0:
        print(f"BLOCKED {args.gate_topic}={gate['ok']} — link/feedback not healthy")
        return 1

    print("plan+exec -> [%s] deg  (from [%s])"
          % (", ".join(f"{d:.1f}" for d in tgt_deg),
             ", ".join(f"{math.degrees(cur[nm]):.1f}" for nm in NAMES)))

    ac = ActionClient(n, MoveGroup, "/move_action")
    if not ac.wait_for_server(timeout_sec=8):
        print("no /move_action — is move_group up?")
        return 1
    goal = MoveGroup.Goal()
    goal.request.group_name = GROUP
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = VEL_SCALE
    goal.request.max_acceleration_scaling_factor = VEL_SCALE
    goal.request.start_state.is_diff = True
    c = Constraints()
    for nm, p in zip(NAMES, tgt):
        c.joint_constraints.append(JointConstraint(
            joint_name=nm, position=p,
            tolerance_above=GOAL_TOL_RAD, tolerance_below=GOAL_TOL_RAD, weight=1.0))
    goal.request.goal_constraints = [c]
    goal.planning_options.plan_only = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    t_plan = time.time()
    gf = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, gf, timeout_sec=15)
    gh = gf.result()
    if gh is None or not gh.accepted:
        print("goal REJECTED by move_group")
        return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=120)
    res = rf.result()
    if res is None:
        print("no result (timeout) — check the bridge/controller")
        return 1
    code = res.result.error_code.val
    print("move_group result: %s (%.1fs)"
          % ("SUCCESS" if code == 1 else f"error {code}", time.time() - t_plan))
    if code != 1:
        return 1

    # Real completion: wait until the ARM (not the command stream) is on the goal.
    t0 = time.time()
    stable = 0
    arrived = False
    while time.time() - t0 < ARRIVE_TIMEOUT:
        rclpy.spin_once(n, timeout_sec=0.2)
        errs = [abs(cur.get(nm, 9.0) - tgt[i]) for i, nm in enumerate(NAMES)]
        if max(errs) < math.radians(ARRIVE_TOL_DEG):
            stable += 1
            if stable >= 5:
                arrived = True
                break
        else:
            stable = 0
    final = [math.degrees(cur.get(nm, float("nan"))) for nm in NAMES]
    print("%s: joints=[%s] deg  (max err %.2f deg)"
          % ("ARRIVED" if arrived else "TIMEOUT waiting for arrival",
             ", ".join(f"{d:.2f}" for d in final),
             max(abs(final[i] - tgt_deg[i]) for i in range(6))))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if arrived else 1


if __name__ == "__main__":
    sys.exit(main())
