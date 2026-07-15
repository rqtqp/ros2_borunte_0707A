"""HC1 smooth-motion lab: the E1-E4 protocol experiments (no rclpy, no MoveIt).

Implements the experiment ladder from the smooth-motion investigation (see
docs/HC1_SMOOTH_MOTION.md for the full runbook + vendor background):

    e1   smooth-level sweep: same small joint path at smooth = 0,1,3,6,9
    e2   append while idle:  emptyList=1 batch, then emptyList=0 batch at rest
    e3   append while MOVING (the streaming test): emptyList=0 mid-execution
    raw  send verbatim AddRCC instruction JSON (E4 Cartesian probes: action 10/17)

    ros2 run borunte0707a_driver smooth_lab e1                  # dry-run (default)
    ros2 run borunte0707a_driver smooth_lab e1 -- --live        # moves the arm!
    ros2 run borunte0707a_driver smooth_lab e3 -- --live --speed 5
    ros2 run borunte0707a_driver smooth_lab raw -- \
        --instruction '{"oneshot":"1","action":"10", ... }' --live

SAFETY: dry-run by default -- nothing is sent until --live, and --live asks for
confirmation (skip with --yes). Run with an operator at the e-stop and the
workcell clear. Speed is capped at 20% (default 10%). One HC1Client connection
is used for everything (tiny controller socket pool -- NEVER run this while the
motion bridge / RViz stack holds its connections). On Ctrl+C or any error in a
live run the tool sends actionStop before exiting. Every AddRCC reply is logged
verbatim, and each run writes a JSON log (--log-dir) for the findings doc.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from borunte0707a_driver import calibration
from borunte0707a_driver.calibration import NUM_JOINTS
from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import (
    HC1Client,
    build_free_path_instruction,
    motion_gate_status,
)

MAX_SPEED_PCT = 20.0
AXIS_ADDRESSES = [f"axis-{i}" for i in range(NUM_JOINTS)]


# --------------------------------------------------------------------------
# Pure helpers (offline-testable)
# --------------------------------------------------------------------------

def sweep_waypoints(start_deg, joint: int = 0, amplitude_deg: float = 8.0,
                    cycles: int = 2):
    """Back-and-forth sweep on one joint: +a, -a per cycle, ending back at the
    start pose (2*cycles + 1 points -- 5 for the default E1 path)."""
    points = []
    for _ in range(cycles):
        for direction in (+1.0, -1.0):
            p = list(start_deg)
            p[joint] += direction * amplitude_deg
            points.append(p)
    points.append(list(start_deg))
    return points


def ramp_waypoints(start_deg, joint: int = 0, amplitude_deg: float = 8.0,
                   steps: int = 4):
    """Monotonic ramp start -> start+amplitude in `steps` evenly spaced points
    (E3's slow path A; several waypoints so the move is long enough to append
    into)."""
    points = []
    for k in range(1, steps + 1):
        p = list(start_deg)
        p[joint] += amplitude_deg * k / steps
        points.append(p)
    return points


def assert_within_soft_limits(waypoints) -> None:
    for w in waypoints:
        ok, violations = calibration.within_soft_limits(w)
        if not ok:
            detail = ", ".join(
                f"J{i + 1}={v:.2f} not in [{lo},{hi}]" for i, v, lo, hi in violations
            )
            raise ValueError(f"waypoint outside soft limits: {detail}")


def moving_gaps(samples, min_dur_s: float = 0.15):
    """Runs of isMoving==0 strictly BETWEEN isMoving==1 periods, as
    (t_start, duration) -- the controller's own word for 'the arm stopped at a
    seam'. samples: [{"t": float, "isMoving": int, "axes": [...]}, ...]."""
    moving_idx = [i for i, s in enumerate(samples) if s["isMoving"] == 1]
    if len(moving_idx) < 2:
        return []
    gaps = []
    run_start = None
    for i in range(moving_idx[0], moving_idx[-1] + 1):
        if samples[i]["isMoving"] == 0:
            if run_start is None:
                run_start = i
        elif run_start is not None:
            duration = samples[i]["t"] - samples[run_start]["t"]
            if duration >= min_dur_s:
                gaps.append((samples[run_start]["t"], duration))
            run_start = None
    return gaps


def detect_pauses(samples, joint: int = 0, still_eps_deg: float = 0.05,
                  min_dur_s: float = 0.25):
    """Position-stillness runs strictly inside the motion window (the watched
    joint stopped changing while the path was still incomplete), as
    (t_start, duration). Complements moving_gaps: catches a decelerate-to-
    crawl seam even if isMoving never drops."""
    if len(samples) < 3:
        return []
    moved = [
        abs(samples[i + 1]["axes"][joint] - samples[i]["axes"][joint]) > still_eps_deg
        for i in range(len(samples) - 1)
    ]
    if not any(moved):
        return []
    first = moved.index(True)
    last = len(moved) - 1 - moved[::-1].index(True)
    pauses = []
    run_start = None
    for i in range(first, last + 1):
        if not moved[i]:
            if run_start is None:
                run_start = i
        elif run_start is not None:
            duration = samples[i]["t"] - samples[run_start]["t"]
            if duration >= min_dur_s:
                pauses.append((samples[run_start]["t"], duration))
            run_start = None
    return pauses


def max_joint_err(a, b) -> float:
    return max(abs(a[i] - b[i]) for i in range(NUM_JOINTS))


# --------------------------------------------------------------------------
# Lab runtime (talks to the controller; client injectable for tests)
# --------------------------------------------------------------------------

class SmoothLab:
    """One reused HC1 connection + event log around the experiments."""

    def __init__(self, client, live: bool = False, speed_pct: float = 10.0,
                 command_timeout: float = 8.0, poll_hz: float = 15.0):
        self.client = client
        self.live = live
        self.speed_pct = speed_pct
        self.command_timeout = command_timeout
        self.poll_period = 1.0 / poll_hz
        self.events: list[dict] = []
        # Injectable for offline tests.
        self.sleep = time.sleep
        self.clock = time.monotonic

    def note(self, message: str, **data) -> None:
        print(message)
        self.events.append({"t": self.clock(), "msg": message, **data})

    def read_axes(self):
        data = self.client.query(AXIS_ADDRESSES)
        return [float(data[a]) for a in AXIS_ADDRESSES]

    def read_motion(self):
        data = self.client.query(["isMoving", *AXIS_ADDRESSES])
        return (
            int(str(data.get("isMoving", "0")).strip() or 0),
            [float(data[a]) for a in AXIS_ADDRESSES],
        )

    def wait_gate(self, timeout_s: float = 30.0) -> None:
        """Block until the motion gate is satisfied (arm idle, no alarm)."""
        deadline = self.clock() + timeout_s
        while True:
            gate = motion_gate_status(self.client)
            if gate is True:
                return
            if self.clock() >= deadline:
                raise RuntimeError(f"motion gate not satisfied after {timeout_s:g}s: {gate}")
            self.note(f"  waiting for gate: {gate}")
            self.sleep(1.0)

    def send(self, instructions: list[dict], empty_list: str, label: str):
        """Send (or dry-run print) one AddRCC batch; log the verbatim reply.
        Returns the reply dict, or None in dry-run / on send failure."""
        summary = f"{label}: {len(instructions)} instr, emptyList={empty_list}"
        if not self.live:
            self.note(f"[DRY-RUN] would send AddRCC {summary}",
                      instructions=instructions, emptyList=empty_list)
            for k, instr in enumerate(instructions):
                print(f"  [{k}] {json.dumps(instr, separators=(',', ':'))}")
            return None
        self.note(f"sending AddRCC {summary}",
                  instructions=instructions, emptyList=empty_list)
        t0 = self.clock()
        try:
            reply = self.client.send_addrcc(
                "www.hc-system.com.HCRemoteCommand", instructions,
                empty_list=empty_list, pack_id=f"smooth-lab-{label.split()[0]}",
                timeout=self.command_timeout,
            )
        except (OSError, ValueError, RuntimeError) as error:
            # Never resent: a timed-out AddRCC may already have been received.
            self.note(f"  send FAILED after {(self.clock() - t0) * 1e3:.0f}ms: {error} "
                      f"(NOT resent)", error=str(error))
            return None
        latency_ms = (self.clock() - t0) * 1e3
        self.note(f"  reply ({latency_ms:.0f}ms): {json.dumps(reply, separators=(',', ':'))}",
                  reply=reply, latency_ms=latency_ms)
        return reply

    def watch(self, label: str, wait_start: bool = True,
              start_timeout_s: float = 8.0, stop_timeout_s: float = 180.0,
              stop_hold_s: float = 1.5):
        """Sample isMoving + axis-0..5 until the arm has been still for
        stop_hold_s (longer than any seam pause we want to detect). Returns the
        sample list. wait_start=False when the arm is already moving."""
        samples = []
        t0 = self.clock()
        started = not wait_start
        still_since = None
        while self.clock() - t0 < stop_timeout_s:
            is_moving, axes = self.read_motion()
            samples.append({"t": self.clock() - t0, "isMoving": is_moving, "axes": axes})
            if not started:
                if is_moving:
                    started = True
                elif self.clock() - t0 > start_timeout_s:
                    self.note(f"  {label}: motion never started within "
                              f"{start_timeout_s:g}s ({len(samples)} samples)")
                    return samples
            else:
                if is_moving:
                    still_since = None
                else:
                    still_since = still_since or self.clock()
                    if self.clock() - still_since >= stop_hold_s:
                        self.note(f"  {label}: motion complete "
                                  f"({samples[-1]['t']:.1f}s, {len(samples)} samples)")
                        return samples
            self.sleep(self.poll_period)
        self.note(f"  {label}: watch TIMED OUT after {stop_timeout_s:g}s")
        return samples

    def stop(self) -> None:
        """Best-effort actionStop (anomaly / Ctrl+C path)."""
        try:
            result = self.client.send_command("actionStop")
            self.note(f"actionStop sent: ok={result.get('ok')}", reply=result.get("reply"))
        except (OSError, ValueError, RuntimeError) as error:
            self.note(f"actionStop FAILED: {error} -- use the e-stop / pendant")


# --------------------------------------------------------------------------
# Experiments
# --------------------------------------------------------------------------

def run_e1(lab: SmoothLab, args) -> dict:
    """E1: identical small path at several smooth levels; compare duration,
    terminal error, and mid-path pauses."""
    joint = args.joint - 1
    start = lab.read_axes()
    points = sweep_waypoints(start, joint, args.amplitude_deg, cycles=2)
    assert_within_soft_limits(points)
    lab.note(f"E1: J{args.joint} +/-{args.amplitude_deg:g} deg sweep, "
             f"{len(points)} waypoints, levels {args.levels}, speed {lab.speed_pct:g}%")
    results = []
    for level in args.levels:
        instructions = [
            build_free_path_instruction(p, lab.speed_pct, smooth=str(level))
            for p in points
        ]
        if lab.live:
            lab.wait_gate()
        t_send = lab.clock()
        reply = lab.send(instructions, "1", f"E1 smooth={level}")
        record = {"smooth": level, "reply": reply}
        if lab.live and reply is not None:
            samples = lab.watch(f"E1 smooth={level}")
            final = lab.read_axes()
            record.update(
                duration_s=round(lab.clock() - t_send, 2),
                terminal_err_deg=round(max_joint_err(final, points[-1]), 3),
                gaps=moving_gaps(samples),
                pauses=detect_pauses(samples, joint),
                samples=samples,
            )
            lab.note(f"  smooth={level}: {record['duration_s']}s, terminal err "
                     f"{record['terminal_err_deg']} deg, isMoving gaps "
                     f"{record['gaps']}, stillness pauses {record['pauses']}")
        results.append(record)
    if lab.live:
        print("\nE1 summary (pick the default smooth level from this):")
        print("  smooth  duration_s  terminal_err_deg  gaps  pauses")
        for r in results:
            print(f"  {r['smooth']:>6}  {r.get('duration_s', '-'):>10}  "
                  f"{r.get('terminal_err_deg', '-'):>16}  "
                  f"{len(r.get('gaps', [])):>4}  {len(r.get('pauses', [])):>6}")
    return {"experiment": "e1", "waypoints": points, "results": results}


def run_e2(lab: SmoothLab, args) -> dict:
    """E2: list bookkeeping while IDLE. A (emptyList=1) -> complete; B
    (emptyList=0) at rest: does the appended batch auto-execute, once, in
    order? Then A again with emptyList=1 to prove clearing leaves no leftovers."""
    joint = args.joint - 1
    start = lab.read_axes()
    away = list(start)
    away[joint] += args.amplitude_deg
    assert_within_soft_limits([start, away])
    smooth = str(args.smooth)
    a_point = build_free_path_instruction(away, lab.speed_pct, smooth=smooth)
    b_point = build_free_path_instruction(start, lab.speed_pct, smooth=smooth)
    steps = []

    def step(name, instructions, empty_list, expect):
        if lab.live:
            lab.wait_gate()
        reply = lab.send(instructions, empty_list, name)
        record = {"step": name, "expect": expect, "reply": reply}
        if lab.live and reply is not None:
            samples = lab.watch(name, start_timeout_s=10.0)
            record["samples"] = samples
            record["executed"] = any(s["isMoving"] for s in samples)
            record["final_axes"] = lab.read_axes()
            lab.note(f"  {name}: executed={record['executed']}, final "
                     f"J{args.joint}={record['final_axes'][joint]:.2f}")
        steps.append(record)
        return record

    lab.note(f"E2: append-while-idle bookkeeping, J{args.joint} "
             f"+{args.amplitude_deg:g} deg, speed {lab.speed_pct:g}%")
    step("E2-A (clear+add)", [a_point], "1", "moves to start+amp")
    step("E2-B (append at rest)", [b_point], "0",
         "UNKNOWN: does an appended batch auto-execute? once? in order?")
    step("E2-C (clear+add again)", [a_point], "1",
         "exactly ONE burst to start+amp -- leftovers would repeat B first")
    step("E2-home (clear+add)", [b_point], "1", "returns to start")
    return {"experiment": "e2", "start": start, "away": away, "steps": steps}


def run_e3(lab: SmoothLab, args) -> dict:
    """E3: THE streaming test. Slow multi-point path A (emptyList=1); while
    isMoving==1 append continuation B (emptyList=0) whose first point equals
    A's last. Accepted or rejected? Seam pause or blend-through?"""
    joint = args.joint - 1
    speed = min(lab.speed_pct, 5.0)  # slow so there is time to append into A
    start = lab.read_axes()
    a_points = ramp_waypoints(start, joint, args.amplitude_deg, steps=4)
    b_points = [list(a_points[-1]), list(start)]  # continues exactly from A's end
    assert_within_soft_limits(a_points + b_points)
    smooth = str(args.smooth)
    a_instr = [build_free_path_instruction(p, speed, smooth=smooth) for p in a_points]
    b_instr = [build_free_path_instruction(p, speed, smooth=smooth) for p in b_points]
    lab.note(f"E3: append-while-moving, J{args.joint} ramp +{args.amplitude_deg:g} deg "
             f"({len(a_points)} pts) then appended return ({len(b_points)} pts), "
             f"speed {speed:g}%, smooth={smooth} (use >0: smooth=0 stops at every "
             f"waypoint and masks the seam)")

    if lab.live:
        lab.wait_gate()
    reply_a = lab.send(a_instr, "1", "E3-A (long slow path)")
    result = {"experiment": "e3", "a_points": a_points, "b_points": b_points,
              "reply_a": reply_a}
    if not lab.live or reply_a is None:
        lab.send(b_instr, "0", "E3-B (append while moving)")
        return result

    # Wait for A to actually start, then append B mid-execution.
    deadline = lab.clock() + 8.0
    while lab.clock() < deadline:
        is_moving, _ = lab.read_motion()
        if is_moving:
            break
        lab.sleep(lab.poll_period)
    else:
        lab.note("E3 ABORTED: path A never started moving; nothing appended")
        return result

    reply_b = lab.send(b_instr, "0", "E3-B (append while moving)")
    result["reply_b"] = reply_b
    # stop_hold 4s: a seam pause shorter than this must read as a PAUSE inside
    # one watch, not as end-of-motion (which would misclassify B as ignored).
    samples = lab.watch("E3", wait_start=False, stop_hold_s=4.0)
    final = lab.read_axes()
    gaps = moving_gaps(samples)
    pauses = detect_pauses(samples, joint)
    b_ran = max_joint_err(final, start) < 1.0  # B's endpoint is the start pose
    result.update(samples=samples, gaps=gaps, pauses=pauses,
                  final_axes=final, b_executed=b_ran)
    if reply_b is None:
        verdict = "append send failed/timed out -- see events; streaming UNPROVEN"
    elif not b_ran:
        verdict = ("append ACCEPTED by reply but B did not execute (arm stayed at "
                   "A's end) -- appends while moving are likely ignored/rejected")
    elif gaps or pauses:
        verdict = (f"append EXECUTED but the arm paused at the A->B seam "
                   f"(gaps={gaps}, pauses={pauses}) -- streaming works, blending does not")
    else:
        verdict = ("append EXECUTED with NO seam pause -- STREAMING IS REAL; "
                   "E5 (bridge stream_path mode) is unlocked")
    result["verdict"] = verdict
    lab.note(f"E3 verdict: {verdict}")
    return result


def run_raw(lab: SmoothLab, args) -> dict:
    """E4 helper: send operator-supplied instruction JSON verbatim (e.g.
    action 10 Posture Line / action 17 Posture Curve probes). No soft-limit
    validation -- Cartesian fields are exactly what we are trying to learn, so
    keep the pendant next to you and the amplitude tiny."""
    instructions = [json.loads(raw) for raw in args.instruction]
    for k, instr in enumerate(instructions):
        if not isinstance(instr, dict):
            raise ValueError(f"--instruction {k} is not a JSON object")
    lab.note(f"raw: {len(instructions)} operator-supplied instruction(s), "
             f"emptyList={args.empty_list} (NO soft-limit validation)")
    if lab.live:
        lab.wait_gate()
    reply = lab.send(instructions, args.empty_list, "raw")
    result = {"experiment": "raw", "instructions": instructions, "reply": reply}
    if lab.live and reply is not None:
        samples = lab.watch("raw", start_timeout_s=10.0)
        result["samples"] = samples
        result["final_axes"] = lab.read_axes()
    return result


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _speed(value: str) -> float:
    speed = float(value)
    if not 1.0 <= speed <= MAX_SPEED_PCT:
        raise argparse.ArgumentTypeError(
            f"speed must be within 1..{MAX_SPEED_PCT:g} (experiment cap)")
    return speed


def _levels(value: str):
    levels = [int(x) for x in value.split(",") if x.strip()]
    if not levels or any(not 0 <= lv <= 9 for lv in levels):
        raise argparse.ArgumentTypeError("levels must be integers 0..9, comma-separated")
    return levels


def build_parser(env) -> argparse.ArgumentParser:
    # Common options live on a PARENT parser attached to every subcommand, so
    # they can be given after it: `smooth_lab e1 --live` (the natural form
    # under `ros2 run ... smooth_lab e1 -- --live`).
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--host", default=env.get("ROBOT_IP", ""),
                        help="controller address (default: ROBOT_IP from .env)")
    common.add_argument("--port", type=int,
                        default=int(env.get("REMOTE_MONITOR_PORT", "9760")))
    common.add_argument("--timeout", type=float,
                        default=float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
    common.add_argument("--live", action="store_true",
                        help="actually send to the controller (default: dry-run)")
    common.add_argument("--yes", action="store_true",
                        help="skip the interactive --live confirmation")
    common.add_argument("--speed", type=_speed, default=10.0,
                        help=f"AddRCC speed %% (1..{MAX_SPEED_PCT:g}; default 10)")
    common.add_argument("--joint", type=int, choices=range(1, NUM_JOINTS + 1),
                        default=1, metavar="1..6",
                        help="joint to exercise (default 1 = base, safest)")
    common.add_argument("--amplitude-deg", type=float, default=8.0,
                        help="motion amplitude in controller degrees (default 8)")
    common.add_argument("--smooth", type=int, choices=range(0, 10), default=0,
                        metavar="0..9",
                        help="smooth level for e2/e3 instructions (default 0; "
                             "E1 sweeps its own levels). E3 with smooth=0 stops "
                             "at every waypoint, masking the seam behavior")
    common.add_argument("--log-dir", default=".",
                        help="directory for the JSON run log (default: cwd)")

    parser = argparse.ArgumentParser(
        prog="smooth_lab", description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="experiment", required=True)
    e1 = sub.add_parser("e1", parents=[common],
                        help="smooth-level sweep (lowest risk)")
    e1.add_argument("--levels", type=_levels, default=[0, 1, 3, 6, 9],
                    help="comma-separated smooth levels to sweep (default 0,1,3,6,9)")
    sub.add_parser("e2", parents=[common],
                   help="append while idle (emptyList bookkeeping)")
    sub.add_parser("e3", parents=[common],
                   help="append while moving (the streaming test)")
    raw = sub.add_parser("raw", parents=[common],
                         help="send verbatim instruction JSON (E4 probes)")
    raw.add_argument("--instruction", action="append", required=True,
                     help="one AddRCC instruction as a JSON object (repeatable)")
    raw.add_argument("--empty-list", choices=["0", "1"], default="1",
                     help="emptyList field: 1=clear list first (default), 0=append")
    return parser


RUNNERS = {"e1": run_e1, "e2": run_e2, "e3": run_e3, "raw": run_raw}


def main(argv=None) -> int:
    env = load_env()
    args = build_parser(env).parse_args(argv)
    if not args.host:
        print("no host: set ROBOT_IP in .env / environment or pass --host")
        return 2
    if args.live and not args.yes:
        print("LIVE MODE: the arm WILL move. Operator at the e-stop? Workcell "
              "clear? Motion bridge / RViz stack STOPPED (socket pool)?")
        if input("type GO to continue: ").strip() != "GO":
            print("aborted.")
            return 2

    mode = "LIVE" if args.live else "DRY-RUN"
    print(f"smooth_lab {args.experiment} [{mode}] -> {args.host}:{args.port} "
          f"speed={args.speed:g}%")
    exit_code = 0
    result: dict = {}
    with HC1Client(args.host, args.port, args.timeout) as client:
        lab = SmoothLab(client, live=args.live, speed_pct=args.speed)
        try:
            result = RUNNERS[args.experiment](lab, args)
        except KeyboardInterrupt:
            lab.note("INTERRUPTED (Ctrl+C)")
            if args.live:
                lab.stop()
            exit_code = 130
        except Exception as error:  # anomaly: halt first, then report
            lab.note(f"ERROR: {error}")
            if args.live:
                lab.stop()
            exit_code = 1
        log_path = (Path(args.log_dir) /
                    f"smooth_lab_{args.experiment}_"
                    f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        log_path.write_text(json.dumps({
            "experiment": args.experiment, "mode": mode, "speed": args.speed,
            "joint": args.joint, "amplitude_deg": args.amplitude_deg,
            "events": lab.events, "result": result,
        }, indent=2, default=str), encoding="utf-8")
        print(f"run log written: {log_path}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
