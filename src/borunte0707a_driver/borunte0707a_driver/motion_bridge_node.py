"""Phase 3: joint-command -> AddRCC motion bridge.

Subscribes a target `sensor_msgs/JointState` (URDF convention, radians),
converts to controller axis-N degrees via the shared calibration, validates
against the safety gate + soft limits, and emits an HCRemoteCommand `AddRCC`
free-path point (`action:"4"`).

SAFE BY DEFAULT: `dry_run` is True until explicitly disabled, so the node logs
the exact AddRCC packet it *would* send without moving the arm. Flip
`dry_run:=false` only with an operator at the e-stop.

Safety enforced before every send (re-queried live, never cached):
  * controller motion gate: curMode==7, curAlarm==0, isMoving==0, origin==1
  * all 6 targets within controller soft limits (calibration.SOFT_LIMITS_DEG)
  * optional max step clamp vs. the current pose (rejects large jumps)

Commands are coalesced and rate-limited (`max_rate_hz`): a streaming source like
MoveIt's TopicBasedSystem publishes at the controller-manager rate, so the bridge
keeps only the latest target and emits at most one AddRCC per timer tick.

COMPLETION FEEDBACK (`completion_feedback`, default on): after each live send
the bridge polls the gate until the arm stops, then compares the actual pose
against the held streamed setpoint (the trajectory's true endpoint) and sends
one exact-goal correction if it is more than `correction_tol_deg` off. This
closes the ~0.3-0.6 deg terminal error from the settle heuristic sampling the
stream early, and logs a definitive "goal reached: max err X deg".
"""

from __future__ import annotations

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from borunte0707a_driver import calibration
from borunte0707a_driver.calibration import NUM_JOINTS
from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client

# Controller motion preconditions (reference/HC1_DEBUG_REFERENCE.md). Mode 7 is
# the only mode that executes AddRCC on F5.2.1.
REQUIRED_MODE = "7"
GATE_ADDRESSES = ["curMode", "curAlarm", "isMoving", "origin"]


def _is(value, expected: str) -> bool:
    return value is not None and str(value).strip() == expected


def build_free_path_instruction(axis_deg, speed_pct: float,
                                smooth: str = "0", oneshot: str = "1") -> dict:
    """One AddRCC free-path (joint) point. m6/m7 must be present as '0.0'."""
    instr = {
        "oneshot": oneshot,
        "action": "4",
        "ckStatus": "0x3F",
        "speed": f"{speed_pct:.1f}",
        "delay": "0.0",
        "tool": "0",
        "coord": "0",
        "smooth": smooth,
    }
    for i in range(NUM_JOINTS):
        instr[f"m{i}"] = f"{axis_deg[i]:.4f}"
    instr["m6"] = "0.0"
    instr["m7"] = "0.0"
    return instr


class MotionBridge(Node):
    def __init__(self):
        super().__init__("borunte0707a_motion_bridge")
        env = load_env()

        self.declare_parameter("robot_ip", env.get("ROBOT_IP", ""))
        self.declare_parameter("remote_monitor_port", int(env.get("REMOTE_MONITOR_PORT", "9760")))
        self.declare_parameter("timeout", float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
        self.declare_parameter(
            "command_service_id",
            env.get("HC1_REMOTE_COMMAND_SERVICE_ID", "www.hc-system.com.HCRemoteCommand"),
        )
        self.declare_parameter("command_topic", "joint_command")
        self.declare_parameter(
            "joint_names", [f"brtirus0707a_joint_{i + 1}" for i in range(NUM_JOINTS)]
        )
        # SAFETY: never moves the arm until this is explicitly set false.
        self.declare_parameter("dry_run", True)
        self.declare_parameter("speed_pct", 10.0)
        # Reject any command whose per-joint delta from the current pose exceeds
        # this (controller degrees). 0 disables the check. Keep small for bring-up.
        self.declare_parameter("max_step_deg", 15.0)
        # Rate limit: TopicBasedSystem streams commands at the controller-manager
        # rate (100s of Hz) -- far above a safe AddRCC/s. Coalesce to the latest
        # target and send at most this often. 0 = send on every message (legacy).
        self.declare_parameter("max_rate_hz", 5.0)
        # GOAL MODE (point-to-point): AddRCC is a point-to-point move that sets
        # isMoving=1, which blocks the next streamed setpoint -- so chasing a
        # time-based MoveIt trajectory stalls. Instead, wait for the streamed
        # target to stop changing (trajectory finished), then send the FINAL goal
        # as a single AddRCC and let the controller run the whole move. This is
        # the natural fit for an AddRCC arm. Seconds of stillness that mark a
        # settled goal; 0 = legacy streaming (send every coalesced setpoint).
        self.declare_parameter("goal_settle_sec", 0.5)
        # PATH FOLLOWING: in plain goal mode only the endpoint is sent, so the arm
        # cuts straight to the goal and ignores MoveIt's collision-free *path*.
        # With send_path=true the bridge accumulates the streamed trajectory's
        # waypoints (one per >= path_waypoint_deg of joint motion) and, on settle,
        # sends them as a single multi-point AddRCC so the arm follows the planned
        # path. path_smooth (0-9) blends corners: low keeps the arm close to the
        # waypoints (safer near obstacles), high is smoother but cuts corners.
        # Default on (validated live); set false for endpoint-only point-to-point.
        self.declare_parameter("send_path", True)
        self.declare_parameter("path_waypoint_deg", 5.0)
        self.declare_parameter("path_smooth", 1)
        # The controller reliably accepts only SHORT AddRCC instruction lists.
        # Empirically (live test) 1-7 points reply in ~20 ms; 10-16 points never
        # reply (8 s timeout); >=20 points are rejected instantly (connection
        # reset). So cap every AddRCC to this many waypoints. Raising this risks
        # unsent paths.
        self.declare_parameter("path_max_points", 8)
        # CHUNKING: with chunk_path=false a long path is downsampled to fit one
        # AddRCC (<=path_max_points), cutting corners. With chunk_path=true the
        # FULL path is sent as a sequence of <=path_max_points AddRCC segments
        # (overlapping by one waypoint), each fired when the arm finishes the
        # previous one -- faithful path following, at the cost of a brief stop at
        # each chunk boundary.
        self.declare_parameter("chunk_path", False)
        # AddRCC reply timeout. Longer than the query timeout: the controller can
        # be slow to acknowledge a motion command while finishing a prior move.
        self.declare_parameter("command_timeout", 8.0)
        # COMPLETION FEEDBACK: after each live send, poll the gate until the arm
        # stops (isMoving==0), then compare its actual position against the held
        # streamed setpoint -- which by then IS the trajectory's true endpoint --
        # and, if off by more than correction_tol_deg, send ONE exact-goal
        # correction point. Fixes the settle heuristic sampling the stream
        # slightly before the trajectory end (measured ~0.3-0.6 deg short on the
        # live arm) and gives a logged "goal reached" with the real error.
        self.declare_parameter("completion_feedback", True)
        self.declare_parameter("correction_tol_deg", 0.1)
        self.declare_parameter("completion_timeout", 90.0)
        # Control command the /stop service sends. "actionStop" (default) is a
        # decisive halt: confirmed to drop the controller from curMode 7
        # (auto-running) to 2 (auto-idle) -- the pendant remote program must be
        # re-armed (Start/Cycle -> curMode 7) before motion resumes. "actionPause"
        # may instead pause without leaving curMode 7 (softer cancel; untested).
        self.declare_parameter("stop_command", "actionStop")
        # Calibration overrides (controller<->URDF). Defaults from calibration.py.
        self.declare_parameter("sign", list(calibration.SIGN))
        self.declare_parameter("offset_rad", list(calibration.OFFSET_RAD))

        robot_ip = self.get_parameter("robot_ip").value
        if not robot_ip:
            raise RuntimeError(
                "robot_ip is not set. Provide it via .env (ROBOT_IP) or "
                "--ros-args -p robot_ip:=<addr>"
            )

        self.client = HC1Client(
            host=robot_ip,
            port=int(self.get_parameter("remote_monitor_port").value),
            timeout=float(self.get_parameter("timeout").value),
        )
        self.service_id = self.get_parameter("command_service_id").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.speed_pct = float(self.get_parameter("speed_pct").value)
        self.max_step_deg = float(self.get_parameter("max_step_deg").value)
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.goal_settle_sec = float(self.get_parameter("goal_settle_sec").value)
        self.send_path = bool(self.get_parameter("send_path").value)
        self.path_waypoint_deg = float(self.get_parameter("path_waypoint_deg").value)
        self.path_smooth = int(self.get_parameter("path_smooth").value)
        self.path_max_points = max(2, int(self.get_parameter("path_max_points").value))
        self.chunk_path = bool(self.get_parameter("chunk_path").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.completion_feedback = bool(self.get_parameter("completion_feedback").value)
        self.correction_tol_deg = float(self.get_parameter("correction_tol_deg").value)
        self.completion_timeout = float(self.get_parameter("completion_timeout").value)
        self.stop_command = str(self.get_parameter("stop_command").value)
        self.sign = tuple(self.get_parameter("sign").value)
        self.offset_rad = tuple(self.get_parameter("offset_rad").value)

        # Rate-limit state: latest target awaiting send, and last accepted target.
        self._pending_axis_deg = None
        self._last_sent_deg = None
        # Goal-mode state: when the pending target last changed (to detect that a
        # streamed trajectory has settled on its final goal).
        self._pending_since = self.get_clock().now()
        self._settle_eps_deg = 0.05
        # Path-mode state: waypoints accumulated for the current trajectory.
        self._path_waypoints: list[list[float]] = []
        self._max_waypoints = 100
        # Instrumentation: consecutive isMoving gate rejections immediately before
        # a send (a proxy for "the controller was just busy"), so logged AddRCC
        # reply latency can be correlated with busy-state vs point count.
        self._gate_wait_count = 0
        # Halt state: set by the /stop service to suppress sends until a new goal.
        self._halted = False
        self._halt_goal = None
        # Chunking state: queue of AddRCC segments still to send for the current
        # path, and the goal they lead to (for dedupe).
        self._chunk_queue: list[list[list[float]]] = []
        self._chunk_goal = None
        self._chunk_total = 0
        # Completion state: goal of the last live send being waited on
        # (None = idle). "corrected" marks that the one-shot exact-goal
        # correction was already sent for this goal.
        self._completing: dict | None = None

        self.sub = self.create_subscription(
            JointState, self.get_parameter("command_topic").value, self.on_command, 10
        )
        if self.max_rate_hz > 0:
            self.timer = self.create_timer(1.0 / self.max_rate_hz, self._on_timer)

        # Abort service. Its own connection + callback group so it can send
        # actionStop on a separate thread/socket even while the main loop is
        # mid-send -- the ROS-level complement to the physical e-stop.
        self.stop_client = HC1Client(
            host=robot_ip,
            port=int(self.get_parameter("remote_monitor_port").value),
            timeout=float(self.get_parameter("timeout").value),
        )
        self.stop_srv = self.create_service(
            Trigger, "stop", self.on_stop,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        mode = "DRY-RUN (no motion)" if self.dry_run else "LIVE MOTION"
        rate = f"{self.max_rate_hz:g} Hz" if self.max_rate_hz > 0 else "per-message"
        if self.goal_settle_sec <= 0:
            send_mode = "STREAM (per-setpoint)"
        elif self.send_path:
            send_mode = (
                f"PATH ({'chunked' if self.chunk_path else 'downsampled'}, settle "
                f"{self.goal_settle_sec:g}s, @{self.path_waypoint_deg:g}deg, "
                f"max {self.path_max_points} pts/AddRCC, smooth={self.path_smooth})"
            )
        else:
            send_mode = f"GOAL (settle {self.goal_settle_sec:g}s, point-to-point)"
        self.get_logger().info(
            f"Motion bridge up [{mode}] {send_mode} on "
            f"'{self.get_parameter('command_topic').value}' "
            f"-> {robot_ip}:{self.client.port}, speed={self.speed_pct:g}%, send rate {rate}"
        )
        if not self.dry_run:
            self.get_logger().warn(
                "dry_run=false: this node WILL move the arm. Keep an operator at "
                "the e-stop and ensure the workcell is clear."
            )

    def on_command(self, msg: JointState) -> None:
        # Reorder incoming positions to our canonical joint order and convert to
        # controller degrees. Cheap work only -- the gate/send happens in
        # _process_target, rate-limited by the timer (or inline if disabled).
        try:
            q_rad = self._extract_targets(msg)
        except ValueError as error:
            self.get_logger().warn(f"rejecting command: {error}", throttle_duration_sec=2.0)
            return

        axis_deg = calibration.urdf_rad_to_controller_deg(q_rad, self.sign, self.offset_rad)
        # After a /stop, ignore the held setpoint; resume only on a NEW goal.
        if self._halted:
            if self._halt_goal is None or self._max_joint_delta(axis_deg, self._halt_goal) > 1.0:
                self._halted = False
                self.get_logger().info("resuming: new goal after stop")
            else:
                return
        if self.max_rate_hz <= 0:
            self._process_target(axis_deg, enforce_step=self.max_step_deg > 0)
            return
        # Coalesce; the timer sends the latest. Record when the target last
        # *changed* so goal mode can detect a settled (finished) trajectory.
        now = self.get_clock().now()
        changed = self._pending_axis_deg is None or any(
            abs(axis_deg[i] - self._pending_axis_deg[i]) > self._settle_eps_deg
            for i in range(NUM_JOINTS)
        )
        if changed:
            # A change after a settled pause marks the start of a new trajectory:
            # reset the path accumulator so it captures only this move.
            gap = (now - self._pending_since).nanoseconds * 1e-9
            if self.send_path and gap > self.goal_settle_sec:
                self._path_waypoints = []
            self._pending_since = now
        if self.send_path and (
            not self._path_waypoints
            or self._max_joint_delta(axis_deg, self._path_waypoints[-1]) >= self.path_waypoint_deg
        ) and len(self._path_waypoints) < self._max_waypoints:
            self._path_waypoints.append(list(axis_deg))
        self._pending_axis_deg = axis_deg

    @staticmethod
    def _max_joint_delta(a, b) -> float:
        return max(abs(a[i] - b[i]) for i in range(NUM_JOINTS))

    def _on_timer(self) -> None:
        if self._halted:
            return
        # Drain queued path chunks first: send the next segment as soon as the arm
        # finishes the previous one. Finish the current path before any new goal.
        if self._chunk_queue:
            self._drain_chunks()
            return
        # Completion phase: wait for the arm to actually stop on the last sent
        # goal, then correct/confirm it. Suppresses new sends meanwhile unless
        # the stream has clearly moved on to a NEW trajectory.
        if self._completing is not None and not self._handle_completion():
            return
        if self._pending_axis_deg is None:
            return
        if self.goal_settle_sec > 0:
            # Only act once the streamed setpoint has held still long enough to be
            # the trajectory's final goal.
            elapsed = (self.get_clock().now() - self._pending_since).nanoseconds * 1e-9
            if elapsed < self.goal_settle_sec:
                return
            if self.send_path and self.chunk_path:
                self._enqueue_chunks(self._pending_axis_deg)
                self._drain_chunks()
            elif self.send_path:
                self._process_path(self._pending_axis_deg)
            else:
                # Point-to-point: single absolute move, so the per-step guard does
                # not apply (soft limits + the live gate still do).
                self._process_target(self._pending_axis_deg, enforce_step=False,
                                     track_completion=True)
        else:
            self._process_target(self._pending_axis_deg, enforce_step=self.max_step_deg > 0)

    def _chunkify(self, points, m: int):
        """Split a waypoint list into <=m-point chunks overlapping by one point
        (so each chunk starts where the previous ended -> continuous motion)."""
        if len(points) <= m:
            return [points]
        chunks, i = [], 0
        while i < len(points) - 1:
            chunks.append(points[i:i + m])
            i += m - 1
        return chunks

    def _enqueue_chunks(self, final_goal) -> None:
        """On a settled path, build the chunk queue from the full accumulated
        waypoints (validated against soft limits). Skips if already enqueued."""
        if self._chunk_goal is not None and self._max_joint_delta(final_goal, self._chunk_goal) < 1e-3:
            return
        waypoints = list(self._path_waypoints)
        if not waypoints or self._max_joint_delta(final_goal, waypoints[-1]) > 1e-3:
            waypoints.append(list(final_goal))
        for w in waypoints:
            ok, violations = calibration.within_soft_limits(w)
            if not ok:
                detail = ", ".join(
                    f"J{i + 1}={v:.2f} not in [{lo},{hi}]" for i, v, lo, hi in violations
                )
                self.get_logger().warn(
                    f"rejecting path (soft-limit): {detail}", throttle_duration_sec=2.0
                )
                return
        self._chunk_queue = self._chunkify(waypoints, self.path_max_points)
        self._chunk_goal = list(final_goal)
        self._chunk_total = len(self._chunk_queue)
        self.get_logger().info(
            f"path: {len(waypoints)} waypoints -> {self._chunk_total} chunk(s) "
            f"(<= {self.path_max_points} pts each)"
        )

    def _drain_chunks(self) -> None:
        """Send the next queued chunk once the arm is idle (gate satisfied)."""
        if not self._chunk_queue:
            return
        gate = self._check_gate()
        if gate is not True:
            self._gate_wait_count += 1
            self.get_logger().warn(
                f"waiting to send next chunk (gate): {gate}", throttle_duration_sec=2.0
            )
            return
        if not self._chunk_queue:  # /stop (other thread) may have cleared it during the query
            return
        chunk = self._chunk_queue[0]
        instructions = [
            build_free_path_instruction(w, self.speed_pct, smooth=str(self.path_smooth))
            for w in chunk
        ]
        idx = self._chunk_total - len(self._chunk_queue) + 1
        if self.dry_run:
            targets = " ".join(f"m{i}={chunk[-1][i]:.1f}" for i in range(NUM_JOINTS))
            self.get_logger().info(
                f"[DRY-RUN] would send AddRCC chunk {idx}/{self._chunk_total}: "
                f"{len(chunk)} pts -> {targets}"
            )
            self._chunk_queue.pop(0)
            if not self._chunk_queue:
                self._last_sent_deg = list(self._chunk_goal)
            return
        waits = self._gate_wait_count
        self._gate_wait_count = 0
        t0 = self.get_clock().now()
        try:
            reply = self.client.send_addrcc(
                self.service_id, instructions, timeout=self.command_timeout
            )
        except (OSError, ValueError, RuntimeError) as error:
            latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
            # Abort the rest of the path: a timed-out chunk may have been received,
            # so don't resend, and don't continue a partial path. Re-Execute.
            self.get_logger().warn(
                f"chunk {idx}/{self._chunk_total} send FAILED after {latency_ms:.0f}ms: "
                f"{error}; aborting remaining path. Re-Execute if needed."
            )
            self._chunk_queue = []
            self._last_sent_deg = list(self._chunk_goal) if self._chunk_goal else None
            return
        latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
        cmd_reply = reply.get("cmdReply", [])
        if len(cmd_reply) >= 2 and cmd_reply[1] == "ok":
            self.get_logger().info(
                f"AddRCC chunk {idx}/{self._chunk_total} ok: {len(chunk)} pts, "
                f"smooth={self.path_smooth} | latency={latency_ms:.0f}ms waits={waits}"
            )
            self._chunk_queue.pop(0)
            if not self._chunk_queue:
                self._last_sent_deg = list(self._chunk_goal)
                self._begin_completion(self._chunk_goal)
        else:
            self.get_logger().error(
                f"AddRCC chunk {idx}/{self._chunk_total} rejected: {reply}; aborting path"
            )
            self._chunk_queue = []

    def _begin_completion(self, goal) -> None:
        """Arm the completion watcher for a goal that was just sent live."""
        if self.completion_feedback and not self.dry_run:
            self._completing = {
                "goal": list(goal), "corrected": False,
                "since": self.get_clock().now(),
            }

    def _handle_completion(self) -> bool:
        """Progress the completion phase. Returns True when the timer may resume
        normal send processing (goal confirmed, abandoned, or timed out)."""
        goal = self._completing["goal"]
        # A pending setpoint far from the watched goal means a NEW trajectory is
        # streaming -- abandon the wait and let the normal chase logic run.
        if (self._pending_axis_deg is not None
                and self._max_joint_delta(self._pending_axis_deg, goal) > 1.0):
            self._completing = None
            return True
        elapsed = (self.get_clock().now() - self._completing["since"]).nanoseconds * 1e-9
        if elapsed > self.completion_timeout:
            self.get_logger().warn(
                f"completion timeout after {elapsed:.0f}s; giving up on goal confirm"
            )
            self._completing = None
            return True
        gate = self._check_gate()
        if gate is not True:
            # isMoving=1 is the normal "still executing" case; anything else
            # (alarm, mode drop, query failure) also just waits -- the timeout
            # bounds it, and correction must not fire while the gate is unmet.
            return False
        current = self._current_axis_deg()
        if current is None:
            return False
        # The stream has settled and the arm has stopped: the held setpoint is
        # the trajectory's true endpoint. Prefer it over the goal sampled at
        # send time (which the settle heuristic can clip short).
        target = list(self._pending_axis_deg) if self._pending_axis_deg else goal
        err = self._max_joint_delta(current, target)
        if err <= self.correction_tol_deg or self._completing["corrected"]:
            note = " (after correction)" if self._completing["corrected"] else ""
            self.get_logger().info(f"goal reached{note}: max err {err:.2f} deg")
            self._last_sent_deg = list(target)
            self._completing = None
            return True
        ok, _ = calibration.within_soft_limits(target)
        if not ok:
            self._completing = None
            return True
        instr = build_free_path_instruction(target, self.speed_pct)
        try:
            reply = self.client.send_addrcc(
                self.service_id, [instr], timeout=self.command_timeout
            )
        except (OSError, ValueError, RuntimeError) as error:
            # Never resend a motion command; log and confirm on the next stop.
            self.get_logger().warn(f"correction send FAILED: {error}; not resending")
            self._completing["corrected"] = True
            return False
        cmd_reply = reply.get("cmdReply", [])
        if len(cmd_reply) >= 2 and cmd_reply[1] == "ok":
            self.get_logger().info(
                f"correction sent: {err:.2f} deg short of goal "
                f"m=[{', '.join(f'{g:.2f}' for g in target)}]"
            )
            self._completing.update(goal=list(target), corrected=True)
            self._last_sent_deg = list(target)
        else:
            self.get_logger().error(f"correction rejected: {reply}")
            self._completing = None
            return True
        return False

    def _process_path(self, final_goal) -> None:
        # Dedupe on the final goal: don't re-send a path we already executed while
        # MoveIt holds the goal as a steady setpoint. Tolerance matches the
        # completion correction: anything closer counts as the same goal.
        if self._last_sent_deg is not None and self._max_joint_delta(
            final_goal, self._last_sent_deg
        ) < self.correction_tol_deg:
            return

        # Waypoints accumulated during the move, ending exactly on the goal.
        waypoints = list(self._path_waypoints)
        if not waypoints or self._max_joint_delta(final_goal, waypoints[-1]) > 1e-3:
            waypoints.append(list(final_goal))
        # The controller rejects long instruction lists -- downsample evenly to a
        # safe length, always keeping the first waypoint and the final goal.
        n = len(waypoints)
        if n > self.path_max_points:
            m = self.path_max_points
            idx, last = [], -1
            for k in range(m):
                j = round(k * (n - 1) / (m - 1))
                if j != last:
                    idx.append(j)
                    last = j
            waypoints = [waypoints[j] for j in idx]

        # All waypoints must satisfy the soft limits or the whole path is refused.
        for w in waypoints:
            ok, violations = calibration.within_soft_limits(w)
            if not ok:
                detail = ", ".join(
                    f"J{i + 1}={v:.2f} not in [{lo},{hi}]" for i, v, lo, hi in violations
                )
                self.get_logger().warn(
                    f"rejecting path (soft-limit): {detail}", throttle_duration_sec=2.0
                )
                return

        gate = self._check_gate()
        if gate is not True:
            self._gate_wait_count += 1
            self.get_logger().warn(
                f"rejecting path (gate): {gate}", throttle_duration_sec=2.0
            )
            return

        instructions = [
            build_free_path_instruction(w, self.speed_pct, smooth=str(self.path_smooth))
            for w in waypoints
        ]
        summary = (
            f"{len(waypoints)} pts, smooth={self.path_smooth}, "
            f"goal m=[{', '.join(f'{g:.1f}' for g in final_goal)}]"
        )

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] would send AddRCC path: {summary}")
            for k, w in enumerate(waypoints):
                self.get_logger().info(
                    f"[DRY-RUN]   wp{k}: " + " ".join(f"m{i}={w[i]:.2f}" for i in range(NUM_JOINTS))
                )
            self._last_sent_deg = list(final_goal)
            return

        waits = self._gate_wait_count
        self._gate_wait_count = 0
        t0 = self.get_clock().now()
        try:
            reply = self.client.send_addrcc(
                self.service_id, instructions, timeout=self.command_timeout
            )
        except (OSError, ValueError, RuntimeError) as error:
            latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
            # A timed-out send may already have been received -- mark it sent so we
            # never auto-resend the path. Re-Execute if the arm did not move.
            self._last_sent_deg = list(final_goal)
            self.get_logger().warn(
                f"AddRCC path send FAILED after {latency_ms:.0f}ms: {error} "
                f"(n_pts={len(waypoints)}, waits={waits}); not resending. "
                f"Re-Execute if the arm did not move."
            )
            return
        latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6

        cmd_reply = reply.get("cmdReply", [])
        if len(cmd_reply) >= 2 and cmd_reply[1] == "ok":
            self.get_logger().info(
                f"AddRCC path ok: {summary} | latency={latency_ms:.0f}ms "
                f"n_pts={len(waypoints)} waits={waits}"
            )
            self._last_sent_deg = list(final_goal)
            self._begin_completion(final_goal)
        else:
            self.get_logger().error(f"AddRCC path rejected: {reply}")

    def _process_target(self, axis_deg, enforce_step: bool = True,
                        track_completion: bool = False) -> None:
        # Dedupe: don't re-send a target we already accepted (avoids spamming the
        # controller / dry-run log while the input holds a steady setpoint).
        if self._last_sent_deg is not None and all(
            abs(axis_deg[i] - self._last_sent_deg[i]) < self.correction_tol_deg
            for i in range(NUM_JOINTS)
        ):
            return

        ok, violations = calibration.within_soft_limits(axis_deg)
        if not ok:
            detail = ", ".join(
                f"J{i + 1}={v:.2f} not in [{lo},{hi}]" for i, v, lo, hi in violations
            )
            self.get_logger().warn(
                f"rejecting command (soft-limit): {detail}", throttle_duration_sec=2.0
            )
            return

        gate = self._check_gate()
        if gate is not True:
            self._gate_wait_count += 1
            self.get_logger().warn(
                f"rejecting command (gate): {gate}", throttle_duration_sec=2.0
            )
            return

        current = self._current_axis_deg()
        if enforce_step and self.max_step_deg > 0 and current is not None:
            big = [
                (i, axis_deg[i], current[i])
                for i in range(NUM_JOINTS)
                if abs(axis_deg[i] - current[i]) > self.max_step_deg
            ]
            if big:
                detail = ", ".join(
                    f"J{i + 1} {cur:.1f}->{tgt:.1f}" for i, tgt, cur in big
                )
                self.get_logger().warn(
                    f"rejecting command (step > {self.max_step_deg:g} deg): {detail}",
                    throttle_duration_sec=2.0,
                )
                return

        instr = build_free_path_instruction(axis_deg, self.speed_pct)
        targets = " ".join(f"m{i}={axis_deg[i]:.2f}" for i in range(NUM_JOINTS))

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] would send AddRCC: {targets}")
            self._last_sent_deg = list(axis_deg)
            return

        waits = self._gate_wait_count
        self._gate_wait_count = 0
        t0 = self.get_clock().now()
        try:
            reply = self.client.send_addrcc(
                self.service_id, [instr], timeout=self.command_timeout
            )
        except (OSError, ValueError, RuntimeError) as error:
            latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
            # A timed-out send may already have been received by the controller --
            # mark it sent so we never auto-resend (which could move twice).
            # Re-Execute in RViz if the arm did not actually move.
            self._last_sent_deg = list(axis_deg)
            self.get_logger().warn(
                f"AddRCC TIMEOUT after {latency_ms:.0f}ms (n_pts=1, waits={waits}); "
                f"not resending. Re-Execute if the arm did not move."
            )
            return
        latency_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6

        cmd_reply = reply.get("cmdReply", [])
        if len(cmd_reply) >= 2 and cmd_reply[1] == "ok":
            self.get_logger().info(
                f"AddRCC ok: {targets} | latency={latency_ms:.0f}ms n_pts=1 waits={waits}"
            )
            self._last_sent_deg = list(axis_deg)
            if track_completion:
                self._begin_completion(axis_deg)
        else:
            self.get_logger().error(f"AddRCC rejected: {reply}")

    def _extract_targets(self, msg: JointState):
        if not msg.position:
            raise ValueError("empty position array")
        if msg.name and len(msg.name) == len(msg.position):
            index = {n: i for i, n in enumerate(msg.name)}
            missing = [n for n in self.joint_names if n not in index]
            if missing:
                raise ValueError(f"missing joints {missing}")
            return [float(msg.position[index[n]]) for n in self.joint_names]
        if len(msg.position) == NUM_JOINTS:
            return [float(p) for p in msg.position]  # assume canonical order
        raise ValueError(
            f"got {len(msg.position)} positions without matching names; "
            f"expected {NUM_JOINTS} or a name->position map"
        )

    def _check_gate(self):
        try:
            data = self.client.query(GATE_ADDRESSES)
        except (OSError, ValueError, RuntimeError) as error:
            return f"status query failed: {error}"
        if not _is(data.get("curMode"), REQUIRED_MODE):
            return f"curMode={data.get('curMode')} (need {REQUIRED_MODE})"
        if not _is(data.get("curAlarm"), "0"):
            return f"curAlarm={data.get('curAlarm')}"
        if not _is(data.get("isMoving"), "0"):
            return "isMoving=1"
        if not _is(data.get("origin"), "1"):
            return "origin!=1"
        return True

    def _current_axis_deg(self):
        try:
            data = self.client.query([f"axis-{i}" for i in range(NUM_JOINTS)])
            return [float(data[f"axis-{i}"]) for i in range(NUM_JOINTS)]
        except (OSError, ValueError, RuntimeError, KeyError):
            return None

    def on_stop(self, request, response):
        """Immediately halt motion (actionStop) and suppress further sends until a
        new goal arrives. Uses a dedicated connection so it works mid-send."""
        try:
            result = self.stop_client.send_command(self.stop_command)
        except (OSError, ValueError, RuntimeError) as error:
            response.success = False
            response.message = f"{self.stop_command} send failed: {error}"
            self.get_logger().error(f"/stop: {response.message}")
            return response
        # Latch halt on the currently-held goal so the bridge does not re-send it,
        # and drop any queued path chunks so the aborted path is not continued.
        self._halt_goal = list(self._pending_axis_deg) if self._pending_axis_deg else None
        self._halted = True
        self._path_waypoints = []
        self._chunk_queue = []
        self._chunk_goal = None
        self._completing = None
        response.success = bool(result.get("ok"))
        response.message = (
            f"{self.stop_command} sent; motion halted. Re-arm the pendant "
            f"(curMode 7) if needed, then Execute a new goal to resume."
            if response.success else f"controller did not ack: {result.get('reply')}"
        )
        self.get_logger().warn(f"/stop: {response.message}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MotionBridge()
    except RuntimeError as error:
        print(f"startup failed: {error}")
        rclpy.shutdown()
        return
    # MultiThreadedExecutor so the /stop service runs on its own thread (and
    # connection) and can abort even while the main loop is mid-send.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: halt any in-progress motion on shutdown.
        try:
            node.stop_client.send_command(node.stop_command)
        except (OSError, ValueError, RuntimeError):
            pass
        node.client.close()
        node.stop_client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
