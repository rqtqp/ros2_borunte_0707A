# ROS 2 Integration Plan — Borunte BRTIRUS0707A (HC1)

Status legend: ✅ done · 🚧 in progress · ⬜ planned

## Goal

Drive the physical BRTIRUS0707A from ROS 2 / MoveIt 2 using the existing
**JSON-over-TCP** interface (RemoteMonitor query + HCRemoteCommand `AddRCC`,
port 9760). Start read-only; add motion only behind the safety gate
(see `CLAUDE.md` → Safety Rules).

The driver is a thin wrapper over the controller's documented JSON/TCP
payloads — no new protocol is invented.

## Architecture

```
ROS 2 graph                              HC1 controller (ROBOT_IP:9760)
-----------                              ------------------------------
borunte0707a_driver/hc1_client.py  ──►  RemoteMonitor  query   (telemetry)
                                   ──►  HCRemoteCommand AddRCC  (motion)

joint_state_publisher  ──►  /joint_states  (sensor_msgs/JointState)
status_node (ph.2)     ──►  /robot_status, /diagnostics
topic_bridge (ph.3)    ◄──  joint command topic  ─►  AddRCC
                       ──►  joint state topic
MoveIt 2 (ph.4)        ──►  plans → command topic
```

`hc1_client.py` is pure sockets (no rclpy) so it stays testable and reusable.

## Key insight — reuse `topic_based_ros2_control`

`brtirus0707a_moveit_config` already drives a robot through
`topic_based_ros2_control/TopicBasedSystem`, currently pointed at Isaac Sim
topics (`/isaac_joint_commands`, `/isaac_joint_states`). To drive the **real**
arm we therefore do **not** need a custom `hardware_interface` plugin — the
driver just has to:

1. **publish** real joint state (`sensor_msgs/JointState`) to the state topic
   the TopicBasedSystem reads, and
2. **subscribe** to the command topic and translate to `AddRCC`.

This is implemented: the topics are xacro args (default Isaac), and
`real.launch.py` overrides them to the driver's `/joint_command` /
`/hw_joint_states` for the real-hardware profile.

## Conventions

| Quantity | Controller (query) | ROS message | Conversion |
|----------|--------------------|-------------|------------|
| position | `axis-0..5`, deg | `JointState.position`, rad | `deg·π/180` |
| velocity | `curSpeed-0..5`, RPM | `JointState.velocity`, rad/s | `rpm·2π/60` |
| effort | `curTorque-0..5`, `2580 = 1×` | `JointState.effort`, ×rated | `val/2580` |

Joint names: `brtirus0707a_joint_1..6` (match URDF). **Per-joint sign + offset
calibration is required** (URDF flips axes on joints 2/3/5/6).

## Phases

- ✅ **Phase 0 — Scaffold + protocol client.** `borunte0707a_driver` colcon
  package; `hc1_client.py` (query + AddRCC builder); `.env` reuse via
  `env_config.py`.
- ✅ **Phase 1 — Read-only `JointState` publisher (MINIMUM).** Polls
  `axis/curSpeed/curTorque`, publishes `/joint_states`. Zero motion.
- ✅ **Model + MoveIt 2 config present** (`brtirus0707a_description`,
  `brtirus0707a_moveit_config`) — CAD meshes, vendor-correct limits.
- ✅ **Phase 2 — Status / diagnostics.** `status_node` publishes `curMode`,
  `curAlarm`, `isMoving`, `origin` (+ decoded mode name) to `/robot_status`
  (`std_msgs/String` JSON), the AddRCC safety gate to `/motion_ready`
  (`std_msgs/Bool`), and a `diagnostic_msgs/DiagnosticArray` to `/diagnostics`.
  Verified against the live arm (10.0.0.49, firmware F5.2.1).
- ✅ **Phase 3 — Calibration + motion bridge.** Live-validated; real motion done.
  - `calibration.py` — shared sign/offset map (`SIGN=(+1,-1,-1,+1,-1,-1)`,
    `OFFSET_RAD ≈ 0`) + controller-space soft limits, used by both publisher and
    bridge so state and command round-trip exactly. **The authoritative zero is
    the factory dowel/pin grooves**: pins seated → all `axis-N ≈ 0` → URDF `q=0`,
    so offsets are ~0. SIGN was confirmed empirically by jog-direction tests.
    (`kin_calibrate` is deprecated for joint zeros — its free base/tool fit
    absorbs the offsets and misreports them.)
  - `motion_bridge` — `joint_command` (URDF rad) → `AddRCC` free-path
    (`action:"4"`, `m0..m5`+`m6/m7=0`, `ckStatus:"0x3F"`). Enforces the live gate
    (curMode 7, curAlarm 0, isMoving 0, origin 1) + soft limits. **`dry_run=true`
    by default.** Exposes a `/stop` abort service (`actionStop`).
- ✅ **Phase 4 — Real-arm MoveIt 2 bringup.** `real.launch.py` binds
  `TopicBasedSystem` to the driver (bridge reads `/joint_command`, publisher
  feeds `/hw_joint_states`), brings up move_group with **OMPL** + defined accel
  limits, and plans within the controller soft limits. Plan + Execute in RViz
  moves the real arm (validated live). The bridge waits for the streamed
  trajectory to settle, then sends it as a blended `AddRCC`: a long path is
  downsampled to one ≤8-point `AddRCC` (`chunk_path=false`, default, smooth) or
  split into sequential segments (`chunk_path=true`, faithful but pausing).
- ✅ **Phase 5 — Hardening.** Done: persistent reused HC1 connection,
  no-resend-on-motion-timeout, `/stop` + stop-on-shutdown, segment-safe JSON
  reply framing (desync-proof reconnect) + `TCP_NODELAY`/`SO_KEEPALIVE`,
  `hc1_ping` link preflight, OS-env config override (live-validated on the
  Jetson-gateway link, jog J1 ±5°, AddRCC 17–22 ms), **completion feedback**
  (`completion_feedback`: poll the gate until `isMoving=0`, one exact-goal
  correction if >`correction_tol_deg` off, definitive "goal reached" log —
  live-validated: home accuracy ~0.6°→0.05°), quieter gate-wait logs
  (`isMoving=1` flow control is DEBUG; real gate problems stay WARN), automated
  tests (`src/borunte0707a_driver/test/`, 27 tests: client framing/reconnect/
  no-resend vs a fake controller socket, calibration round-trip + soft limits,
  bridge chunking/downsample/dedupe/completion state machine — run with
  `colcon test --packages-select borunte0707a_driver`). Remaining (optional):
  smooth+faithful streaming (`AddRCC` append + `RemoteCmdLen` flow control) —
  needs supervised live experimentation with an untested controller feature.
- 🔬 **Phase 6 — Smooth motion (in progress).** Vendor confirmed (2026-07-15)
  that `smooth` is a 0–9 blending level and `emptyList:"0"` APPENDS to a
  persistent instruction list (a potential streaming primitive). The
  `smooth_lab` harness implements the supervised experiment ladder E1 (smooth
  sweep) → E2 (append idle) → E3 (append while moving) → E4 (Cartesian
  action 10/17); `path_smooth` is runtime-settable for sweeps. Plan, runbook,
  and findings log: `docs/HC1_SMOOTH_MOTION.md`. A bridge `stream_path` mode
  (E5) is designed but blocked on an E3 pass.

## Resolved / open questions

1. ✅ Real `curTorque`/`curSpeed`? Present in `queryData` (not null); idle reads
   `0`/`-1`. Confirm non-zero under load before trusting `effort`.
2. ✅ Streaming vs. PTP? `AddRCC` is point-to-point and the controller reliably
   accepts only **short** instruction lists (≤~8 points; ≥~20 are rejected). So
   the bridge sends MoveIt's settled path as one ≤8-point blended `AddRCC`
   (downsample) or sequential segments (chunk), not a high-rate setpoint stream.
3. ✅ Topic naming: `real.launch.py` points the xacro's `topic_based_ros2_control`
   args at the driver topics (`/joint_command`, `/hw_joint_states`).
4. 🔬 Smooth **and** faithful long paths? Vendor confirms `emptyList:"0"`
   appends to a persistent list; whether appends are legal *while moving* (and
   blend across the boundary) is exactly experiment E3 in
   `docs/HC1_SMOOTH_MOTION.md` — run `smooth_lab e3` supervised to resolve.

## Quick start (this session)

```bash
cp .env.example .env        # set ROBOT_IP
colcon build && source install/setup.bash
ros2 run borunte0707a_driver joint_state_publisher           # telemetry
ros2 launch brtirus0707a_moveit_config real.launch.py        # MoveIt, dry-run
#   ...add  dry_run:=false speed_pct:=5.0  with an operator at the e-stop
```
