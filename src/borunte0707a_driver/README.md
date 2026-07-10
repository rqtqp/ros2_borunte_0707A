# borunte0707a_driver

ROS 2 driver for the Borunte **BRTIRUS0707A** (HC1 controller) over the existing
JSON-over-TCP RemoteMonitor / HCRemoteCommand interface (port 9760).

See [`../../README.md`](../../README.md) for the workspace overview and the
phased roadmap. Nodes:

- `joint_state_publisher` (Phase 1, read-only) — polls `axis/curSpeed/curTorque`
  and publishes `sensor_msgs/JointState` on `/joint_states` (URDF sign/offset
  applied via `calibration.py`).
- `status_node` (Phase 2, read-only) — polls the controller's mode/health fields:
  - `/robot_status` (`std_msgs/String`, JSON snapshot incl. decoded mode name)
  - `/motion_ready` (`std_msgs/Bool`, the AddRCC safety gate as one boolean)
  - `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, for `rqt_robot_monitor`)
- `motion_bridge` (**motion**) — subscribes `joint_command` (`JointState`, URDF
  rad), converts to controller degrees, validates the live gate + soft limits,
  and emits `AddRCC` free-path motion. Drives the real arm from MoveIt via
  `real.launch.py`. Exposes a `/stop` abort service. **`dry_run=true` by
  default**: logs the packet without sending. See "Motion bridge" below.

## Prerequisites

Telemetry works anytime. **Motion** (`AddRCC`, Phase 3) only works while the
controller is in `curMode=7` — a pendant program running and waiting for remote
commands. Set this up once on the teach pendant; see
[rqtqp/borunte-pc-rc](https://github.com/rqtqp/borunte-pc-rc) (network = port
`9760`/Serve; a "Long Distance Command" program with Data Source
`www.hc-system.com.HCRemoteCommand::[HID:100]`, run in Auto + Cycle mode). The
workspace [`../../README.md`](../../README.md#prerequisites--make-the-arm-controllable)
has the step-by-step. `status_node`'s `/motion_ready` confirms when the arm is
ready.

## Layout

| File | Role |
|------|------|
| `borunte0707a_driver/hc1_client.py` | Pure-socket protocol client (query + AddRCC). No rclpy dep. |
| `borunte0707a_driver/env_config.py` | Loads workspace `.env` so config isn't duplicated. |
| `borunte0707a_driver/calibration.py` | Controller↔URDF sign/offset map + soft limits (shared). |
| `borunte0707a_driver/joint_state_publisher_node.py` | Phase 1 telemetry node. |
| `borunte0707a_driver/status_node.py` | Phase 2 status / diagnostics node. |
| `borunte0707a_driver/motion_bridge_node.py` | Phase 3 command → AddRCC motion bridge. |
| `borunte0707a_driver/calibration_helper.py` | Read-only sign/offset calibration tool (no motion). |
| `borunte0707a_driver/kin_calibrate.py` | Precise offset calibration vs the controller's TCP readout. |

## Build & run

```bash
cd <workspace root>
colcon build --packages-select borunte0707a_driver
source install/setup.bash

# robot_ip defaults to ROBOT_IP from the workspace .env
ros2 run borunte0707a_driver joint_state_publisher
ros2 topic echo /joint_states

# controller health + AddRCC readiness gate:
ros2 run borunte0707a_driver status_node
ros2 topic echo /motion_ready

# override config:
ros2 run borunte0707a_driver joint_state_publisher --ros-args \
  -p robot_ip:=10.0.0.49 -p publish_rate_hz:=20.0
```

> ROS 2 Humble runs in **WSL2 Ubuntu 22.04** (`wsl -d Ubuntu-22.04`) for
> development; **deployment runs on the Jetson** wired to the controller
> (see the sensors/vision repo's `jetson_arm_host_migration.md`). Verify the
> controller path from the deployment host first:
>
> ```bash
> ros2 run borunte0707a_driver hc1_ping     # read-only round-trip + latency
> ```

## Motion test tools

With the full stack up (`real.launch.py`) and the trajectory controller
**active**, two console tools drive validated test motion (both fail-closed on
a `Bool` health gate topic, `--gate-topic`, default `arm_motion_ok`):

```bash
# MoveIt Plan+Execute to a joint-space goal (URDF degrees); blocks on REAL arrival
ros2 run borunte0707a_driver plan_exec -- 20 -8 15 0 -10 0
ros2 run borunte0707a_driver plan_exec -- home

# Repeatability: N cycles between two 6-joint poses per speed, encoder stats
# (sets the bridge's runtime speed_pct per phase; parks home when done)
ros2 run borunte0707a_driver repeatability_test -- --cycles 5 --speeds 10,20,30
```

Measured 2026-07-10: worst-joint spread **0.000°** (encoder level) at every
speed and endpoint; travel 25.3/16.4/13.5 s at 10/20/30 %.

## Motion bridge (Phase 3)

`motion_bridge` turns a target `JointState` (URDF radians) into an `AddRCC`
free-path point. It is **dry-run by default** and never moves the arm until you
pass `dry_run:=false`.

```bash
# 1. Watch what it WOULD send (safe — no motion). Requires curMode=7.
ros2 run borunte0707a_driver motion_bridge          # [DRY-RUN]
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  '{name: [brtirus0707a_joint_1,brtirus0707a_joint_2,brtirus0707a_joint_3,\
           brtirus0707a_joint_4,brtirus0707a_joint_5,brtirus0707a_joint_6],
    position: [0.0,0.0,0.0,0.0,0.0,0.0]}'
# -> logs "[DRY-RUN] would send AddRCC: m0=.. m5=.."  (verify the m-values!)

# 2. First real move — operator at the e-stop, low speed, small step.
ros2 run borunte0707a_driver motion_bridge --ros-args \
  -p dry_run:=false -p speed_pct:=5.0 -p max_step_deg:=5.0
```

Guards enforced before every send (all re-queried live):

| Guard | Reject condition |
|-------|------------------|
| Motion gate | `curMode≠7`, `curAlarm≠0`, `isMoving=1`, or `origin≠1` |
| Soft limits | any target outside `calibration.SOFT_LIMITS_DEG` (controller deg) |

### How a MoveIt trajectory becomes AddRCC

MoveIt streams a time-based trajectory, but `AddRCC` is **point-to-point** (it
moves to a target and sets `isMoving`). The bridge coalesces the stream and waits
for it to settle (`goal_settle_sec`), then sends the move. The controller
reliably accepts only **short** instruction lists (≤~8 points), so:

- **`send_path=false`** — send only the final goal as one `AddRCC` (point-to-point).
- **`send_path=true`** (default in `real.launch.py`) — follow the planned path:
  - **`chunk_path=false`** (default) — downsample the whole path to one ≤8-point
    blended `AddRCC`. **Smooth**, slight corner-cutting on long paths.
    - **`chunk_path=true`** — send the full path as sequential ≤8-point `AddRCC`
    segments, each fired when the arm finishes the previous (gated on `isMoving`).
    Faithful, but **stops at each segment boundary**.

### `/stop` abort

`ros2 service call /stop std_srvs/srv/Trigger` immediately halts motion
(`actionStop`) on a dedicated connection (works mid-send) and latches a halt
until a new goal. The bridge also stops on shutdown. **`actionStop` drops the
controller `curMode 7 → 2`**, so re-arm the pendant before resuming.

### Parameters

`command_topic` (`joint_command`), `dry_run` (`true`), `speed_pct` (`10`, % of
max), `goal_settle_sec` (`0.5`), `send_path` (`false`; `real.launch.py` sets
`true`), `path_waypoint_deg` (`5`), `path_smooth` (`1`, 0–9), `path_max_points`
(`8`), `chunk_path` (`false`), `command_timeout` (`8` s), `stop_command`
(`actionStop`), `max_step_deg`/`max_rate_hz` (legacy per-setpoint streaming),
`sign`/`offset_rad` (calibration overrides — see `calibration.py`).

For MoveIt-driven motion the bridge is started by `real.launch.py`, which binds
it to the `topic_based_ros2_control` command stream (`/joint_command`) and feeds
joint state back via `/hw_joint_states`.

> ⚠️ Run only **one** telemetry consumer at a time — the HC1 RemoteMonitor has a
> tiny socket pool; stray driver nodes can wedge it (queries time out) and need a
> controller power-cycle.

## Calibration (read-only, no motion)

**The authoritative zero is the per-joint alignment grooves.** Seat the flat
blade gauge flush in each joint's groove to set the mechanical zero; that pose
**is** the URDF zero — all `axis-N` read ~0 and the model sits at `q=0`. So
`SIGN=(+1,−1,−1,+1,−1,−1)` with `OFFSET_RAD ≈ 0` is correct and already in
`calibration.py`. Don't re-calibrate from an eyeballed "home" — it can be tens of
degrees off. Full groove procedure: [`../../docs/HOME_CALIBRATION.md`](../../docs/HOME_CALIBRATION.md).

**Sign check** (visual, the real test): launch the real-arm view (model driven by
driver telemetry, not `display.launch.py`'s sliders) and jog each joint — the
model must move the **same physical way** as the arm.

```bash
ros2 launch brtirus0707a_description view_real.launch.py
# optional numeric index/scale check (J2/J3/J5/J6 show "sign -1" by design):
ros2 run borunte0707a_driver calibration_helper
```

**Offset confirmation**: with the pins seated, snapshot — the implied offsets
should be ~0, confirming the defaults.

```bash
ros2 run borunte0707a_driver calibration_helper --capture-zero
```

> Note: `kin_calibrate` (a least-squares fit of the URDF FK to the controller's
> TCP readout) is **deprecated for joint zeros** — its free base/tool transforms
> absorb the joint offsets, so it can report misleading values. Trust the pinned
> zero. The tool remains only for Cartesian/world-frame investigations.

## Notes

- Joint names default to `brtirus0707a_joint_1..6` to match the URDF.
- `position` is radians, `velocity` rad/s, `effort` ×rated torque (`2580 = 1×`).
- `/motion_ready` is `True` only when `curMode∈{2,7}`, `curAlarm=0`, `isMoving=0`,
  `origin=1`. Phase 3's bridge must re-check this immediately before each `AddRCC`.
- If the controller does not expose `curSpeed`/`curTorque`, velocity/effort are
  published as `0.0` and a one-time warning is logged.
- Tested ROS 2 distro: Humble+ (rclpy, sensor_msgs only).
