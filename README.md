# ros2_borunte_0707A

Dedicated ROS 2 workspace for the **Borunte BRTIRUS0707A** 6-axis arm on an
**HC1** controller — robot model, MoveIt 2 config, and the real-hardware driver
that bridges ROS 2 to the controller's JSON/TCP interface (port 9760).

## Packages (`src/`)

| Package | Build type | Role |
|---------|-----------|------|
| `brtirus0707a_description` | ament_cmake | URDF/xacro + CAD (Autodesk) meshes + joint limits. Vendor-correct limits. |
| `brtirus0707a_moveit_config` | ament_cmake | MoveIt 2 config (SRDF, kinematics, OMPL, controllers). Drives the real arm via `real.launch.py`. |
| `borunte0707a_driver` | ament_python | **Real HC1 bridge.** Telemetry (`/joint_states`, status), sign/offset calibration, and the motion bridge (joint command → `AddRCC`, dry-run by default). |

## Status — working end to end (live-validated)

- ✅ **Telemetry** — `joint_state_publisher` → `/joint_states` (sign/offset
  calibrated; one persistent controller connection reused across queries).
- ✅ **Signalling** — `status_node`: `/robot_status`, `/motion_ready`,
  `/diagnostics` (decoded `curMode`/`curAlarm`/`isMoving`/`origin`).
- ✅ **Motion** — `motion_bridge`: joint command → `AddRCC` behind the live
  safety gate + soft limits; **dry-run by default**.
- ✅ **RViz MoveIt → real arm** (`real.launch.py`): Plan + Execute moves the
  physical arm. Validated live. MoveIt uses **OMPL** and plans within the
  controller soft limits.
  - **Path following** (default): the bridge sends MoveIt's path as one blended
    `AddRCC` of ≤8 waypoints (smooth). `chunk_path:=true` sends the full path as
    sequential segments (faithful, but stops at each segment boundary).
    `stream_path:=true` sends the full path as chunks **appended while the arm
    is moving** (`emptyList=0`) — faithful AND smooth, zero mid-move pauses
    (live-validated at encoder level; see below).
  - **`/stop` service** — ROS-level abort (`actionStop`); also stops on shutdown.
  - **Completion feedback** (default on): after each send the bridge waits for
    the arm to actually stop, corrects the goal if it landed short, and logs
    "goal reached: max err X deg" (live-validated: ≤0.05° terminal accuracy).
- ✅ **Smooth motion (Phase 6, live-validated 2026-07-15)** — the AddRCC
  `smooth` field is a 0–9 blending level (level ≥1 removes waypoint stops,
  terminal accuracy unaffected), and `emptyList:"0"` **appends to the
  controller's instruction list mid-execution with seamless blending** — the
  streaming primitive behind `stream_path`. Protocol notes, experiment runbook
  (`smooth_lab`), and the full findings log:
  [`docs/HC1_SMOOTH_MOTION.md`](docs/HC1_SMOOTH_MOTION.md).
- ✅ **Link hardening** — each node reuses one persistent controller connection
  with segment-safe JSON reply framing (a reply split across TCP segments can
  never desync the socket), `TCP_NODELAY` + `SO_KEEPALIVE`, and a strict
  never-resend rule for timed-out motion commands. `hc1_ping` is a standalone
  read-only preflight (round-trips + latency) for checking the controller link.
- **Calibration**: the URDF zero pose **is** the controller's mechanical home —
  seat the factory dowel/pin grooves to define zero (`axis-N == 0 → q = 0`);
  offsets are ~0. See `borunte0707a_driver/calibration.py`.

## Prerequisites — make the arm controllable

Telemetry (`query`) works anytime, but the controller only **accepts motion**
(`AddRCC`) while a pendant program is running and waiting for remote commands —
i.e. `curMode=7`. Set that up once on the teach pendant (see
[rqtqp/borunte-pc-rc](https://github.com/rqtqp/borunte-pc-rc) for the full
walkthrough + a ready-made `pc_rc.zip` program):

1. **Network** (pendant → CommunicateMode1): port `9760`, mode **Serve**; put the
   driver host on the same subnet as the controller (e.g. `192.168.1.x /
   255.255.255.0`). Run the driver on a host with a **direct, ideally dedicated
   wired route** to the controller — do not forward `:9760` across networks or
   NATs: the RemoteMonitor socket pool is tiny and extra clients wedge it (see
   the warning below).
2. **Pendant program**: in **Manual** mode create a program with one
   **"Long Distance Command"** instruction whose Data Source is
   `www.hc-system.com.HCRemoteCommand::[HID:100]` (or import `pc_rc.zip` via USB).
3. **Run it**: switch to **Auto**, open the program, enable **Cycle mode**, press
   **Start** → controller enters `curMode=7` (waiting for RC).

Confirm with `status_node`: `/motion_ready` goes `true` once `curMode=7`,
`curAlarm=0`, `isMoving=0`, `origin=1`. Tested controller firmware:
`HC-QC-RX-7.8.07-master-F5.2.1`.

For the first ROS 2 motion, follow the staged runbook in
[`FIRST_MOTION_CHECKLIST.md`](FIRST_MOTION_CHECKLIST.md) (dry-run → live, with an
operator at the e-stop).

> ROS 2 Humble runs in **WSL2 Ubuntu 22.04** (`wsl -d Ubuntu-22.04`); the repo is
> at `/path/to/ros2_borunte_0707A` (under `/mnt/c/...` if cloned on the Windows
> filesystem). Run all `colcon`/`ros2` commands there. That is the
> **development/build** environment — to run against the arm, deploy this
> workspace on a host wired to the controller (this rig runs it in a plain
> `ros:humble` Docker container on a Jetson that gateways the arm's dedicated
> ethernet link; the perception/integration workspace documents that setup).

## Build & run

```bash
cp .env.example .env        # set ROBOT_IP
colcon build
source install/setup.bash
colcon test --packages-select borunte0707a_driver   # 51 offline tests, no robot needed

# preflight: verify the controller link (read-only round-trips + latency)
ros2 run borunte0707a_driver hc1_ping

# read-only telemetry from the real arm
ros2 run borunte0707a_driver joint_state_publisher
ros2 topic echo /joint_states

# read-only controller status + AddRCC readiness gate
ros2 run borunte0707a_driver status_node
ros2 topic echo /motion_ready

# visualize the model (slider GUI — NOT connected to the arm)
ros2 launch brtirus0707a_description display.launch.py
# visualize the model driven by the REAL arm telemetry (read-only)
ros2 launch brtirus0707a_description view_real.launch.py

# Phase 4 — drive the REAL arm from MoveIt in RViz (Plan + Execute).
# Dry-run by default (logs the AddRCC it WOULD send; no motion):
ros2 launch brtirus0707a_moveit_config real.launch.py
# Live motion — operator at the e-stop, workcell clear, low speed:
ros2 launch brtirus0707a_moveit_config real.launch.py dry_run:=false speed_pct:=10.0

# abort motion from ROS at any time (also stops on Ctrl-C):
ros2 service call /stop std_srvs/srv/Trigger
```

> After `/stop` (or Ctrl-C) the controller leaves `curMode=7` (auto-running) for
> `2` (auto-idle) — **re-arm the pendant program** (Start/Cycle → `curMode 7`)
> before motion resumes.

> ⚠️ Run **only one** telemetry consumer at a time. The HC1 RemoteMonitor has a
> tiny socket pool; leaving stray driver nodes running can wedge it (queries time
> out) and require a controller power-cycle to recover.

## Integration notes

- **Joint names** match the URDF: `brtirus0707a_joint_1..6` (driver default).
- **Sign/offset calibration**: `SIGN=(+1,−1,−1,+1,−1,−1)` (the URDF flips axes on
  J2/3/5/6) with ~0 offsets, because the URDF zero coincides with the pinned
  mechanical home. The single source of truth is `calibration.py`, shared by the
  telemetry publisher and the motion bridge. **Authoritative zero = per-joint
  alignment grooves** (blade-gauge procedure in
  [`docs/HOME_CALIBRATION.md`](docs/HOME_CALIBRATION.md)); never calibrate from an
  eyeballed home.
- **Motion is point-to-point.** `AddRCC` moves the arm to a target and reports
  `isMoving`; the bridge waits for MoveIt's streamed trajectory to settle
  (`goal_settle_sec`), then sends it as a blended `AddRCC` path. The controller
  reliably accepts only **short** instruction lists (≤~8 points), so a long path
  is downsampled to one ≤8-point `AddRCC` (`chunk_path:=false`, default, smooth),
  split into sequential segments (`chunk_path:=true`, faithful, stops at seams),
  or **streamed** (`stream_path:=true`, faithful and smooth: chunks after the
  first are appended with `emptyList=0` while the arm moves and the controller
  blends across the seams — needs `path_smooth≥1`; see
  [`docs/HC1_SMOOTH_MOTION.md`](docs/HC1_SMOOTH_MOTION.md)).
- **MoveIt plans within the controller soft limits** (`joint_limits.yaml`
  mirrors `calibration.SOFT_LIMITS_DEG`), so it won't produce goals the bridge
  rejects. Keep the two in sync if the soft limits change.
- **No collision world.** MoveIt knows only the robot's own links; add any
  workcell obstacles to the planning scene yourself if you need them.
- **Telemetry caveat**: `curTorque`/`curSpeed` are documented "mainboard network
  version" only; the publisher falls back to position-only if they are absent.

## License

Free software under the **GNU General Public License v3.0 or later**
(`GPL-3.0-or-later`) — see [`LICENSE`](LICENSE). You may use, study, share, and
modify it; derivative works must remain free under the same terms.
