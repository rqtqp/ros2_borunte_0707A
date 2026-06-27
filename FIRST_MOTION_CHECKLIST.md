# First Motion Checklist — BRTIRUS0707A via ROS 2

Operator runbook for the **first time the arm moves under ROS 2 control**. Work
top-to-bottom; do not skip the dry-run stages. Everything through Stage 3 is
read-only / no motion. Real motion only starts in Stage 5.

> **Have one person at the physical e-stop for Stages 4–5.** Keep the workcell
> clear. Start at the lowest speed. You can abort any stage with `Ctrl-C` in its
> terminal, but that is **not** a substitute for the e-stop.

All commands run inside WSL Ubuntu 22.04:

```bash
wsl -d Ubuntu-22.04
cd /path/to/ros2_borunte_0707A
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Stage 0 — Preconditions (one-time / per power-cycle)

- [ ] Arm powered, no physical obstructions, e-stop reachable.
- [ ] `.env` has the right `ROBOT_IP` (currently `10.0.0.49`).
- [ ] Pendant program for remote control is running so the controller is in
      **curMode 7** (see `README.md` → Prerequisites / rqtqp/borunte-pc-rc).
- [ ] Workspace built: `colcon build && source install/setup.bash`.

Confirm the controller is reachable and ready (read-only):

```bash
ros2 run borunte0707a_driver status_node
ros2 topic echo /motion_ready      # must print: data: true
```

`/motion_ready` is `true` only when `curMode=7, curAlarm=0, isMoving=0, origin=1`.
If it is `false`, fix that before continuing (check `/robot_status`).

---

## Stage 1 — Telemetry sanity (no motion)

```bash
ros2 run borunte0707a_driver joint_state_publisher
ros2 topic echo /joint_states
```

- [ ] Six joints report plausible angles (radians).
- [ ] Gently hand-move / jog a joint on the pendant and confirm the matching
      `/joint_states` value changes in the **expected direction**.

To watch the **model** track the real arm in RViz, use the dedicated launch
(driven by the driver, not the slider GUI):

```bash
ros2 launch brtirus0707a_description view_real.launch.py
```

> Do **not** use `display.launch.py` for this — it runs
> `joint_state_publisher_gui` (sliders) with no connection to the controller, so
> the model will not move when you jog the real arm.

---

## Stage 2 — Confirm calibration (no motion)

**Sign check** — open the real-arm RViz view, then jog ONE joint at a time on the
pendant:

```bash
ros2 launch brtirus0707a_description view_real.launch.py   # model tracks the arm
# optional second terminal, numeric index/scale check:
ros2 run borunte0707a_driver calibration_helper
```

- [ ] **In RViz, the model moves the SAME physical way as the arm** for every
      joint. This is the real sign check. If a joint moves the *opposite* way,
      flip that joint's entry in `borunte0707a_driver/calibration.py` and rebuild.
- [ ] (helper) Jogging pendant Jn moves the matching joint, and `scale ok`
      (|URDF Δ| == |controller Δ|). A `SCALE?` flag means a unit/gear mismatch.
- [ ] (helper) J2/J3/J5/J6 report `sign -1, URDF opposite by design` — that is
      the configured `SIGN=[+1,-1,-1,+1,-1,-1]`, **expected, not an error**.

**Offset check** — the URDF zero pose **is** the controller's mechanical home,
set by the per-joint **alignment grooves**. Seat the flat blade gauge flush in
each joint's groove (full procedure: [`docs/HOME_CALIBRATION.md`](docs/HOME_CALIBRATION.md)),
then confirm the offsets are ~0 (the defaults in `calibration.py`):

```bash
ros2 run borunte0707a_driver calibration_helper --capture-zero
```

- [ ] With every groove seated, all `axis-N` read ~0 and the implied offsets ~0.
- [ ] In `view_real.launch.py` the arm matches the URDF home pose.

> Do **not** calibrate offsets from an eyeballed home (it can be tens of degrees
> off). `kin_calibrate` is deprecated for joint zeros — see the driver README.

---

## Stage 3 — Bridge dry-run (no motion)

Watch exactly what the bridge *would* send, without sending it:

```bash
ros2 run borunte0707a_driver motion_bridge          # dry_run=true by default
# in another terminal, command the CURRENT pose (safe — zero motion):
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  '{name: [brtirus0707a_joint_1,brtirus0707a_joint_2,brtirus0707a_joint_3,brtirus0707a_joint_4,brtirus0707a_joint_5,brtirus0707a_joint_6],
    position: [0,0,0,0,0,0]}'
```

- [ ] Bridge logs `[DRY-RUN] would send AddRCC: m0=.. .. m5=..`.
- [ ] The `m0..m5` values are sane controller degrees for that target. (Command
      the current pose and they should match the live `axis-N` readings.)
- [ ] Out-of-range or large-jump commands are **rejected** with a warning.

---

## Stage 4 — MoveIt dry-run (no motion)

Bring up the full MoveIt stack against the real arm, bridge still in dry-run:

```bash
ros2 launch brtirus0707a_moveit_config real.launch.py        # dry_run:=true
```

- [ ] RViz shows the model at the arm's **actual** pose (state comes from the
      real arm via `/hw_joint_states`).
- [ ] In RViz MotionPlanning: set a **small** goal, **Plan**, then **Execute**.
- [ ] On Execute, the bridge logs `[DRY-RUN] would send AddRCC ...` — confirm the
      targets match the plan. **The arm does not move** (expected, dry-run).

---

## Stage 5 — First real motion (MOTION — operator at e-stop)

Only after Stages 1–4 are all green. Relaunch with motion enabled, lowest speed:

```bash
ros2 launch brtirus0707a_moveit_config real.launch.py dry_run:=false speed_pct:=5.0
```

- [ ] Bridge logs `LIVE MOTION` and the e-stop warning.
- [ ] Plan a **tiny single-joint** move in RViz first (a few degrees). Hand on
      the e-stop. **Execute.** The bridge logs `AddRCC path ok: …`.
- [ ] Direction and magnitude match the plan. If anything is wrong — wrong
      direction, overshoot, unexpected joint — **e-stop immediately** (and/or
      `ros2 service call /stop std_srvs/srv/Trigger`), then recheck Stage 2.
- [ ] Increase target size and `speed_pct` only gradually once confident.

> `ros2 service call /stop std_srvs/srv/Trigger` aborts from ROS at any time.
> After a stop (or Ctrl-C) the controller drops to `curMode 2` — **re-arm the
> pendant** (Start/Cycle → `curMode 7`) before the next move.

---

## If something goes wrong

| Symptom | First check |
|---------|-------------|
| `/motion_ready` false | `/robot_status`: curMode must be 7, curAlarm 0, origin 1 |
| Bridge: `rejecting … (gate): curMode=2` | left mode 7 (often after a `/stop`) — re-arm the pendant |
| Bridge: `rejecting … (gate): isMoving=1` | normal — waiting for the previous move to finish |
| Bridge: `rejecting … (soft-limit)` | target outside `SOFT_LIMITS_DEG` — intended guard; plan within limits |
| Wrong direction on a joint | `SIGN` in `calibration.py` (Stage 2) |
| Move accepted, arm still | speed too low, or alarm mid-move — re-read `curAlarm` |
| `AddRCC … FAILED` on a long path | too many waypoints — keep `path_max_points` ≤ 8 |
| `AddRCC` no reply / timeout | not in curMode 7, or packet malformed — see `reference/HC1_DEBUG_REFERENCE.md` |

Never auto-clear an alarm (`clearAlarm*`) remotely — inspect the physical cause
first (see `CLAUDE.md` → Safety Rules and the debug reference).
