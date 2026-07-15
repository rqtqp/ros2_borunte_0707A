# HC1 smooth motion — AddRCC protocol notes, experiments, findings

Tracking doc for the smooth-motion investigation: why externally-commanded
motion is segmented, what the vendor revealed about the AddRCC schema, the
experiment ladder (E1–E5) with the tooling to run it, and a findings log.
Originates from
`../ros2_borunte_0707A_sensors_and_vision/docs/arm/hc1_smooth_motion_upstream_plan.md`
(2026-07); this file is the upstream (arm-repo) home for the results.

## Problem

Externally-commanded motion visibly decelerates to a full stop at every AddRCC
batch boundary. Root causes in the current bridge design:

1. The bridge requires `isMoving == 0` (live gate query) before EVERY send —
   consecutive commands can never blend.
2. Long paths are chunked (`path_max_points`, default 8); each chunk is a
   separate gated AddRCC → stop at each chunk seam (`chunk_path=true`).
3. `smooth` was sent as `1` and assumed boolean — it is a 0–9 level.
4. Only motion type `action:"4"` (free path, joint space) is used.

## Vendor information (Borunte support reply, 2026-07-15)

AddRCC instruction schema knowledge, verbatim from the reply:

- **`emptyList: "0" | "1"`** — *"Should the remote list be cleared?"* AddRCC
  maintains a **persistent instruction list**; `1` clears before adding, `0`
  APPENDS. The driver has always effectively replaced. If the controller
  accepts appends **while executing**, this is a streaming primitive: feed the
  queue faster than it drains and the arm never stops.
- **`smooth: "0".."9"`** — a smoothness LEVEL, not a flag. Higher levels
  presumably blend more aggressively through waypoints. Exact semantics
  (blend radius? look-ahead?) unconfirmed — E1 measures it.
- **Motion types:** `action 4` = Free Path (what we use), `action 10` =
  Posture Line (Cartesian straight line), `action 17` = Posture Curve
  (spline; extra fields `m0_p..m7_p` = "coordinates of the end position of
  the curve"). Curves could natively execute smooth Cartesian arcs.
- **`ckStatus`** — axis mask, one bit per axis; `0x3F` = first 6 axes.
- **`delay`** — per-instruction delay, 0.1 s resolution.
- **`oneshot: "0" | "1"`** — 1 = execute once, 0 = "execute continuously"
  (loop semantics — a looping instruction list would repeat motion
  indefinitely; the driver and lab always send `"1"`).
- **`packID`** — echoed in the reply; required for a reply at all.
- Vendor: *"This instruction set can contain multiple position coordinates,
  and the robot can move continuously."* (confirms within-batch continuity —
  matches what we see live: multi-point AddRCC batches run without stops.)

Still unanswered (follow-up email pending, see bottom): max list length +
overflow behavior; append-while-moving legality; blending across the append
boundary; smooth-level semantics; whether action-17 segments blend.

## Tooling

- **`smooth_lab`** (`ros2 run borunte0707a_driver smooth_lab …`) — standalone
  experiment harness (no rclpy/MoveIt): subcommands `e1`, `e2`, `e3`, `raw`.
  Dry-run by default; `--live` (+ confirmation) to move. One reused HC1
  connection; caps speed at 20 % (default 10 %); logs every AddRCC reply
  verbatim; writes a JSON run log per invocation; sends `actionStop` on
  Ctrl+C/anomaly. **Stop the motion bridge / RViz stack first** — the
  controller's socket pool is tiny.
- **`path_smooth`** bridge parameter — the AddRCC `smooth` level 0–9 used for
  path sends (this is the investigation plan's `smooth_level`). Now
  runtime-settable, e.g.
  `ros2 param set /borunte0707a_motion_bridge path_smooth 3`.

## Experiment ladder (each gates the next)

**Safety preamble for EVERY live step:** operator at the e-stop; workcell
clear; preflight SAFE (`hc1_ping` / `arm_health.sh`); speed ≤ 10 %; start each
experiment dry-run; change ONE variable at a time; never open a second TCP
connection while another client holds the controller; after any anomaly send
stop and re-verify state. Treat `emptyList=0` and `oneshot=0` as potentially
dangerous until proven otherwise — append semantics could REORDER or REPEAT
motion.

Run on JTSN inside the arm container, bridge stopped:

```bash
# E1 — smooth level sweep (lowest risk): same 5-point J1 ±8° path at 0,1,3,6,9
ros2 run borunte0707a_driver smooth_lab e1                    # dry-run first
ros2 run borunte0707a_driver smooth_lab e1 -- --live
# outcome: duration / terminal error / pause count per level -> pick a default,
# set it as the bridge's path_smooth default.

# E2 — append while idle: emptyList bookkeeping without concurrency
ros2 run borunte0707a_driver smooth_lab e2 -- --live
# checks: does an appended (emptyList=0) batch auto-execute at rest? once, in
# order? does emptyList=1 truly clear leftovers?

# E3 — append while MOVING (the streaming test)
ros2 run borunte0707a_driver smooth_lab e3 -- --live --speed 5
# slow ramp A (emptyList=1), then mid-motion continuation B (emptyList=0)
# starting at A's endpoint. Prints a verdict: rejected / executed-with-seam-
# pause / blended (STREAMING IS REAL -> unlocks E5).

# E4 — Cartesian primitives (action 10, then 17): operator-supplied JSON,
# pendant nearby to compare displayed coordinates; tiny amplitudes.
ros2 run borunte0707a_driver smooth_lab raw -- --instruction \
  '{"oneshot":"1","action":"10","ckStatus":"0x3F","speed":"5.0","delay":"0.0","tool":"0","coord":"1","smooth":"0","m0":"...","m1":"...","m2":"...","m3":"...","m4":"...","m5":"...","m6":"0.0","m7":"0.0"}'
# (frames/units of m0..m7 for action 10/17 are exactly what E4 establishes —
# no builder is provided on purpose; --live only after the dry-run JSON is
# confirmed against the pendant.)
```

### E5 — streaming mode in the bridge (IMPLEMENTED 2026-07-15)

`stream_path` bridge mode (default **off**; `stream_path:=true` on
`real.launch.py` or `ros2 param set /borunte0707a_motion_bridge stream_path
true`): the settled MoveIt path is chunked exactly like `chunk_path`
(≤`path_max_points`, overlapping by one waypoint), the first chunk is sent
through the full safety gate with `emptyList=1`, and the rest are **appended**
(`emptyList=0`) while the arm is moving. Faithful AND smooth — needs
`path_smooth>0` to blend the seams (E3).

Flow control (no `RemoteCmdLen` — list capacity is vendor follow-up #1, so
stay conservative): at most `stream_inflight` (default 2) chunks are on the
controller beyond nothing — i.e. the executing chunk plus one queued.
Consumption is detected positionally: chunks overlap by one waypoint, so when
the arm passes within `stream_advance_deg` (default 3°) of an appended chunk's
first waypoint, the previous chunk is done. Fallback: if `isMoving` drops to 0
with chunks still queued (boundary missed, e.g. heavy corner-cutting), the
in-flight estimate resets and the next chunk is appended immediately — worst
case degrades to `chunk_path`'s stop-and-go, never a stall. The `isMoving`
gate still guards the FIRST chunk of every path (a NEW goal); appends belong
to that accepted goal. `/stop` clears the queue + stream state; a failed or
rejected append aborts the rest of the path and is never auto-resent.
Completion feedback confirms/corrects the final goal as usual.

## Findings log

Record every result here, including negative ones (e.g. "appends while moving
are rejected: reply X") — a negative result closes a branch and is as valuable
as a positive one. Run logs live in `reference/smooth_lab_runs/` (local-only)
and on JTSN in `~/borunte_arm/labruns/`.

| Date | Exp | Result | Run log |
|------|-----|--------|---------|
| 2026-07-15 | E1 | **All smooth levels 0–9 accepted** (5-pt J1 ±8° sweep @10 %). Durations 32.6 / 31.7 / 31.5 / 31.3 / 31.2 s for smooth 0/1/3/6/9; terminal error 0.001° at *every* level. Only `smooth=0` produced detectable stops (2 × ~0.25 s at direction reversals). Blending saturates early: level 1 already removes the stops; higher levels shave tenths of a second. → keep bridge default `path_smooth=1` (closest waypoint tracking that still blends). | `smooth_lab_e1_20260715_030551.json` |
| 2026-07-15 | E2 | **Append-at-rest bookkeeping is sane.** `emptyList=0` batch sent while idle auto-executes immediately (no separate trigger), exactly once, correct endpoint; a following `emptyList=1` batch produced exactly one burst — clearing truly clears, no leftovers replayed. All replies `["AddRCC","ok"]` in 16–20 ms. | `smooth_lab_e2_20260715_030710.json` |
| 2026-07-15 | E3 | **PASS — STREAMING IS REAL.** `emptyList=0` sent while `isMoving=1` is accepted (`ok`, 20 ms) and executed in order. First run used `smooth=0` and showed ~0.33 s stops at all waypoints *including* the seam — a smooth=0 artifact that masks the seam (hence `--smooth`). Re-run with `smooth=5` on A and B: **no seam pause, no stillness anywhere** — the controller blends across the appended boundary mid-execution. E5 unlocked. | `smooth_lab_e3_20260715_030806.json` (smooth=0), `..._033543.json` (smooth=5) |
| 2026-07-15 | E5 | **stream_path live-validated end-to-end** (MoveIt `plan_exec` home↔[J1 30°, J6 80°] @5 %, `stream_path:=true`, smooth=1): 17 waypoints → 3 chunks; `open 1/3` gated then `append 2/3` at +0.2 s and `append 3/3` at +20 s (boundary crossing), replies 22–26 ms; completion feedback closed both moves at **max err 0.00°**. Independent 100 Hz `/joint_states` capture (25 918 samples): **zero pauses ≥ 0.25 s inside either move** — the only stillness in the 101.5 s window is the 6.6 s between the two moves. Chunk-seam stops are gone. | `js_stream_e5_validation_20260715.json` |
| —    | E4  | *pending — needs a pendant-side session to establish action 10/17 Cartesian frames/units* | |

Known so far (pre-experiments, live-validated during phases 3–5):

- Multi-point batches (≤8 pts) execute continuously within the batch;
  1–7 points ack in ~20 ms; 10–16 points never reply; ≥20 points reset the
  connection (hence `path_max_points=8`).
- `smooth=1` on path sends was accepted everywhere; levels >1 untested.
- Every batch so far was sent with `emptyList="1"` (replace) after an
  `isMoving==0` gate — the segmentation we're attacking.

## Follow-up questions for Borunte / HC-System (email pending)

1. Maximum instruction-list length per AddRCC and on the controller's list;
   behavior on overflow (reject / block / silently drop?).
2. Is `AddRCC` with `emptyList:"0"` while the robot is executing supported?
   Does the controller blend across the appended boundary?
3. Exact semantics of `smooth` levels 0–9 (blend radius? look-ahead?).
4. Do consecutive `action:"17"` curve segments blend into each other?
5. Official RemoteMonitor/HCRemoteCommand specification + error code list;
   recommended connection management for port 9760's socket pool.
