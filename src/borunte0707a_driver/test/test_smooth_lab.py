"""smooth_lab logic: waypoint builders, seam/pause analysis, dry-run inertness,
and the E1/E3 experiment flows against a scripted fake controller -- no
network, no rclpy, no motion."""
from types import SimpleNamespace

import pytest

from borunte0707a_driver.smooth_lab import (
    SmoothLab,
    assert_within_soft_limits,
    detect_pauses,
    max_joint_err,
    moving_gaps,
    ramp_waypoints,
    run_e1,
    run_e3,
    sweep_waypoints,
)

START = [10.0, -5.0, 15.0, 0.0, -10.0, 0.0]


# --- waypoint builders --------------------------------------------------------

def test_sweep_waypoints_shape_and_return_to_start():
    pts = sweep_waypoints(START, joint=0, amplitude_deg=8.0, cycles=2)
    assert len(pts) == 5
    assert [p[0] for p in pts] == [18.0, 2.0, 18.0, 2.0, 10.0]
    assert all(p[1:] == START[1:] for p in pts)
    assert pts[-1] == START


def test_ramp_waypoints_monotonic_to_amplitude():
    pts = ramp_waypoints(START, joint=2, amplitude_deg=8.0, steps=4)
    assert [p[2] for p in pts] == [17.0, 19.0, 21.0, 23.0]
    assert all(p[0] == START[0] for p in pts)


def test_assert_within_soft_limits_raises_with_detail():
    with pytest.raises(ValueError, match="J1"):
        assert_within_soft_limits([[500.0, 0, 0, 0, 0, 0]])
    assert_within_soft_limits([START])  # in range: no raise


def test_max_joint_err():
    assert max_joint_err([0, 0, 0, 0, 0, 3.5], [0, 0, 0, 0, 0, 1.0]) == 2.5


# --- seam analysis ------------------------------------------------------------

def _samples(moving_flags, positions=None, dt=0.1):
    positions = positions or [0.0] * len(moving_flags)
    return [
        {"t": i * dt, "isMoving": m, "axes": [p, 0, 0, 0, 0, 0]}
        for i, (m, p) in enumerate(zip(moving_flags, positions))
    ]


def test_moving_gaps_detects_seam_stop_ignores_endpoints():
    samples = _samples([0, 1, 1, 1, 0, 0, 0, 1, 1, 0])
    gaps = moving_gaps(samples, min_dur_s=0.15)
    assert len(gaps) == 1
    t_start, duration = gaps[0]
    assert t_start == pytest.approx(0.4) and duration == pytest.approx(0.3)


def test_moving_gaps_ignores_short_flap():
    assert moving_gaps(_samples([1, 0, 1]), min_dur_s=0.15) == []


def test_detect_pauses_finds_mid_path_stillness():
    positions = [0, .2, .4, .6, .8, 1, 1, 1, 1, 1, 1.2, 1.4, 1.6]
    samples = _samples([1] * len(positions), positions)
    pauses = detect_pauses(samples, joint=0, still_eps_deg=0.05, min_dur_s=0.25)
    assert len(pauses) == 1
    assert pauses[0][0] == pytest.approx(0.5) and pauses[0][1] == pytest.approx(0.4)


def test_detect_pauses_ignores_rest_before_and_after():
    positions = [0, 0, 0, .2, .4, .6, .6, .6]
    samples = _samples([1] * len(positions), positions)
    assert detect_pauses(samples, joint=0) == []


# --- scripted controller ------------------------------------------------------

class FakeTime:
    def __init__(self):
        self.t = 0.0

    def clock(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class FakeArm:
    """Controller stub: J1 follows a piecewise-linear (t, deg) profile over the
    injected fake clock; gate is always satisfied when still."""

    def __init__(self, timeline, faketime):
        self.timeline = timeline
        self.ft = faketime
        self.sent = []

    def _j1(self, t):
        pts = self.timeline
        if t <= pts[0][0]:
            return pts[0][1]
        for (t0, v0), (t1, v1) in zip(pts, pts[1:]):
            if t <= t1:
                return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
        return pts[-1][1]

    def _moving(self, t):
        return 1 if abs(self._j1(t + 0.05) - self._j1(t - 0.05)) > 1e-4 else 0

    def query(self, addresses):
        t = self.ft.t
        values = {
            "curMode": "7", "curAlarm": "0", "origin": "1",
            "isMoving": str(self._moving(t)),
        }
        for i in range(6):
            values[f"axis-{i}"] = f"{self._j1(t):.4f}" if i == 0 else "0.0000"
        return {a: values[a] for a in addresses}

    def send_addrcc(self, service_id, instructions, empty_list="1",
                    pack_id="", timeout=None):
        self.sent.append({"instructions": instructions, "empty_list": empty_list})
        return {"cmdReply": ["0", "ok"]}

    def send_command(self, cmd, *args, timeout=None):
        return {"ok": True, "reply": {"cmdReply": [cmd, "ok"]}}

    def close(self):
        pass


def make_lab(timeline, live):
    ft = FakeTime()
    arm = FakeArm(timeline, ft)
    lab = SmoothLab(arm, live=live)
    lab.clock = ft.clock
    lab.sleep = ft.sleep
    return lab, arm


# --- dry-run inertness --------------------------------------------------------

def test_dry_run_send_is_inert():
    lab, arm = make_lab([(0, 0.0), (60, 0.0)], live=False)
    assert lab.send([{"action": "4"}], "1", "test") is None
    assert arm.sent == []


def test_e1_dry_run_sends_nothing():
    lab, arm = make_lab([(0, 0.0), (60, 0.0)], live=False)
    args = SimpleNamespace(joint=1, amplitude_deg=8.0, levels=[0, 1, 3, 6, 9])
    result = run_e1(lab, args)
    assert arm.sent == []
    assert [r["smooth"] for r in result["results"]] == [0, 1, 3, 6, 9]
    assert all(r["reply"] is None for r in result["results"])


# --- E1 live (fake): smooth levels reach the wire ------------------------------

def test_e1_live_sweeps_smooth_levels():
    lab, arm = make_lab([(0, 0.0), (60, 0.0)], live=True)
    args = SimpleNamespace(joint=1, amplitude_deg=8.0, levels=[0, 9])
    run_e1(lab, args)
    assert len(arm.sent) == 2
    assert all(batch["empty_list"] == "1" for batch in arm.sent)
    assert {batch["instructions"][0]["smooth"] for batch in arm.sent} == {"0", "9"}
    assert all(len(batch["instructions"]) == 5 for batch in arm.sent)


# --- E3 live (fake): the streaming verdicts ------------------------------------

def test_e3_append_executes_and_blends():
    # J1 ramps 0->8 (path A), then blends straight back to 0 (appended B).
    lab, arm = make_lab([(0, 0.0), (0.5, 0.0), (5, 8.0), (9, 0.0), (60, 0.0)],
                        live=True)
    args = SimpleNamespace(joint=1, amplitude_deg=8.0, smooth=5)
    result = run_e3(lab, args)
    assert [b["empty_list"] for b in arm.sent] == ["1", "0"]
    assert arm.sent[1]["instructions"][0]["m0"] == "8.0000"  # B starts at A's end
    assert all(i["smooth"] == "5" for b in arm.sent for i in b["instructions"])
    assert result["b_executed"] is True
    assert result["gaps"] == [] and result["pauses"] == []
    assert "STREAMING IS REAL" in result["verdict"]


def test_e3_append_ignored_detected():
    # Arm executes A (0->8) and then just stays there: the append did nothing.
    lab, arm = make_lab([(0, 0.0), (0.5, 0.0), (5, 8.0), (60, 8.0)], live=True)
    args = SimpleNamespace(joint=1, amplitude_deg=8.0, smooth=5)
    result = run_e3(lab, args)
    assert result["b_executed"] is False
    assert "did not execute" in result["verdict"]


def test_e3_seam_pause_detected():
    # B executes but the arm stops for 2 s at the A->B seam.
    lab, arm = make_lab(
        [(0, 0.0), (0.5, 0.0), (5, 8.0), (7, 8.0), (11, 0.0), (60, 0.0)],
        live=True)
    args = SimpleNamespace(joint=1, amplitude_deg=8.0, smooth=5)
    result = run_e3(lab, args)
    assert result["b_executed"] is True
    assert result["gaps"] or result["pauses"]
    assert "paused at the A->B seam" in result["verdict"]
