"""motion_bridge logic: chunking, downsampling, target extraction, dedupe,
and the completion-feedback state machine. Uses a real rclpy node (never
spun) with the HC1 client mocked -- no controller, no motion."""
import math
import os

import pytest

os.environ.setdefault("ROBOT_IP", "192.0.2.1")  # never contacted in tests

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

from borunte0707a_driver import motion_bridge_node
from borunte0707a_driver.motion_bridge_node import (
    MotionBridge, build_free_path_instruction,
)

NAMES = [f"brtirus0707a_joint_{i}" for i in range(1, 7)]


class FakeClient:
    def __init__(self):
        self.sent = []
        self.reply = {"cmdReply": ["x", "ok"]}

    def send_addrcc(self, service_id, instructions, timeout=None):
        self.sent.append(instructions)
        return self.reply

    def query(self, addresses):
        raise AssertionError("tests must mock _check_gate/_current_axis_deg")

    def close(self):
        pass


@pytest.fixture()
def node():
    rclpy.init()
    n = MotionBridge()
    n.client = FakeClient()
    n.stop_client = FakeClient()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def js(positions, names=NAMES):
    msg = JointState()
    msg.name = list(names)
    msg.position = [float(p) for p in positions]
    return msg


# --- build_free_path_instruction -------------------------------------------

def test_instruction_shape():
    instr = build_free_path_instruction([1, -2, 3, -4, 5, -6], 7.5)
    assert instr["action"] == "4" and instr["ckStatus"] == "0x3F"
    assert instr["speed"] == "7.5" and instr["m6"] == "0.0" and instr["m7"] == "0.0"
    assert instr["m1"] == "-2.0000"


# --- _extract_targets --------------------------------------------------------

def test_extract_targets_reorders_by_name(node):
    shuffled = list(reversed(NAMES))
    got = node._extract_targets(js([6, 5, 4, 3, 2, 1], shuffled))
    assert got == [1, 2, 3, 4, 5, 6]


def test_extract_targets_missing_joint_rejected(node):
    with pytest.raises(ValueError):
        node._extract_targets(js([1, 2, 3, 4, 5], NAMES[:5]))


def test_extract_targets_positional_fallback(node):
    got = node._extract_targets(js([1, 2, 3, 4, 5, 6], names=[]))
    assert got == [1, 2, 3, 4, 5, 6]


# --- chunking ----------------------------------------------------------------

def test_chunkify_overlap_invariant(node):
    points = [[float(i)] * 6 for i in range(20)]
    chunks = node._chunkify(points, 8)
    assert all(len(c) <= 8 for c in chunks)
    for a, b in zip(chunks, chunks[1:]):
        assert a[-1] == b[0]          # continuous: next chunk starts where prev ended
    flat = chunks[0] + [p for c in chunks[1:] for p in c[1:]]
    assert flat == points             # nothing lost, nothing duplicated


def test_chunkify_short_path_single_chunk(node):
    points = [[0.0] * 6, [1.0] * 6]
    assert node._chunkify(points, 8) == [points]


# --- path downsampling (via dry-run _process_path) ---------------------------

def test_downsample_caps_points_keeps_endpoints(node, monkeypatch):
    node._check_gate = lambda: True
    built = []
    monkeypatch.setattr(
        motion_bridge_node, "build_free_path_instruction",
        lambda w, s, smooth="0", oneshot="1": built.append(list(w)) or {},
    )
    node._path_waypoints = [[float(i), 0, 0, 0, 0, 0] for i in range(30)]
    goal = [40.0, 0, 0, 0, 0, 0]
    node._process_path(goal)
    assert 2 <= len(built) <= node.path_max_points
    assert built[0][0] == 0.0 and built[-1] == goal


def test_path_rejected_outside_soft_limits(node):
    node._check_gate = lambda: True
    node._path_waypoints = []
    node._process_path([500.0, 0, 0, 0, 0, 0])
    assert node._last_sent_deg is None       # refused, nothing marked sent


# --- dedupe ------------------------------------------------------------------

def test_dedupe_uses_correction_tolerance(node):
    node._check_gate = lambda: True
    node._last_sent_deg = [10.0, 0, 0, 0, 0, 0]
    node._path_waypoints = []
    nearly_same = [10.0 + node.correction_tol_deg / 2, 0, 0, 0, 0, 0]
    node._process_path(nearly_same)
    assert node._last_sent_deg == [10.0, 0, 0, 0, 0, 0]   # deduped, not re-sent


# --- completion feedback -----------------------------------------------------

HELD = [35.0, 10.0, -20.0, 0.0, 15.0, 0.0]
SHORT = [34.7, 10.6, -20.4, -0.2, 15.4, -0.6]


def arm_completion(node, goal):
    node.dry_run = False
    node._begin_completion(goal)
    assert node._completing is not None


def test_completion_waits_while_moving(node):
    arm_completion(node, SHORT)
    node._pending_axis_deg = list(HELD)
    node._check_gate = lambda: "isMoving=1"
    assert node._handle_completion() is False
    assert not node.client.sent


def test_completion_corrects_to_held_endpoint_once(node):
    arm_completion(node, SHORT)
    node._pending_axis_deg = list(HELD)
    node._check_gate = lambda: True
    node._current_axis_deg = lambda: [34.36, 9.86, -19.47, 0.18, 14.59, -0.56]
    assert node._handle_completion() is False
    assert len(node.client.sent) == 1
    assert node.client.sent[0][0]["m0"] == "35.0000"      # exact held endpoint
    assert node._completing["corrected"] is True
    # arm arrives -> confirmed, no second correction
    node._current_axis_deg = lambda: list(HELD)
    assert node._handle_completion() is True
    assert node._completing is None and len(node.client.sent) == 1
    assert node._last_sent_deg == HELD


def test_completion_within_tolerance_no_correction(node):
    arm_completion(node, HELD)
    node._pending_axis_deg = list(HELD)
    node._check_gate = lambda: True
    node._current_axis_deg = lambda: [35.05, 10.0, -20.0, 0.0, 15.0, 0.0]
    assert node._handle_completion() is True
    assert not node.client.sent and node._completing is None


def test_completion_abandoned_by_new_trajectory(node):
    arm_completion(node, HELD)
    node._pending_axis_deg = [20.0, 10.0, -20.0, 0.0, 15.0, 0.0]  # 15 deg away
    assert node._handle_completion() is True
    assert node._completing is None and not node.client.sent


def test_completion_timeout_gives_up(node):
    arm_completion(node, HELD)
    node._pending_axis_deg = list(HELD)
    node._completing["since"] = node.get_clock().now() - Duration(
        seconds=node.completion_timeout + 1
    )
    assert node._handle_completion() is True
    assert node._completing is None


def test_dry_run_never_arms_completion(node):
    node.dry_run = True
    node._begin_completion(HELD)
    assert node._completing is None


# --- runtime speed_pct -------------------------------------------------------

def test_speed_pct_settable_at_runtime(node):
    from rclpy.parameter import Parameter
    results = node.set_parameters([Parameter("speed_pct", value=12.5)])
    assert results[0].successful and node.speed_pct == 12.5


def test_speed_pct_out_of_range_rejected(node):
    from rclpy.parameter import Parameter
    before = node.speed_pct
    results = node.set_parameters([Parameter("speed_pct", value=0.0)])
    assert not results[0].successful and node.speed_pct == before
    results = node.set_parameters([Parameter("speed_pct", value=150.0)])
    assert not results[0].successful and node.speed_pct == before
