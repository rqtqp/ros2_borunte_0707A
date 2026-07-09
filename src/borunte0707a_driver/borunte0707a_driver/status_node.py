"""Phase 2: read-only controller status + diagnostics.

Polls the HC1 RemoteMonitor `query` service for the controller's health/mode
fields and republishes them three ways:

  /robot_status   std_msgs/String              human-readable JSON snapshot
  /motion_ready   std_msgs/Bool                True iff the AddRCC safety gate
                                                (CLAUDE.md -> Safety Rules) is met
  /diagnostics    diagnostic_msgs/DiagnosticArray  rqt_robot_monitor-friendly

Contains NO motion capability -- `reqType: query` only, safe anytime. The
`/motion_ready` gate published here is exactly the precondition set Phase 3's
topic bridge must re-check immediately before sending any `AddRCC`.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client

# curMode codes (reference/HC1_DEBUG_REFERENCE.md). Mode 7 is the only mode
# that accepts AddRCC on F5.2.1; 2 is Auto-idle (accepted by the gate but
# rejected by this firmware in practice -- the bridge still checks for 7).
CUR_MODE_NAMES = {
    "0": "None",
    "1": "Manual",
    "2": "Automatic (idle)",
    "3": "Stop",
    "7": "Auto-running (remote-cmd wait)",
    "8": "Step-by-step",
    "9": "Single loop",
}
# Modes from which remote motion can be commanded (firmware requires 7; 2 is
# accepted by the documented gate but not by this controller -- kept for parity).
MOTION_MODES = {"2", "7"}

# Read-only addresses to poll each tick.
STATUS_ADDRESSES = [
    "version",
    "curMode",
    "curAlarm",
    "isMoving",
    "origin",
    "axisNum",
    "RemoteCmdLen",
]


def _is(value, expected: str) -> bool:
    return value is not None and str(value).strip() == expected


class StatusNode(Node):
    def __init__(self):
        super().__init__("borunte0707a_status_node")
        env = load_env()

        self.declare_parameter("robot_ip", env.get("ROBOT_IP", ""))
        self.declare_parameter("remote_monitor_port", int(env.get("REMOTE_MONITOR_PORT", "9760")))
        self.declare_parameter("timeout", float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
        self.declare_parameter("publish_rate_hz", 2.0)

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

        self.status_pub = self.create_publisher(String, "robot_status", 10)
        self.ready_pub = self.create_publisher(Bool, "motion_ready", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)

        self._last_summary: str | None = None
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"Polling controller status from {robot_ip}:{self.client.port} at {rate:g} Hz"
        )

    def tick(self) -> None:
        try:
            data = self.client.query(STATUS_ADDRESSES)
        except (OSError, ValueError, RuntimeError) as error:
            self.get_logger().warn(f"status query failed: {error}", throttle_duration_sec=5.0)
            self._publish_diag(
                DiagnosticStatus.ERROR, "controller unreachable", {"error": str(error)}
            )
            self.ready_pub.publish(Bool(data=False))
            return

        mode = data.get("curMode")
        alarm = data.get("curAlarm")
        moving = data.get("isMoving")
        origin = data.get("origin")

        motion_ready = (
            str(mode).strip() in MOTION_MODES
            and _is(alarm, "0")
            and _is(moving, "0")
            and _is(origin, "1")
        )

        # /robot_status -- full JSON snapshot, augmented with a decoded mode name.
        snapshot = dict(data)
        snapshot["curModeName"] = CUR_MODE_NAMES.get(str(mode).strip(), "unknown")
        snapshot["motion_ready"] = motion_ready
        self.status_pub.publish(String(data=json.dumps(snapshot)))

        # /motion_ready -- the AddRCC safety gate as a single boolean.
        self.ready_pub.publish(Bool(data=motion_ready))

        # /diagnostics -- level reflects severity; alarm is the hard error.
        if not _is(alarm, "0"):
            level, summary = DiagnosticStatus.ERROR, f"alarm {alarm} active"
        elif not _is(origin, "1"):
            level, summary = DiagnosticStatus.WARN, "origin not established"
        elif not motion_ready:
            level, summary = (
                DiagnosticStatus.WARN,
                f"not motion-ready (mode {mode} "
                f"{CUR_MODE_NAMES.get(str(mode).strip(), '?')})",
            )
        else:
            level, summary = DiagnosticStatus.OK, "ready"
        self._publish_diag(level, summary, snapshot)

        if summary != self._last_summary:
            self.get_logger().info(f"status: {summary}")
            self._last_summary = summary

    def _publish_diag(self, level, summary: str, values: dict) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "borunte0707a: controller"
        status.message = summary
        status.hardware_id = self.get_parameter("robot_ip").value
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = StatusNode()
    except RuntimeError as error:
        print(f"startup failed: {error}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
