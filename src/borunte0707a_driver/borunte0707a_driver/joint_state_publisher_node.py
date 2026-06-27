"""Phase 1 (MINIMUM): read-only /joint_states publisher.

Polls the HC1 RemoteMonitor `query` service for joint angle, speed, and torque
and republishes as sensor_msgs/JointState. Contains NO motion capability.

Units (see ROS2_INTEGRATION_PLAN.md):
  position: axis-n degrees      -> radians
  velocity: curSpeed-n RPM      -> rad/s
  effort:   curTorque-n (2580=1x rated) -> x-rated multiplier
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from borunte0707a_driver.calibration import SIGN, OFFSET_RAD
from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client

NUM_JOINTS = 6
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
TORQUE_RATED_SCALE = 2580.0


def _to_float(value) -> float | None:
    try:
        if value in (None, "", "null"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("borunte0707a_joint_state_publisher")
        env = load_env()

        self.declare_parameter("robot_ip", env.get("ROBOT_IP", ""))
        self.declare_parameter("remote_monitor_port", int(env.get("REMOTE_MONITOR_PORT", "9760")))
        self.declare_parameter("timeout", float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "3.0")))
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter(
            "joint_names", [f"brtirus0707a_joint_{i + 1}" for i in range(NUM_JOINTS)]
        )

        robot_ip = self.get_parameter("robot_ip").value
        if not robot_ip:
            raise RuntimeError(
                "robot_ip is not set. Provide it via .env (ROBOT_IP) or "
                "--ros-args -p robot_ip:=<addr>"
            )

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.client = HC1Client(
            host=robot_ip,
            port=int(self.get_parameter("remote_monitor_port").value),
            timeout=float(self.get_parameter("timeout").value),
        )
        self.addresses = (
            [f"axis-{i}" for i in range(NUM_JOINTS)]
            + [f"curSpeed-{i}" for i in range(NUM_JOINTS)]
            + [f"curTorque-{i}" for i in range(NUM_JOINTS)]
        )
        self._warned_aux = False

        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"Publishing /joint_states from {robot_ip}:"
            f"{self.client.port} at {rate:g} Hz"
        )

    def tick(self) -> None:
        try:
            data = self.client.query(self.addresses)
        except (OSError, ValueError, RuntimeError) as error:
            self.get_logger().warn(f"query failed: {error}", throttle_duration_sec=5.0)
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions, velocities, efforts = [], [], []
        aux_missing = False
        for i in range(NUM_JOINTS):
            deg = _to_float(data.get(f"axis-{i}"))
            rpm = _to_float(data.get(f"curSpeed-{i}"))
            trq = _to_float(data.get(f"curTorque-{i}"))
            # Apply the URDF sign/offset so /joint_states matches the model
            # frame (joints 2/3/5/6 are flipped). velocity follows the same
            # sign; effort is unsigned magnitude (x-rated torque).
            if deg is not None:
                positions.append(SIGN[i] * math.radians(deg) + OFFSET_RAD[i])
            else:
                positions.append(math.nan)
            velocities.append(SIGN[i] * rpm * RPM_TO_RAD_S if rpm is not None else 0.0)
            efforts.append(trq / TORQUE_RATED_SCALE if trq is not None else 0.0)
            if rpm is None or trq is None:
                aux_missing = True

        if aux_missing and not self._warned_aux:
            self.get_logger().warn(
                "curSpeed/curTorque unavailable on this controller; publishing "
                "position only (velocity/effort = 0). See ROS2_INTEGRATION_PLAN.md."
            )
            self._warned_aux = True

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = JointStatePublisher()
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
