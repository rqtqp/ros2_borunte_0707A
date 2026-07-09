"""Standalone HC1 connectivity preflight (no rclpy, no motion).

Opens one RemoteMonitor connection, queries a handful of read-only status
addresses, and prints them. Use this first when debugging the network path to
the controller (e.g. after changing how the arm is reached -- gateway forward,
new subnet, WiFi hop) before starting any ROS nodes.

    ros2 run borunte0707a_driver hc1_ping
    ros2 run borunte0707a_driver hc1_ping -- --host 192.168.1.5 --repeat 10

Exit code 0 iff every query round-trip succeeded.
"""

from __future__ import annotations

import argparse
import sys
import time

from borunte0707a_driver.env_config import load_env
from borunte0707a_driver.hc1_client import HC1Client

PING_ADDRESSES = ["version", "curMode", "curAlarm", "isMoving", "origin", "axisNum"]


def main(argv=None) -> int:
    env = load_env()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default=env.get("ROBOT_IP", ""),
                        help="controller/gateway address (default: ROBOT_IP from .env)")
    parser.add_argument("--port", type=int, default=int(env.get("REMOTE_MONITOR_PORT", "9760")))
    parser.add_argument("--timeout", type=float,
                        default=float(env.get("ROBOT_REQUEST_TIMEOUT_SECONDS", "1.0")))
    parser.add_argument("--repeat", type=int, default=3,
                        help="number of query round-trips (latency sample)")
    args = parser.parse_args(argv)

    if not args.host:
        parser.error("no host: set ROBOT_IP in .env / environment or pass --host")

    print(f"HC1 preflight -> {args.host}:{args.port} (timeout {args.timeout:g}s)")
    failures = 0
    with HC1Client(args.host, args.port, args.timeout) as client:
        for n in range(1, args.repeat + 1):
            t0 = time.monotonic()
            try:
                data = client.query(PING_ADDRESSES)
            except (OSError, ValueError, RuntimeError) as error:
                failures += 1
                print(f"  [{n}/{args.repeat}] FAILED: {error}")
                continue
            latency_ms = (time.monotonic() - t0) * 1e3
            fields = " ".join(f"{k}={data.get(k)}" for k in PING_ADDRESSES)
            print(f"  [{n}/{args.repeat}] ok {latency_ms:.1f}ms  {fields}")

    if failures:
        print(f"{failures}/{args.repeat} round-trips failed")
        return 1
    print("all round-trips ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
