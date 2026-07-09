"""Reuse the repo-root .env as the single source of truth for robot config.

The ROS 2 package lives several directories below the repo root, so we walk
upward to find the `.env` (falling back to `.git`). This keeps ROS parameters
defaulting to the same values everywhere instead of duplicating IP/port
settings in multiple places.

OS environment variables override the `.env` file for the known keys, so a
deployment without a checkout (e.g. the driver running on a gateway host next
to the controller) can be configured with plain exports:

    ROBOT_IP=192.168.1.5 ros2 run borunte0707a_driver joint_state_publisher
"""

from __future__ import annotations

import os
from pathlib import Path

# Keys the driver reads; OS environment values take precedence over .env.
ENV_KEYS = (
    "ROBOT_IP",
    "REMOTE_MONITOR_PORT",
    "ROBOT_REQUEST_TIMEOUT_SECONDS",
    "HC1_REMOTE_COMMAND_SERVICE_ID",
)


def find_repo_root(start: Path | None = None) -> Path | None:
    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        if (parent / ".env").is_file() or (parent / ".git").exists():
            return parent
    return None


def load_env() -> dict[str, str]:
    """Config from the repo-root .env, overlaid with OS environment variables.

    Returns {} if neither source provides anything."""
    values: dict[str, str] = {}

    root = find_repo_root()
    if root is not None and (root / ".env").is_file():
        for raw in (root / ".env").read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", maxsplit=1)
            values[key.strip()] = value.strip()

    for key in ENV_KEYS:
        if key in os.environ:
            values[key] = os.environ[key]
    return values
