"""Reuse the repo-root .env as the single source of truth for robot config.

The ROS 2 package lives several directories below the repo root, so we walk
upward to find the `.env` (falling back to `.git`). This keeps ROS parameters
defaulting to the same values the standalone scripts in `scripts/` use, instead
of duplicating IP/port settings in two places.
"""

from __future__ import annotations

from pathlib import Path


def find_repo_root(start: Path | None = None) -> Path | None:
    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        if (parent / ".env").is_file() or (parent / ".git").exists():
            return parent
    return None


def load_env() -> dict[str, str]:
    """Parse the repo-root .env into a dict. Returns {} if none is found."""
    root = find_repo_root()
    if root is None:
        return {}
    env_path = root / ".env"
    if not env_path.is_file():
        return {}

    values: dict[str, str] = {}
    for raw in env_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", maxsplit=1)
        values[key.strip()] = value.strip()
    return values
