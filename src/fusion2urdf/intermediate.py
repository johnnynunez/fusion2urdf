"""Load/save the intermediate robot description (robot.json)."""

from __future__ import annotations

import json
from pathlib import Path

from .model import Robot


def save_robot_json(robot: Robot, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(robot.to_dict(), indent=2) + "\n", encoding="utf-8")
    return path


def load_robot_json(path: str | Path) -> Robot:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    schema = data.get("schema", 1)
    if schema != 1:
        raise ValueError(f"unsupported robot.json schema version: {schema}")
    return Robot.from_dict(data)
