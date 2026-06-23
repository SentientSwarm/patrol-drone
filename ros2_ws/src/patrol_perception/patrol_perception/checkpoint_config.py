"""03 checkpoint-config loader (M6.A, T A.2 — design §4.2.5).

Parses 03's canonical ``sim/config/checkpoints.yaml`` (settled-default schema, OQ-5) into a
``{tag_id: CheckpointEntry}`` map consumed by :class:`CheckpointResolver`. 04 reads the
``tag_id <-> checkpoint_id`` relation only; it does NOT author or fork the map. The loader is
**fail-fast** (design §4.4.5 config row): a missing, malformed, or schema-violating source
raises rather than letting the node run blind on a broken map.

ROS-free by construction — only stdlib + PyYAML, no rclpy/message imports (AC-5).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_REQUIRED_AXES = ("x", "y", "z")


class CheckpointConfigError(ValueError):
    """Raised when checkpoints.yaml is missing, malformed, or violates the schema."""


@dataclass(frozen=True)
class CheckpointEntry:
    """One checkpoint row: the tag_id -> checkpoint_id relation plus its world/ENU position."""

    checkpoint_id: str
    tag_id: int
    tag_family: str
    position: tuple[float, float, float]  # x, y, z in world/ENU meters


def _require_mapping(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CheckpointConfigError(f"{ctx} must be a mapping, got {type(value).__name__}")
    return value


def _require_str(row: dict[str, Any], key: str, ctx: str) -> str:
    value = row.get(key)
    if not isinstance(value, str) or not value:
        raise CheckpointConfigError(f"{ctx}: '{key}' must be a non-empty string")
    return value


def _require_int(row: dict[str, Any], key: str, ctx: str) -> int:
    value = row.get(key)
    # bool is an int subclass in Python; a YAML `true` is not a valid tag_id.
    if not isinstance(value, int) or isinstance(value, bool):
        raise CheckpointConfigError(f"{ctx}: '{key}' must be an integer")
    return value


def _require_number(mapping: dict[str, Any], key: str, ctx: str) -> float:
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise CheckpointConfigError(f"{ctx}: '{key}' must be numeric")
    return float(value)


def _parse_position(row: dict[str, Any], ctx: str) -> tuple[float, float, float]:
    pos = _require_mapping(row.get("position"), f"{ctx} position")
    x, y, z = (_require_number(pos, axis, f"{ctx} position") for axis in _REQUIRED_AXES)
    return (x, y, z)


def _parse_row(row: Any, index: int) -> CheckpointEntry:
    ctx = f"checkpoint #{index}"
    mapping = _require_mapping(row, ctx)
    return CheckpointEntry(
        checkpoint_id=_require_str(mapping, "checkpoint_id", ctx),
        tag_id=_require_int(mapping, "tag_id", ctx),
        tag_family=_require_str(mapping, "tag_family", ctx),
        position=_parse_position(mapping, ctx),
    )


class CheckpointConfigLoader:
    """Loads + validates 03's checkpoints.yaml into a tag_id-keyed map (design §4.2.5)."""

    def load(self, path: str) -> dict[int, CheckpointEntry]:
        rows = self._checkpoint_rows(self._read(path))
        result: dict[int, CheckpointEntry] = {}
        for index, row in enumerate(rows):
            entry = _parse_row(row, index)
            if entry.tag_id in result:
                raise CheckpointConfigError(
                    f"duplicate tag_id {entry.tag_id} at checkpoint #{index} "
                    f"(already mapped to '{result[entry.tag_id].checkpoint_id}')"
                )
            result[entry.tag_id] = entry
        return result

    def _read(self, path: str) -> Any:
        source = Path(path)
        if not source.is_file():
            raise CheckpointConfigError(f"checkpoints config not found: {path}")
        try:
            return yaml.safe_load(source.read_text())
        except yaml.YAMLError as exc:
            raise CheckpointConfigError(
                f"checkpoints config is not valid YAML: {path}: {exc}"
            ) from exc

    def _checkpoint_rows(self, raw: Any) -> list[Any]:
        top = _require_mapping(raw, "checkpoints config top level")
        rows = top.get("checkpoints")
        if not isinstance(rows, list) or not rows:
            raise CheckpointConfigError(
                "checkpoints config must have a non-empty 'checkpoints' list"
            )
        return rows
