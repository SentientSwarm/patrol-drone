"""ROS-free recorder core for docset 05-logging-replay (M7 — record side).

This module owns the three decisions a bag recording makes that need no ROS at runtime, so they
live on the per-PR pure-Python tier (CLAUDE.md London-TDD; ADR-0002 fast lane):

  * ``bag_name`` — the ``patrol_<missionId>_<timestamp>`` naming contract (DoD AC-1). The
    ``missionId`` segment is sanitized to a filesystem-safe token so a hostile mission id can
    neither escape the output directory nor produce an unportable filename (mirrors M6's
    checkpoint_id fs-safety).
  * ``build_record_argv`` — the ``ros2 bag record --storage mcap`` argv. MCAP, never sqlite3
    (settled constraint, plan M7 / DoD §6). Records a broad set: named topics positionally plus
    ``--regex`` patterns (``/fmu/out/.*`` absorbs PX4 v1.17's ``_v1`` topic-version churn without
    pinning exact names).
  * ``BagSidecar`` + ``build_sidecar`` + ``write_sidecar`` — the per-bag JSON metadata sidecar
    (``<bag>.meta.json``, OQ-10) that identifies and correlates a run (DoD AC-2, design §4.2.2).
    Per the dumb-producer invariant (design §3.4) the sidecar is identity/correlation metadata
    only; duration and per-topic message counts are re-derived from the bag at ingest time (M8),
    never trusted from here.

The thin launch/subprocess layer that actually spawns ``ros2 bag record`` and SIGINT-finalizes the
MCAP lives in ``launch/record.launch.py`` (a launch file), verified by colcon build + the nightly
SITL bag-producing check rather than measured here.
"""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

# Characters allowed verbatim in the mission-id segment of a bag name. Everything else collapses to
# '_' so the resulting basename is portable and can't contain a path separator.
_FS_SAFE = re.compile(r"[^A-Za-z0-9._-]+")

_BAG_TIMESTAMP_FMT = "%Y%m%d_%H%M%S"

# The shared run-id format (matches patrol_perception's run-dir token) — a single UTC instant both
# the perception run dir and the bag's mission-id segment are minted from so captures correlate to
# their bag (OQ-4). Distinct from _BAG_TIMESTAMP_FMT, which is the bag *name*'s trailing timestamp.
_RUN_ID_FMT = "%Y%m%dT%H%M%SZ"


def _sanitize_mission_id(mission_id: str) -> str:
    """Collapse fs-hostile runs to a single '_' so the id is a safe single path segment."""
    if not mission_id or not mission_id.strip():
        raise ValueError("mission_id must be a non-empty string")
    return _FS_SAFE.sub("_", mission_id.strip())


def validate_run_id(run_id: str) -> str:
    """Return ``run_id`` unchanged iff it is a safe single path segment; else raise (SWM-83).

    The run_id is forwarded into the perception capture path (``<root>/<run_id>/...``) and the bag's
    mission-id segment, so an operator-supplied token must not be able to escape the output root.
    Rejects (rather than silently rewrites) anything with a path separator, a ``..`` component, an
    absolute path, or that is empty/whitespace — a bad token is an operator error worth surfacing,
    and rewriting it would break the bag↔capture correlation the shared id guarantees.
    """
    if not run_id or not run_id.strip():
        raise ValueError("run_id must be a non-empty string")
    if run_id != run_id.strip():
        raise ValueError(f"run_id must not have leading/trailing whitespace: {run_id!r}")
    if "/" in run_id or "\\" in run_id:
        raise ValueError(f"run_id must be a single path segment (no separators): {run_id!r}")
    if run_id in (".", "..") or run_id.startswith(("./", "../")):
        raise ValueError(f"run_id must not be a relative-path component: {run_id!r}")
    return run_id


def bag_name(mission_id: str, started: datetime) -> str:
    """Return the bag basename ``patrol_<missionId>_<timestamp>`` (no extension) (DoD AC-1).

    ``timestamp`` is ``started`` rendered ``%Y%m%d_%H%M%S`` — sortable and collision-free across
    runs. ``mission_id`` is sanitized to a filesystem-safe token (see ``_sanitize_mission_id``).
    """
    return f"patrol_{_sanitize_mission_id(mission_id)}_{started.strftime(_BAG_TIMESTAMP_FMT)}"


def build_record_argv(
    *,
    output_dir: Path,
    bag_basename: str,
    topics: list[str],
    regexes: list[str],
) -> list[str]:
    """Build the ``ros2 bag record`` argv for one MCAP bag (DoD AC-2).

    Records ``topics`` after a ``--topics`` flag and each ``regexes`` entry as ``--regex <pattern>``;
    the bag is written to ``output_dir/bag_basename`` via ``-o``. The storage plugin is MCAP, never
    sqlite3. ``--topics`` is the supported form on Jazzy — passing topics positionally still works
    but is deprecated (``ros2bag`` warns), so we use the explicit flag to stay forward-compatible.

    Raises ``ValueError`` if neither a topic nor a regex is given — a recording with nothing to
    record is a configuration error, not a silent empty bag.
    """
    if not topics and not regexes:
        raise ValueError("at least one topic or regex pattern is required to record a bag")

    argv = [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "-o",
        str(output_dir / bag_basename),
    ]
    for pattern in regexes:
        argv += ["--regex", pattern]
    if topics:
        argv += ["--topics", *topics]
    return argv


@dataclass
class BagSidecar:
    """Per-bag identity/correlation metadata (``<bag>.meta.json``, OQ-10, design §4.2.2).

    Identity + correlation only. Duration and per-topic counts are intentionally absent: the M8
    ingest service re-derives those from the bag itself (dumb-producer invariant, design §3.4), so
    a buggy sidecar can never corrupt the indexed topic truth.
    """

    mission_id: str
    bag_uri: str  # bag directory patrol_<missionId>_<timestamp> (rosbag2 -o URI; .mcap nested)
    started_utc: str  # ISO-8601
    ended_utc: str  # ISO-8601
    recorded_topics: list[str]  # the named topics + regex patterns requested at record time
    mission_config_ref: str  # path/ref to the mission YAML that produced this run


@dataclass
class RecordingRun:
    """The identity of a recording, known when it *starts* (design §4.2.2).

    Groups the fields the recorder fixes at record-start — so ``build_sidecar`` takes this plus only
    the stop-time facts (``ended``, the requested topic set) instead of a long argument list. The
    launch file builds one of these at ``start`` and hands it to ``stop``.
    """

    mission_id: str
    bag_uri: str  # bag directory patrol_<missionId>_<timestamp> (rosbag2 -o URI)
    started: datetime
    mission_config_ref: str  # path/ref to the mission YAML that produced this run


def build_sidecar(run: RecordingRun, ended: datetime, recorded_topics: list[str]) -> BagSidecar:
    """Assemble a :class:`BagSidecar` from the run identity + the stop-time facts.

    Renders the timestamps as ISO-8601 strings.
    """
    return BagSidecar(
        mission_id=run.mission_id,
        bag_uri=run.bag_uri,
        started_utc=run.started.isoformat(),
        ended_utc=ended.isoformat(),
        recorded_topics=list(recorded_topics),
        mission_config_ref=run.mission_config_ref,
    )


def write_sidecar(path: Path, sidecar: BagSidecar) -> None:
    """Write ``sidecar`` to ``path`` as pretty-printed JSON (stdlib ``json``, no YAML dep)."""
    path.write_text(json.dumps(asdict(sidecar), indent=2, sort_keys=True) + "\n")


def resolve_run_id(configured: str, now: datetime) -> str:
    """Return the shared run id: ``configured`` if non-empty, else a minted UTC token (F-01).

    The composed ``mission_patrol.launch.py`` mints one id and forwards it to both the perception and
    recorder includes so captures and the bag share an identity; a standalone leaf launch passes an
    empty ``configured`` and falls back to its own ``now``-stamped token. The format matches
    perception's run-dir name (``_RUN_ID_FMT``) so the bag's mission-id segment equals the run dir.

    A non-empty ``configured`` token is operator-supplied, so it is validated here — the single
    mint/forward point — before it flows to either consumer's path (SWM-83 path-hygiene).
    """
    if configured:
        return validate_run_id(configured)
    return now.strftime(_RUN_ID_FMT)


def recorder_finished_cleanly(event: object, bag_dir: Path) -> bool:
    """True iff the recorder produced a real bag (F-03): clean exit AND ``bag_dir/metadata.yaml``.

    ROS-free so the failure-path decision sits on the Layer-A tier. ``event`` is the launch
    ``OnProcessExit`` event (a ``ProcessExited`` exposing ``returncode``); a non-zero code means
    ``ros2 bag record`` failed, and a missing ``metadata.yaml`` means rosbag2 never finalized a bag —
    either way the sidecar must not bless it. ``returncode is None`` (a non-``ProcessExited`` event
    with no rc) is treated as inconclusive-but-present and still requires the bag artifact to exist.
    """
    returncode = getattr(event, "returncode", None)
    if returncode not in (0, None):
        return False
    return (bag_dir / "metadata.yaml").exists()
