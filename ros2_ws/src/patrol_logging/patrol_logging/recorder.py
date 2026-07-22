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
  * ``write_sidecar_inputs`` / ``read_sidecar_inputs`` / ``finalize_sidecar_from_staging`` — the
    caller-independent sidecar path. The launch stages the record-start facts to
    ``<bag>.sidecar-inputs.json`` while it is alive; the runner finalizes ``<bag>.meta.json`` from
    that staging file after the bag finalizes, so a group-SIGTERM that kills ``ros2 launch`` before
    its OnProcessExit handler runs no longer loses the sidecar (M8 ingestability, dumb producer).

The thin launch/subprocess layer that actually spawns ``ros2 bag record`` and SIGINT-finalizes the
MCAP lives in ``launch/record.launch.py`` (a launch file), verified by colcon build + the nightly
SITL bag-producing check rather than measured here.
"""

from __future__ import annotations

import json
import os
import re
import secrets
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
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


def run_id_rejection(run_id: str) -> str | None:
    """Return why ``run_id`` is an unsafe path segment, or None if it is safe (SWM-83).

    A run_id is forwarded into the perception capture path (``<root>/<run_id>/...``) and the bag's
    mission-id segment, so it must be a single, separator-free, non-relative, non-empty segment that
    cannot escape the output root. Returns the rejection reason as a string (the caller decides
    whether to raise) — a data table of (predicate, reason) keeps the check flat and reusable across
    the recorder and ``CaptureWriter`` (which can't import this module — separate colcon packages).
    """
    checks = (
        (not run_id or not run_id.strip(), "must be a non-empty string"),
        (run_id != run_id.strip(), "must not have leading/trailing whitespace"),
        ("/" in run_id or "\\" in run_id, "must be a single path segment (no separators)"),
        (run_id in (".", ".."), "must not be a relative-path component"),
    )
    return next((reason for failed, reason in checks if failed), None)


def validate_run_id(run_id: str) -> str:
    """Return ``run_id`` unchanged iff it is a safe single path segment; else raise (SWM-83).

    Rejects (rather than silently rewrites) a path-hostile token — a bad token is an operator error
    worth surfacing, and rewriting it would break the bag↔capture correlation the shared id grants.
    """
    reason = run_id_rejection(run_id)
    if reason is not None:
        raise ValueError(f"run_id {reason}: {run_id!r}")
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


def _fsync_dir(directory: Path) -> None:
    """Best-effort fsync of ``directory`` so a rename into it is durable across a crash.

    ``os.replace`` makes the swap atomic, but the new directory ENTRY is only persisted once the
    parent directory itself is fsynced. Without this, a crash right after the rename can lose the
    ``<bag>.meta.json`` entry — the uploader then never sees the sidecar and skips the finalized bag
    forever (F-03). Directory fsync is unsupported on some platforms/filesystems (Windows has no
    directory fd; a few filesystems reject it), so a raised ``OSError`` is swallowed: the durability
    UPGRADE must never turn into a new failure path for the write it is hardening.
    """
    try:
        dir_fd = os.open(directory, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(dir_fd)
    except OSError:
        pass
    finally:
        os.close(dir_fd)


def _atomic_write_text(path: Path, text: str) -> None:
    """Publish ``text`` to ``path`` atomically: write a same-dir temp file, fsync, then os.replace.

    The upload daemon keys "bag complete" on ``<bag>.meta.json`` merely existing and, once it ships
    the file, writes a durable ``<bag>.uploaded`` marker that suppresses every later re-send. A
    truncate-then-stream write (``Path.write_text``) exposes a window where a poll can observe — and
    rsync — a partial/empty file, then mark the bag uploaded forever (F-01). ``os.replace`` swaps the
    fully-written temp file onto the final path in a single atomic rename (POSIX + Windows), so an
    observer only ever sees the old file or the complete new one, never a torn intermediate. The temp
    file is a same-directory sibling (``os.replace`` is atomic only within one filesystem; the sidecar
    and its temp are always siblings of the bag dir, so this holds). ``fsync`` before the rename makes
    the bytes durable so a crash can't leave the renamed-in file empty. On any failure the temp file
    is removed so a ``.tmp`` crumb never masquerades as a real artifact.
    """
    # Exclusive, no-follow temp create (F-03, guide §1.B): a predictable `.<pid>.tmp` sibling opened
    # with plain open() follows a symlink a local process can pre-plant in a writable output dir,
    # redirecting the write (and the subsequent os.replace inode) to an attacker target. An
    # unpredictable suffix + O_CREAT|O_EXCL|O_NOFOLLOW means a pre-existing file OR symlink at the temp
    # path fails the open loudly rather than being reused, while the same-directory placement keeps
    # os.replace atomic. mode 0o600 so the crumb is owner-only if a crash leaves it.
    tmp = path.with_name(f"{path.name}.{secrets.token_hex(8)}.tmp")
    # The exclusive create sits OUTSIDE the cleanup try: if it fails (something already occupies the
    # temp path), we created nothing and must remove nothing — unlinking there would delete a FOREIGN
    # planted object we never owned. Cleanup below only ever unlinks a temp this call created.
    fd = os.open(tmp, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        _fsync_dir(path.parent)
    except BaseException:  # clean the temp on any interrupt, then re-raise
        tmp.unlink(missing_ok=True)
        raise


def write_sidecar(path: Path, sidecar: BagSidecar) -> None:
    """Write ``sidecar`` to ``path`` as pretty-printed JSON, published atomically (F-01)."""
    _atomic_write_text(path, json.dumps(asdict(sidecar), indent=2, sort_keys=True) + "\n")


# Suffix of the record-start staging file that lets the sidecar survive a launch death (F-03++).
# The launch writes ``<bag>.sidecar-inputs.json`` while it is unquestionably alive; the runner reads
# it back and writes the real ``<bag>.meta.json`` after the bag finalizes, so a group-SIGTERM that
# kills ``ros2 launch`` before its OnProcessExit handler runs no longer loses the sidecar.
_SIDECAR_INPUTS_SUFFIX = ".sidecar-inputs.json"


def sidecar_inputs_path(bag_dir: Path) -> Path:
    """Return the staging-file path for a bag directory (``<bag>.sidecar-inputs.json``)."""
    return bag_dir.with_name(bag_dir.name + _SIDECAR_INPUTS_SUFFIX)


def write_sidecar_inputs(path: Path, run: RecordingRun, recorded_topics: list[str]) -> None:
    """Persist the record-START sidecar facts (everything ``build_sidecar`` needs except ``ended``).

    Written by the launch the instant recording starts, while the launch is guaranteed alive — so
    the runner can finalize the sidecar later regardless of how the launch shut down. ``started`` is
    stored as ISO-8601 so the finalized ``started_utc`` matches the OnProcessExit path byte-for-byte.
    """
    payload = {
        "mission_id": run.mission_id,
        "bag_uri": run.bag_uri,
        "started_utc": run.started.isoformat(),
        "mission_config_ref": run.mission_config_ref,
        "recorded_topics": list(recorded_topics),
    }
    _atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")


def read_sidecar_inputs(path: Path) -> tuple[RecordingRun, list[str]]:
    """Inverse of :func:`write_sidecar_inputs`: reconstruct the run identity + requested topic set."""
    payload = json.loads(path.read_text())
    run = RecordingRun(
        mission_id=payload["mission_id"],
        bag_uri=payload["bag_uri"],
        started=datetime.fromisoformat(payload["started_utc"]),
        mission_config_ref=payload["mission_config_ref"],
    )
    return run, list(payload["recorded_topics"])


def _is_regular_file(path: Path) -> bool:
    """A real file at ``path``, not a symlink (mirrors the uploader's predicate, F-02).

    The finalize boundary must agree with the uploader (``_shared.bag_layout.is_regular_file``): a
    symlinked ``metadata.yaml`` / staging / sidecar is NOT a finalizable artifact, so a planted
    symlink can neither drive a spurious "finalized" nor be written through. Following-``exists()``
    let a symlink pass the recorder while the uploader rejected the same bag — the two halves
    disagreeing about one artifact (F-02, the twin of the uploader hardening H-04).
    """
    return path.is_file() and not path.is_symlink()


def finalize_sidecar_from_staging(bag_dir: Path) -> Path | None:
    """Write ``<bag>.meta.json`` from the staging file once the bag has finalized (caller-independent).

    Idempotent and safe to call from the runner after *any* shutdown: returns ``None`` (a no-op) when
    there is nothing to finalize — no finalized bag (``metadata.yaml`` absent), no staging file, or a
    sidecar already present (the OnProcessExit happy path beat us to it). On success it writes the
    sidecar (``ended`` stamped now), removes the staging file, and returns the sidecar path.
    """
    if not _is_regular_file(bag_dir / "metadata.yaml"):
        return None
    inputs = sidecar_inputs_path(bag_dir)
    if not _is_regular_file(inputs):
        return None
    sidecar_path = bag_dir.with_name(bag_dir.name + ".meta.json")
    if _is_regular_file(sidecar_path):
        inputs.unlink()  # the handler already finalized; just clear the staging crumb
        return None
    run, recorded_topics = read_sidecar_inputs(inputs)
    write_sidecar(sidecar_path, build_sidecar(run, datetime.now(UTC), recorded_topics))
    inputs.unlink()
    return sidecar_path


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
