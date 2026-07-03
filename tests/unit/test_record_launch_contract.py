"""Layer-A boundary probes for the M7 record→correlate contract (docset 05-logging-replay).

The blocking Hermes review flagged three contracts that live at the launch/integration seam the
ROS-free unit tier had been blind to — yet the *decisions* behind them are pure path/identity logic
and testable here without a live SITL (the report's own point in F-05):

  * **Shared run id (F-01).** ``resolve_run_id`` is the pure half of the launch wiring: a configured
    id passes through, an empty one mints a UTC token. ``mission_patrol.launch.py`` forwards the same
    resolved value to both 04 (perception ``run_id``) and 05 (recorder ``mission_id``) — the
    launch-substitution *wiring* is verified by ``colcon build`` + the SITL re-run (the recorder is a
    launch file, not measured here; ADR-0002 tiering), but the id *decision* is pinned below.
  * **Sidecar ↔ artifact layout (F-02).** The sidecar's ``bag_uri`` is the rosbag2 ``-o`` directory
    (``<output_dir>/<bag_uri>/`` holding ``metadata.yaml`` + ``<bag_uri>_0.mcap``), never a top-level
    ``.mcap``. We assert the URI resolves to the real on-disk directory against a fake bag dir.
  * **Failure → no sidecar (F-03).** ``recorder_finished_cleanly`` decides whether a real bag was
    produced; a non-zero exit or a missing ``metadata.yaml`` must return ``False`` so the launch
    skips the success sidecar.
  * **Staging survives a launch death (F-03++).** The launch writes ``<bag>.sidecar-inputs.json`` at
    record-start; the runner finalizes ``<bag>.meta.json`` from it after the bag finalizes, so a
    group-SIGTERM that kills ``ros2 launch`` before its OnProcessExit handler no longer loses the
    sidecar. We pin that the staged/runner path produces the SAME identity fields as the handler path.

These probes are pure-core (``import patrol_logging.recorder`` only, on the unit pythonpath), so they
run always-on in the bare uv env without a sourced ROS.
"""

from __future__ import annotations

import json
from pathlib import Path

from patrol_logging.recorder import (
    RecordingRun,
    build_sidecar,
    finalize_sidecar_from_staging,
    recorder_finished_cleanly,
    resolve_run_id,
    sidecar_inputs_path,
    write_sidecar_inputs,
)

# Reuse the deterministic instants from the recorder suite — one shared source, no copy-paste.
# Flat module name (tests/unit is the pytest rootdir; there is no tests.unit package) so mypy and
# pytest resolve test_recorder the same way and don't see it under two module names.
from test_recorder import _ENDED, _STARTED, _TS

_RUN_ID = "20260626T140509Z"  # _STARTED rendered in the shared run-id format (%Y%m%dT%H%M%SZ)


class _ProcessExited:
    """Minimal stand-in for launch's OnProcessExit event (exposes ``returncode``)."""

    def __init__(self, returncode: int | None) -> None:
        self.returncode = returncode


# --- F-01: shared run id (the pure resolve_run_id decision) -----------------------------------


def test_resolve_run_id_passes_a_configured_id_through() -> None:
    # An operator-supplied / launch-forwarded id is used verbatim so both includes share it.
    assert resolve_run_id("op-set-id", _STARTED) == "op-set-id"


def test_resolve_run_id_mints_a_utc_token_when_empty() -> None:
    # Empty -> a minted token in perception's run-dir format (so the bag mission-id segment matches).
    assert resolve_run_id("", _STARTED) == _RUN_ID


# --- F-02: sidecar bag_uri resolves to the real on-disk bag directory (pure path) -------------


def test_sidecar_bag_uri_points_at_the_on_disk_bag_directory(tmp_path: Path) -> None:
    bag_uri = f"patrol_alpha_{_TS}"
    # rosbag2's -o layout: a DIRECTORY holding metadata.yaml + the nested <uri>_0.mcap storage file.
    bag_dir = tmp_path / bag_uri
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("version: 9\n")
    (bag_dir / f"{bag_uri}_0.mcap").write_bytes(b"")

    run = RecordingRun(
        mission_id="alpha",
        bag_uri=bag_uri,
        started=_STARTED,
        mission_config_ref="/abs/patrol_mission.yaml",
    )
    sidecar = build_sidecar(run, _ENDED, ["/tf"])

    # The recorded URI is the directory that actually exists; no top-level <uri>.mcap is implied.
    assert (tmp_path / sidecar.bag_uri).is_dir()
    assert not (tmp_path / f"{sidecar.bag_uri}.mcap").exists()


# --- F-03: failure -> no success sidecar (the pure recorder_finished_cleanly decision) --------


def _bag_dir_with_metadata(tmp_path: Path) -> Path:
    bag_dir = tmp_path / f"patrol_alpha_{_TS}"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("version: 9\n")
    return bag_dir


def test_recorder_finished_cleanly_true_on_clean_exit_with_metadata(tmp_path: Path) -> None:
    assert recorder_finished_cleanly(_ProcessExited(0), _bag_dir_with_metadata(tmp_path)) is True


def test_recorder_finished_cleanly_false_on_nonzero_exit(tmp_path: Path) -> None:
    # A failed `ros2 bag record` must not bless a sidecar even if a partial dir exists.
    assert recorder_finished_cleanly(_ProcessExited(1), _bag_dir_with_metadata(tmp_path)) is False


def test_recorder_finished_cleanly_false_when_metadata_missing(tmp_path: Path) -> None:
    # Clean exit but no finalized bag (no metadata.yaml) -> rosbag2 never wrote a bag -> no sidecar.
    empty_bag_dir = tmp_path / f"patrol_alpha_{_TS}"
    empty_bag_dir.mkdir()
    assert recorder_finished_cleanly(_ProcessExited(0), empty_bag_dir) is False


# --- F-03++: the staged/runner path matches the OnProcessExit handler path --------------------


def test_staged_finalize_produces_the_same_identity_as_the_handler_path(tmp_path: Path) -> None:
    # A group-SIGTERM can kill `ros2 launch` before its OnProcessExit handler runs. The runner then
    # finalizes the sidecar from the launch's record-start staging file. This must yield the SAME
    # identity fields the handler's build_sidecar(run, ...) path would have — only ended_utc (stamped
    # at finalize) legitimately differs, so we compare every field except that one.
    run = RecordingRun(
        mission_id="alpha",
        bag_uri=f"patrol_alpha_{_TS}",
        started=_STARTED,
        mission_config_ref="/abs/patrol_mission.yaml",
    )
    topics = ["/patrol/mission_state", "/tf"]
    bag_dir = tmp_path / run.bag_uri
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("version: 9\n")
    write_sidecar_inputs(sidecar_inputs_path(bag_dir), run, topics)  # launch, at record-start

    sidecar_path = finalize_sidecar_from_staging(bag_dir)  # runner, after the launch is gone

    handler_sidecar = build_sidecar(run, _ENDED, topics)  # the OnProcessExit path, same run
    loaded = json.loads(sidecar_path.read_text())
    for field in ("mission_id", "bag_uri", "started_utc", "recorded_topics", "mission_config_ref"):
        assert loaded[field] == getattr(handler_sidecar, field)
