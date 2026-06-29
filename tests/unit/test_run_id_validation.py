"""Layer-A unit tests for run_id path-hygiene (docset 05-logging-replay, M8 / T8.11, SWM-83).

Carried from M7 PR #14 (Hermes Low, deferred to M8): an operator-supplied ``run_id`` is forwarded
into the perception capture path and the bag mission-id without filesystem-segment validation, so a
``run_id`` of ``../escape`` or ``/abs/path`` resolves *outside* the configured output root (path
traversal). M8 is the right home because its upload/ingest pipeline is where ``run_id`` could begin
flowing from a less-trusted source.

The fix validates the token once at the shared mint/forward point (``recorder.resolve_run_id`` — both
the perception run dir and the bag mission-id segment are minted from it) and adds a defense-in-depth
reject in ``CaptureWriter`` (the actual path-join site). A token with path separators, ``..``, or an
absolute path is rejected; the safe default minted token still resolves (no regression).

Covers TS-21 (reject malicious at resolve_run_id), TS-22 (CaptureWriter can't escape root),
TS-23 (valid/default token still works).
"""

from __future__ import annotations

from datetime import UTC, datetime

import pytest
from patrol_logging.recorder import resolve_run_id, validate_run_id
from patrol_perception.capture_writer import CaptureWriter

_NOW = datetime(2026, 6, 26, 14, 5, 9, tzinfo=UTC)

_MALICIOUS = [
    "../escape",
    "..",
    "a/b",
    "a\\b",
    "/abs/path",
    "/etc/passwd",
    "foo/../bar",
    ".",
    "",
    "   ",
]


# TS-21: validate_run_id rejects every path-hostile token (separators, .., absolute, empty).
@pytest.mark.parametrize("bad", _MALICIOUS)
def test_validate_run_id_rejects_malicious(bad: str) -> None:
    with pytest.raises(ValueError, match="run_id"):
        validate_run_id(bad)


# TS-23: a valid single-segment token passes validation unchanged.
@pytest.mark.parametrize("good", ["run1", "20260626T140509Z", "patrol-7", "mission_2"])
def test_validate_run_id_accepts_valid(good: str) -> None:
    assert validate_run_id(good) == good


# TS-21: resolve_run_id rejects a configured malicious token (the shared mint/forward point).
@pytest.mark.parametrize("bad", ["../escape", "/abs/path", "a/b"])
def test_resolve_run_id_rejects_malicious_configured(bad: str) -> None:
    with pytest.raises(ValueError, match="run_id"):
        resolve_run_id(bad, _NOW)


# TS-23: resolve_run_id passes a valid configured token through, and mints a safe default when empty.
def test_resolve_run_id_valid_and_default() -> None:
    assert resolve_run_id("run1", _NOW) == "run1"
    minted = resolve_run_id("", _NOW)
    assert minted == "20260626T140509Z"  # _RUN_ID_FMT of _NOW — a safe single segment
    assert validate_run_id(minted) == minted  # the minted default is itself valid


# TS-22: CaptureWriter rejects a malicious run_id so its run dir can never escape output_root.
def test_capture_writer_rejects_escaping_run_id(tmp_path) -> None:
    with pytest.raises(ValueError, match="run_id"):
        CaptureWriter(output_root=str(tmp_path), run_id="../escape")


# TS-22: with a valid run_id, the run dir stays under the configured root.
def test_capture_writer_run_dir_under_root(tmp_path) -> None:

    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    # The resolved run dir must be a child of output_root (no traversal).
    assert tmp_path.resolve() in writer.run_dir.resolve().parents or (
        writer.run_dir.resolve().parent == tmp_path.resolve()
    )
