"""Layer-A unit tests for the shared bag-layout validator (M8 / F-01, PR #16).

`_shared.bag_layout.is_valid_bag_dir` is the ONE predicate both pipeline boundaries use to decide
"is this a real bag" — the upload daemon's `is_complete` and the ingest service's
`_require_finalized_bag`. Before it, three places keyed on ``metadata.yaml`` alone, so a directory
holding a plausible marker (plus a sidecar) but NO nested ``.mcap`` shipped and then indexed as a
fully-populated manifest row for an unreplayable artifact (Mira High, review 4752923085).

A valid bag dir is therefore a real directory carrying BOTH the finalize marker AND at least one real
``.mcap`` payload, with symlinks refused at every component. These tests pin that truth table
directly; the two boundaries' own tests prove each delegates to it.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from ingest.bag_layout import is_regular_file as ingest_is_regular_file
from ingest.bag_layout import is_valid_bag_dir as ingest_is_valid_bag_dir

from _shared.bag_layout import (
    has_finalize_marker,
    has_mcap_payload,
    is_regular_file,
    is_valid_bag_dir,
)

_BAG_NAME = "patrol_x_20260629_120000"


def _build_bag(tmp_path: Path, *, marker: str, payload: str) -> Path:
    """A bag dir whose marker and payload each take one of: real / symlink / none.

    One builder for the whole truth table so the cases are a parametrize row rather than a copied
    block per shape (CodeScene duplication). Symlink targets live OUTSIDE the bag dir, which is
    exactly the redirect the symlink-strict posture exists to refuse.
    """
    bag = tmp_path / _BAG_NAME
    bag.mkdir()
    _place(bag / "metadata.yaml", tmp_path / "elsewhere-metadata.yaml", marker, b"rosbag2:\n")
    _place(bag / f"{_BAG_NAME}_0.mcap", tmp_path / "elsewhere.mcap", payload, b"\x89MCAP0\r\n")
    return bag


def _place(path: Path, redirect: Path, kind: str, body: bytes) -> None:
    """Create ``path`` as a real file, a symlink to ``redirect``, or not at all."""
    if kind == "real":
        path.write_bytes(body)
    elif kind == "symlink":
        redirect.write_bytes(body)
        path.symlink_to(redirect)


# The F-01 truth table. Only a real marker AND a real payload is a valid bag; every other shape is
# refused. The `marker-only` row is the review's headline case (a metadata-only dir that used to pass
# every gate); `symlinked-payload` is the second shape it names.
@pytest.mark.parametrize(
    ("marker", "payload", "expected"),
    [
        ("real", "real", True),
        ("real", "none", False),
        ("real", "symlink", False),
        ("none", "real", False),
        ("symlink", "real", False),
        ("none", "none", False),
    ],
    ids=[
        "valid",
        "marker-only",
        "symlinked-payload",
        "payload-only",
        "symlinked-marker",
        "empty-dir",
    ],
)
def test_is_valid_bag_dir_truth_table(
    tmp_path: Path, marker: str, payload: str, expected: bool
) -> None:
    bag = _build_bag(tmp_path, marker=marker, payload=payload)

    assert is_valid_bag_dir(bag) is expected


# A symlinked BAG DIR is refused even when it points at a genuinely valid bag — following it would
# redirect the rsync source (and the ingest read) outside the watched/landing tree.
def test_symlinked_bag_dir_is_not_valid(tmp_path: Path) -> None:
    real_root = tmp_path / "elsewhere"
    real_root.mkdir()
    real_bag = _build_bag(real_root, marker="real", payload="real")
    watch_dir = tmp_path / "watch"
    watch_dir.mkdir()
    link = watch_dir / real_bag.name
    link.symlink_to(real_bag)

    assert is_valid_bag_dir(real_bag) is True  # the target itself is fine
    assert is_valid_bag_dir(link) is False  # reached through a symlink, it is not


# Never raises on a path that isn't there: a bag dir the daemon polls for may not exist yet, and a
# missing path is simply "not a valid bag" rather than a crash in the watch loop.
@pytest.mark.parametrize("kind", ["absent", "plain-file"])
def test_non_directory_is_not_valid_and_does_not_raise(tmp_path: Path, kind: str) -> None:
    path = tmp_path / "not_a_bag"
    if kind == "plain-file":
        path.write_text("i am a file")

    assert is_valid_bag_dir(path) is False


# The component predicates are independently meaningful (each boundary's error message cites one).
def test_component_predicates_split_marker_from_payload(tmp_path: Path) -> None:
    bag = _build_bag(tmp_path, marker="real", payload="none")

    assert has_finalize_marker(bag) is True
    assert has_mcap_payload(bag) is False


# is_regular_file is the shared symlink-strict primitive both boundaries use for their sidecar reads.
@pytest.mark.parametrize(
    ("kind", "expected"), [("real", True), ("symlink", False), ("none", False)]
)
def test_is_regular_file_refuses_symlinks_and_absences(
    tmp_path: Path, kind: str, expected: bool
) -> None:
    path = tmp_path / "sidecar.meta.json"
    _place(path, tmp_path / "redirect.meta.json", kind, b"{}")

    assert is_regular_file(path) is expected


def test_ingest_shim_is_the_shared_validator() -> None:
    # The ingest container reaches the validator via `ingest.bag_layout`, a thin re-export of
    # `_shared.bag_layout` (mirrors `ingest.bounded_seen` / `ingest.positive_interval`). Pin the shim
    # to the canonical functions so the two boundaries can never drift onto different predicates —
    # which is the entire point of F-01.
    assert ingest_is_valid_bag_dir is is_valid_bag_dir
    assert ingest_is_regular_file is is_regular_file
