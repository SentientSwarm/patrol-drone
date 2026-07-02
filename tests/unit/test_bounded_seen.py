"""Layer-A unit test for the watch-loops' bounded 'already-seen' set (M8 / F-09).

The upload + ingest watch loops track handled bags in a membership set. An unbounded set grows one
entry per bag for the process lifetime; _BoundedSeen caps it with LRU eviction. Eviction is safe
because both handlers are idempotent (rsync -a / INSERT OR REPLACE), so a re-seen evicted bag is a
cheap redundant re-handle. This pins membership + the cap + re-add-after-eviction.

The class is defined identically in both watch-loop shells (analysis/upload_daemon/__main__.py and
docker/ingest/__main__.py); this exercises the upload copy — they are byte-identical by design.
"""

from __future__ import annotations

from pathlib import Path

from upload_daemon.__main__ import _BoundedSeen


def test_bounded_seen_tracks_membership() -> None:
    seen = _BoundedSeen(maxlen=8)
    bag = Path("/w/patrol_a_20260629_120000")
    assert bag not in seen
    seen.add(bag)
    assert bag in seen


def test_bounded_seen_evicts_oldest_when_over_cap() -> None:
    seen = _BoundedSeen(maxlen=2)
    a, b, c = (Path(f"/w/patrol_{x}_20260629_120000") for x in "abc")
    seen.add(a)
    seen.add(b)
    seen.add(c)  # over cap → evict the oldest-added (a)
    assert a not in seen
    assert b in seen
    assert c in seen
    assert len(seen) == 2


def test_bounded_seen_allows_readd_after_eviction() -> None:
    # An evicted bag reappearing in the watch dir is re-handled once more (idempotent) — it must be
    # addable again, not permanently poisoned.
    seen = _BoundedSeen(maxlen=1)
    a, b = Path("/w/patrol_a_20260629_120000"), Path("/w/patrol_b_20260629_120000")
    seen.add(a)
    seen.add(b)  # evicts a
    assert a not in seen
    seen.add(a)  # re-added fine
    assert a in seen
