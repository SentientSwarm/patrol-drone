"""Layer-A unit tests for the shared --poll-interval validator (M8 / F-05, PR #16).

Both long-running daemons feed --poll-interval to time.sleep(): a negative value crashes it
(ValueError), zero busy-spins at 100% CPU, and nan/inf slip through a bare `type=float`. The shared
`_shared.positive_interval` validator rejects all of them at argparse-parse time with a non-zero
exit, so an operator misconfig is an actionable startup error, not a crash/spin. These tests cover
the validator directly and prove both daemons wire it into `--poll-interval` (aborting before the
watch loop). The ingest side imports it via the `ingest.positive_interval` re-export shim, which the
container COPYs alongside `_shared/` — so this also locks the shim to the canonical implementation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest
from ingest.__main__ import main as ingest_main
from ingest.positive_interval import positive_interval as ingest_positive_interval

from _shared.positive_interval import positive_interval
from upload_daemon.__main__ import main as upload_main

_INVALID = ["-1", "0", "-0.5", "nan", "inf", "-inf"]


@pytest.mark.parametrize("raw", _INVALID)
def test_positive_interval_rejects_non_positive_or_non_finite(raw: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        positive_interval(raw)


@pytest.mark.parametrize("raw", ["5", "5.0", "0.25", "120"])
def test_positive_interval_accepts_finite_positive(raw: str) -> None:
    assert positive_interval(raw) == float(raw)


def test_ingest_shim_is_the_shared_validator() -> None:
    # The ingest container reaches the validator via `ingest.positive_interval`, a thin re-export of
    # `_shared.positive_interval` (mirrors `ingest.bounded_seen`); pin it to the canonical function.
    assert ingest_positive_interval is positive_interval


# End-to-end: `--poll-interval <bad>` aborts each daemon at parse time (argparse exits non-zero)
# BEFORE the watch loop — no time.sleep to reach, so nothing needs mocking.
@pytest.mark.parametrize("raw", ["-1", "0", "nan"])
def test_upload_main_rejects_bad_poll_interval(tmp_path: Path, raw: str) -> None:
    with pytest.raises(SystemExit):
        upload_main(
            ["--watch", str(tmp_path), "--target", "dgx:/data/bags/", "--poll-interval", raw]
        )


@pytest.mark.parametrize("raw", ["-1", "0", "nan"])
def test_ingest_main_rejects_bad_poll_interval(tmp_path: Path, raw: str) -> None:
    with pytest.raises(SystemExit):
        ingest_main(
            ["--watch", str(tmp_path), "--db", str(tmp_path / "m.db"), "--poll-interval", raw]
        )
