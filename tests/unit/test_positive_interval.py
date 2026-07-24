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
from ingest.positive_interval import positive_int as ingest_positive_int
from ingest.positive_interval import positive_interval as ingest_positive_interval

from _shared.positive_interval import positive_int, positive_interval, positive_timeout
from upload_daemon.__main__ import main as upload_main

_INVALID = ["-1", "0", "-0.5", "nan", "inf", "-inf"]

# Both time-valued flags share one body (`_positive_seconds`), so they share one table here too
# rather than a copied pair of test functions (CodeScene duplication).
_SECONDS_VALIDATORS = [positive_interval, positive_timeout]


@pytest.mark.parametrize(
    "validator", _SECONDS_VALIDATORS, ids=["poll-interval", "transfer-timeout"]
)
@pytest.mark.parametrize("raw", _INVALID)
def test_positive_seconds_rejects_non_positive_or_non_finite(validator, raw: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        validator(raw)


@pytest.mark.parametrize(
    "validator", _SECONDS_VALIDATORS, ids=["poll-interval", "transfer-timeout"]
)
@pytest.mark.parametrize("raw", ["5", "5.0", "0.25", "120"])
def test_positive_seconds_accepts_finite_positive(validator, raw: str) -> None:
    assert validator(raw) == float(raw)


# F-04: `--recent` row counts must be >= 1 — SQLite reads a NEGATIVE LIMIT as *no limit* (so -1 would
# dump the whole manifest) and 0 returns nothing. Both are rejected at parse time.
@pytest.mark.parametrize("raw", ["-1", "0", "-10"])
def test_positive_int_rejects_below_one(raw: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        positive_int(raw)


@pytest.mark.parametrize("raw", ["1", "5", "100"])
def test_positive_int_accepts_one_and_above(raw: str) -> None:
    assert positive_int(raw) == int(raw)


def test_ingest_shim_is_the_shared_validator() -> None:
    # The ingest container reaches the validators via `ingest.positive_interval`, a thin re-export of
    # `_shared.positive_interval` (mirrors `ingest.bounded_seen`); pin both to the canonical functions.
    assert ingest_positive_interval is positive_interval
    assert ingest_positive_int is positive_int


# End-to-end: a bad time-valued flag aborts the upload daemon at parse time (argparse exits non-zero)
# BEFORE the watch loop — no time.sleep to reach, so nothing needs mocking. Both flags share the
# table: `--poll-interval` (F-05) and `--transfer-timeout` (F-03, review 4752923085).
@pytest.mark.parametrize("flag", ["--poll-interval", "--transfer-timeout"])
@pytest.mark.parametrize("raw", ["-1", "0", "nan"])
def test_upload_main_rejects_bad_seconds_flag(tmp_path: Path, flag: str, raw: str) -> None:
    with pytest.raises(SystemExit):
        upload_main(["--watch", str(tmp_path), "--target", "dgx:/data/bags/", flag, raw])


@pytest.mark.parametrize("raw", ["-1", "0", "nan"])
def test_ingest_main_rejects_bad_poll_interval(tmp_path: Path, raw: str) -> None:
    with pytest.raises(SystemExit):
        ingest_main(
            ["--watch", str(tmp_path), "--db", str(tmp_path / "m.db"), "--poll-interval", raw]
        )
