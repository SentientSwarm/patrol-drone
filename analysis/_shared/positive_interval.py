"""Shared argparse validators for the operator-facing numeric flags (F-05, F-03, F-04).

Both long-running daemons (upload_daemon, ingest) poll on a --poll-interval and feed it to
time.sleep(). A negative value crashes the daemon (time.sleep raises ValueError), zero busy-spins
at 100% CPU, and nan/inf slip through a bare `type=float`. Rejecting them once at parse time turns
an operator misconfig into an actionable startup error instead of a crash or a spin.

The same shape now covers the upload daemon's --transfer-timeout (a zero/negative/nan transfer cap
is as unusable as a zero poll interval) and the manifest CLI's --recent row count (where a NEGATIVE
value is worse than useless: SQLite reads a negative LIMIT as *no limit*). Keeping all three here
means one validation idiom the two daemons and the query CLI share.
"""

from __future__ import annotations

import argparse
import math


def _positive_seconds(raw: str, flag: str) -> float:
    """The shared body: a finite, strictly positive number of seconds, named for ``flag``.

    Extracted so a second time-valued flag is one wrapper rather than a copied block (the exact
    literal duplication CodeScene penalizes); ``flag`` keeps each message actionable at parse time.
    """
    value = float(raw)  # argparse turns a non-float into its own error; this handles the rest
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(
            f"{flag} must be a finite positive number of seconds, got {raw!r}"
        )
    return value


def positive_interval(raw: str) -> float:
    """argparse ``type=`` callable for ``--poll-interval``: finite, strictly positive seconds (F-05)."""
    return _positive_seconds(raw, "--poll-interval")


def positive_timeout(raw: str) -> float:
    """argparse ``type=`` callable for ``--transfer-timeout``: finite positive seconds (F-03).

    A zero/negative/nan transfer cap is as unusable as a zero poll interval — it would make every
    transfer expire instantly (or never), so it is rejected at parse time rather than at the first bag.
    """
    return _positive_seconds(raw, "--transfer-timeout")


def positive_int(raw: str) -> int:
    """argparse ``type=`` callable: an integer >= 1 — for result/row counts (F-04).

    SQLite reads a NEGATIVE ``LIMIT`` as *no limit*, so an unbounded ``--recent -1`` silently dumps
    the entire manifest (on a DGX with a long retention window, one line per bag ever recorded) and
    ``--recent 0`` returns nothing. Neither is what an operator typing a number means, so both are
    rejected here — at parse time, where the message is actionable — beside the interval validators
    this module already shares between the two daemons. Flag-agnostic by design (it is about row
    counts, not one flag); argparse prefixes ``argument --recent: `` to the message automatically.
    """
    value = int(raw)  # argparse turns a non-int into its own error; this handles the rest
    if value < 1:
        raise argparse.ArgumentTypeError(f"must be an integer >= 1, got {raw!r}")
    return value
