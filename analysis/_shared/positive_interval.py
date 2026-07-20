"""Shared argparse validator: a poll interval must be a finite, strictly positive float (F-05).

Both long-running daemons (upload_daemon, ingest) poll on a --poll-interval and feed it to
time.sleep(). A negative value crashes the daemon (time.sleep raises ValueError), zero busy-spins
at 100% CPU, and nan/inf slip through a bare `type=float`. Rejecting them once at parse time turns
an operator misconfig into an actionable startup error instead of a crash or a spin.
"""

from __future__ import annotations

import argparse
import math


def positive_interval(raw: str) -> float:
    """argparse ``type=`` callable: a finite, strictly positive number of seconds (F-05)."""
    value = float(raw)  # argparse turns a non-float into its own error; this handles the rest
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(
            f"--poll-interval must be a finite positive number of seconds, got {raw!r}"
        )
    return value
