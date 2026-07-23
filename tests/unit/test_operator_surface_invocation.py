"""Layer-A smoke tests: every documented M8 operator command is actually runnable as written.

Companion to ``test_upload_main_invocation.py``, extended to the two entry points that import
``ingest``. The ``_shared`` extraction moved ``bag_layout`` / ``bounded_seen`` / ``positive_interval``
under ``analysis/_shared/``, and ``docker/ingest/`` now re-exports from there. The container image
``COPY``s both trees onto ``/opt/ingest`` so the ``from _shared...`` imports resolve inside it — but
nothing proved the HOST-side invocations still worked, and both had silently broken:

* ``tests/replay/verify_live_bag.py`` self-bootstrapped only ``tests/replay`` + ``docker``, so the
  LR-8 / AC-8 acceptance witness died with ``ModuleNotFoundError: No module named '_shared'``
  before argparse ran — the tool that certifies the milestone could not start on a clean checkout.
* ``analysis/e2e_check.md`` documented ``PYTHONPATH=docker python3 -m ingest[.manifest_query]``,
  one path short, so both documented stand-in commands failed the same way.

Each case spawns a FRESH subprocess with an explicit environment — never the ambient pytest
``pythonpath`` — so these prove the exact documented forms run, and cannot silently rot again.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_INGEST_PATH = f"{_REPO_ROOT / 'analysis'}:{_REPO_ROOT / 'docker'}"

# (id, argv, PYTHONPATH, a substring argparse only prints once the module actually imported)
_DOCUMENTED_COMMANDS = [
    # The witness takes NO PYTHONPATH on purpose: it self-bootstraps sys.path, and the whole point
    # of the fix is that `python3 tests/replay/verify_live_bag.py` works with a bare environment.
    pytest.param(
        [str(_REPO_ROOT / "tests" / "replay" / "verify_live_bag.py"), "--help"],
        "",
        "--bag",
        id="verify_live_bag_self_bootstraps",
    ),
    pytest.param(
        ["-m", "ingest", "--help"],
        _INGEST_PATH,
        "--watch",
        id="e2e_check_ingest_watch",
    ),
    pytest.param(
        ["-m", "ingest.manifest_query", "--help"],
        _INGEST_PATH,
        "--recent",
        id="e2e_check_manifest_query",
    ),
]


@pytest.mark.parametrize(("argv", "pythonpath", "expected_flag"), _DOCUMENTED_COMMANDS)
def test_documented_command_is_runnable(
    argv: list[str], pythonpath: str, expected_flag: str
) -> None:
    result = subprocess.run(
        [sys.executable, *argv],
        cwd=_REPO_ROOT,
        env={"PYTHONPATH": pythonpath},
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )

    # A ModuleNotFoundError exits non-zero and prints nothing to stdout; argparse help means the
    # module imported cleanly and reached its parser.
    assert result.returncode == 0, result.stderr
    assert expected_flag in result.stdout
