"""Layer-A smoke test: the documented ``python -m upload_daemon`` invocation is actually runnable.

`analysis/upload_daemon/__main__.py` documents ``PYTHONPATH=analysis python -m upload_daemon …`` as
the operator command (F-07). Because ``upload_daemon`` lives under ``analysis/``, a bare
``python -m upload_daemon`` from the repo root fails with ``No module named upload_daemon`` unless
``analysis`` is on the path. This test spawns a FRESH subprocess with an explicit
``PYTHONPATH=analysis`` and ``cwd=repo_root`` — NOT the ambient pytest ``pythonpath`` — so it proves
the exact documented raw ``python -m`` form works and can't silently rot.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]


# F-07: the documented `PYTHONPATH=analysis python -m upload_daemon --help` runs from the repo root
# (exit 0) and prints the argparse usage — the advertised operator command is runnable as written.
def test_documented_python_m_invocation_runs() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "upload_daemon", "--help"],
        cwd=_REPO_ROOT,
        env={"PYTHONPATH": str(_REPO_ROOT / "analysis")},
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--watch" in result.stdout  # argparse help rendered → the module ran, not ImportError
