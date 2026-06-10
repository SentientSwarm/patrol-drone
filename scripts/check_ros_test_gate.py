#!/usr/bin/env python3
"""Fail when first-party ROS packages grow a test surface while ROS CI still skips tests.

Hermes Medium #2: `.github/workflows/ros-ci.yml` runs `colcon build` with `skip-tests: 'true'`
as an M2 BUILD gate — no `colcon test` runs. That is correct while the only tests in the workspace
are the vendored upstream's ament-lint stubs (px4_ros_com's cpplint/copyright noise), but it is a
future-activation footgun: the day a real first-party patrol_* test lands, it would silently NOT run.

This guard is the tripwire. It fails CI iff BOTH hold:
  1. ros-ci.yml still has `skip-tests: 'true'`, AND
  2. a FIRST-PARTY package (under ros2_ws/src but NOT ros2_ws/src/external/) has a test surface.

So it stays green today (no first-party tests yet) and self-resolves the moment a maintainer flips
`skip-tests` to `'false'` — no stale guard to remember to delete. It deliberately ignores the
vendored external/ subtrees, whose lint failures are not this project's surface.

Exit 0 = clean; exit 1 = first-party tests exist but ROS CI would skip them, one line per package.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
ROS_SRC = Path("ros2_ws/src")
EXTERNAL = ROS_SRC / "external"
WORKFLOW = Path(".github/workflows/ros-ci.yml")

# A test file: ament/colcon test entry points (Python test_*.py / *_test.py) or a gtest/.cpp test.
_TEST_FILE = re.compile(r"(^test_.*\.py$|_test\.py$|^test_.*\.(cpp|cc|cxx)$|_test\.(cpp|cc|cxx)$)")


def _is_test_file(path: Path) -> bool:
    """A runnable test entry point: a file whose name matches the ament/colcon test convention."""
    return path.is_file() and bool(_TEST_FILE.search(path.name))


def _has_test_surface(pkg_dir: Path) -> bool:
    """True if the package ships runnable tests (any file under a test/ or tests/ dir, or in-pkg)."""
    # Direct children beside package.xml (some ament_cmake layouts) must still match the
    # test-filename convention, so a stray top-level source file is not a false positive.
    if any(_is_test_file(c) for c in pkg_dir.glob("*")):
        return True
    # Inside a conventional test/ or tests/ dir, treat ANY file as a test surface regardless of
    # name: ament/CMake registers test sources like tests/state_machine.cpp that match no
    # test_*/*_test convention, and the filename-only check missed them (Hermes High #1). While
    # skip-tests is on we err toward tripping the guard rather than letting such a test land unrun.
    for sub in ("test", "tests"):
        test_dir = pkg_dir / sub
        if test_dir.is_dir() and any(c.is_file() for c in test_dir.rglob("*")):
            return True
    return False


def first_party_packages_with_tests(repo_root: Path) -> list[str]:
    """First-party package dirs (a package.xml outside external/) that carry a test surface."""
    src = repo_root / ROS_SRC
    if not src.is_dir():
        return []
    external = repo_root / EXTERNAL
    found = []
    for pkg_xml in sorted(src.rglob("package.xml")):
        pkg_dir = pkg_xml.parent
        if external in pkg_dir.parents or pkg_dir == external:
            continue  # vendored upstream — not our test surface
        if _has_test_surface(pkg_dir):
            found.append(str(pkg_dir.relative_to(repo_root)))
    return found


def ros_ci_skips_tests(repo_root: Path) -> bool:
    """True while ros-ci.yml still pins `skip-tests: 'true'` (the build-only gate)."""
    path = repo_root / WORKFLOW
    if not path.exists():
        return False
    return bool(
        re.search(r"^\s*skip-tests:\s*['\"]?true['\"]?\s*$", path.read_text(), re.MULTILINE)
    )


def main() -> int:
    repo_root = REPO_ROOT
    if not ros_ci_skips_tests(repo_root):
        # Tests are enabled (or the workflow is gone) — nothing to guard against.
        return 0
    offenders = first_party_packages_with_tests(repo_root)
    if not offenders:
        return 0
    print(
        "First-party ROS package(s) now carry tests, but ros-ci.yml still sets skip-tests: 'true', "
        "so those tests would NOT run in CI (Hermes Medium #2):",
        file=sys.stderr,
    )
    for pkg in offenders:
        print(f"  - {pkg}", file=sys.stderr)
    print(
        "Flip `skip-tests` to 'false' in .github/workflows/ros-ci.yml (isolating the vendored "
        "external/ lint noise, e.g. via colcon --packages-skip), then this guard self-clears.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
