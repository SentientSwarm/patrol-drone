#!/usr/bin/env python3
"""Fail if anything has drifted from stack-manifest.toml, the pinned-stack source of truth.

ADR-0004 makes `stack-manifest.toml` canonical and forbids duplicated version literals in the
executable consumers (setup script, Dockerfile) and the human-facing summaries (README,
CLAUDE.md). This guard enforces that contract in CI:

  1. px4_msgs_ref sits on px4_version's release line     (Hermes Medium #1)
  2. setup_phase1.sh derives versions, hardcodes none     (Hermes Medium #3; incl. bridge agent + mcap)
  3. docker/sim/Dockerfile ARGs carry no version defaults  (Hermes Medium #3)
  4. README "Stack at a glance" carries no stale PX4 pin    (Hermes Low #1)
  5. CLAUDE.md summary table agrees with the manifest       (ADR-0004)
  6. Dockerfile command bodies carry no hardcoded version/distro literals (round-3 Medium #1)

Exit 0 = clean; exit 1 = drift, with one line per problem.
"""

from __future__ import annotations

import hashlib
import re
import subprocess
import sys
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Vendored upstream subtrees (committed, not fetched) and their manifest tree-hash keys.
VENDOR_TREES = {
    "ros2_ws/src/external/px4_msgs": "px4_msgs_tree_sha",
    "ros2_ws/src/external/px4_ros_com": "px4_ros_com_tree_sha",
}


def load_manifest(repo_root: Path) -> dict:
    with (repo_root / "stack-manifest.toml").open("rb") as fh:
        return tomllib.load(fh)


def px4_release_line(px4_version: str) -> str:
    """'v1.17.0' -> '1.17' — the major.minor that names the px4_msgs release branch."""
    match = re.match(r"v?(\d+)\.(\d+)", px4_version)
    if not match:
        raise ValueError(f"unparseable px4_version: {px4_version!r}")
    return f"{match.group(1)}.{match.group(2)}"


def check_px4_msgs_alignment(manifest: dict) -> list[str]:
    flight = manifest["flight_stack"]
    expected = f"release/{px4_release_line(flight['px4_version'])}"
    if flight["px4_msgs_ref"] != expected:
        return [
            f"px4_msgs_ref={flight['px4_msgs_ref']!r} is off px4_version's release line "
            f"({flight['px4_version']!r} -> expected {expected!r})"
        ]
    return []


# Every pinned value the setup script must DERIVE from the manifest, as {shell var: manifest key}.
# A literal inlined in the script removes the `VAR="$(manifest_get key)"` line, which this gate
# then reports — closing the blind spot where changing UV/QGC/Foxglove values stayed green.
DERIVED_VARS = {
    "PX4_VERSION": "flight_stack.px4_version",
    "PX4_COMMIT": "flight_stack.px4_commit",
    "ROS_DISTRO": "middleware.ros_distro",
    "ROS_APT_SOURCE_VERSION": "ros_apt_source.version",
    "ROS_APT_SOURCE_SHA256": "ros_apt_source.sha256",
    "UXRCE_AGENT_SOURCE": "bridge.uxrce_dds_agent_source",
    "UXRCE_AGENT_VERSION": "bridge.uxrce_dds_agent_version",
    "UXRCE_AGENT_COMMIT": "bridge.uxrce_dds_agent_commit",
    "UXRCE_FASTCDR_COMMIT": "bridge.uxrce_fastcdr_commit",
    "UXRCE_FASTDDS_COMMIT": "bridge.uxrce_fastdds_commit",
    "UXRCE_FOONATHAN_COMMIT": "bridge.uxrce_foonathan_memory_commit",
    "UXRCE_SPDLOG_COMMIT": "bridge.uxrce_spdlog_commit",
    "MCAP_PLUGIN": "bags.mcap_plugin",
    "UV_VERSION": "tools.uv_version",
    "UV_TARBALL_SHA256": "tools.uv_tarball_sha256",
    "QGC_VERSION": "apps.qgc_version",
    "QGC_URL": "apps.qgc_url",
    "QGC_SHA256": "apps.qgc_sha256",
    "FOXGLOVE_VERSION": "apps.foxglove_version",
    "FOXGLOVE_URL": "apps.foxglove_url",
    "FOXGLOVE_SHA256": "apps.foxglove_sha256",
}


def missing_derivations(text: str) -> list[str]:
    """Pinned vars in `text` that are no longer assigned via `manifest_get <key>` (inlined literal)."""
    problems = []
    for var, key in DERIVED_VARS.items():
        if not re.search(rf'{var}="\$\(manifest_get {re.escape(key)}\)"', text):
            problems.append(
                f"setup_phase1.sh no longer derives {var} from {key} via manifest_get()"
            )
    return problems


def check_setup_derives(repo_root: Path) -> list[str]:
    text = (repo_root / "scripts" / "setup_phase1.sh").read_text()
    problems = []
    if "manifest_get" not in text:
        problems.append("setup_phase1.sh no longer derives versions via manifest_get()")
    if re.search(r'(PX4_VERSION|ROS_DISTRO)="v?\d', text):
        problems.append("setup_phase1.sh hardcodes a version literal; derive it from the manifest")
    problems += missing_derivations(text)
    return problems


# Every ARG the Dockerfiles inject from the manifest (via gen_build_args.py) — none may carry a
# default, or the image could silently reintroduce a duplicated version literal (ADR-0004/0005/0006).
# NB: scoped to this set on purpose — a non-version ARG with a legitimate default (e.g. the
# OSRF_GPG_FPRS trust root in docker/sim/Dockerfile) is NOT a manifest value and must not be flagged.
_MANIFEST_ARGS = (
    "PX4_VERSION",
    "PX4_COMMIT",
    "ROS_BASE_IMAGE",
    "GZ_VERSION",
    "ROS_DISTRO",
    "XRCE_AGENT_SOURCE",
    "XRCE_AGENT_VERSION",
    "XRCE_AGENT_COMMIT",
    "XRCE_FASTCDR_COMMIT",
    "XRCE_FASTDDS_COMMIT",
    "XRCE_FOONATHAN_COMMIT",
    "XRCE_SPDLOG_COMMIT",
)


def check_dockerfile_no_defaults(repo_root: Path) -> list[str]:
    problems = []
    for rel in ("docker/sim/Dockerfile", "docker/dev/Dockerfile"):
        path = repo_root / rel
        if not path.exists():
            continue
        text = path.read_text()
        problems += [
            f"{rel} pins ARG {arg} with a default; inject it from the manifest"
            for arg in _MANIFEST_ARGS
            if re.search(rf"^ARG {arg}=", text, re.MULTILINE)
        ]
    return problems


# A hardcoded PX4 clone tag — `--branch v1.17.0` / `--tag=v1.16` — that should be `${PX4_VERSION}`.
# Anchored to the clone-arg so an unrelated version-shaped token (a `pip==1.2.3`, a `/v2.0/` URL
# path, a soname) is NOT misflagged as the PX4 pin.
_PX4_BRANCH_LITERAL = re.compile(r"--(?:branch|tag)[ =]v\d+\.\d+")


def _dockerfile_command_body(text: str) -> str:
    """Dockerfile text with comments removed: full-line comments AND whitespace-preceded trailing
    `# ...` comments. (A `#` not preceded by whitespace — e.g. `${VAR#x}`, `sha256:...` — is kept.)
    """
    lines = []
    for line in text.splitlines():
        if re.match(r"\s*#", line):
            continue  # full-line comment
        lines.append(re.sub(r"\s#.*$", "", line))  # drop a trailing ` # ...` comment
    return "\n".join(lines)


def _literals_in_dockerfile(rel: str, path: Path, forbidden) -> list[str]:
    """Forbidden version/distro literals found in one Dockerfile's command body."""
    body = _dockerfile_command_body(path.read_text())
    return [
        f"{rel} contains {label} in a command body; derive it from the manifest"
        for pattern, label in forbidden
        if pattern.search(body)
    ]


def check_dockerfile_no_literals(repo_root: Path, manifest: dict) -> list[str]:
    """Reject hardcoded version/distro literals in Dockerfile command bodies (not just ARG defaults).

    The forbidden distro/simulator words are DERIVED from the manifest (not hardcoded), so the gate
    self-updates when the ros_distro / gazebo pin changes instead of guarding stale tokens.
    """
    forbidden = (
        (_PX4_BRANCH_LITERAL, "a hardcoded PX4 version (use ${PX4_VERSION})"),
        (
            re.compile(rf"\b{re.escape(manifest['middleware']['ros_distro'])}\b"),
            "a hardcoded ROS distro (use ${ROS_DISTRO})",
        ),
        (
            re.compile(rf"\b{re.escape(manifest['simulator']['gazebo'])}\b"),
            "a hardcoded Gazebo version (use ${GZ_VERSION})",
        ),
    )
    problems = []
    for rel in ("docker/sim/Dockerfile", "docker/dev/Dockerfile"):
        path = repo_root / rel
        if path.exists():
            problems += _literals_in_dockerfile(rel, path, forbidden)
    return problems


# ROS 2 distro codenames — lets the gate reject a *hardcoded* distro package in a Dockerfile command
# body even when it isn't the current manifest distro (the manifest-word check only catches the
# current one, leaving `ros-humble-…` as a blind spot — Hermes Medium #8).
_ROS2_DISTROS = ("foxy", "galactic", "humble", "iron", "jazzy", "kilted", "rolling")
_ROS_DISTRO_LITERAL = re.compile(rf"ros-(?:{'|'.join(_ROS2_DISTROS)})-")

# A Gazebo metapackage pinned to a literal release (gz-fortress, gz-harmonic, …) instead of the
# gz-${GZ_VERSION} variable. Value-agnostic, so a *wrong* version (gz-fortress) is caught too — the
# specific blind spot Hermes flagged (`gz-${GZ_VERSION}` → `gz-fortress` stayed green).
_GZ_LITERAL = re.compile(r"gz-(?!\$\{GZ_VERSION\})[a-z]")


def check_dockerfile_hardcoded_alternatives(repo_root: Path) -> list[str]:
    """Reject hardcoded Gazebo/ROS-distro *alternatives*, not only the current manifest token.

    The existing manifest-word guard catches a literal of the CURRENT pin (e.g. `harmonic`); this
    closes the complementary gap where a literal *wrong* value (`gz-fortress`, `ros-humble-…`) slips
    through because it is not the guarded token (Hermes Medium #8).
    """
    problems = []
    for rel in ("docker/sim/Dockerfile", "docker/dev/Dockerfile"):
        path = repo_root / rel
        if not path.exists():
            continue
        body = _dockerfile_command_body(path.read_text())
        if _GZ_LITERAL.search(body):
            problems.append(f"{rel} pins a literal Gazebo metapackage; use gz-${{GZ_VERSION}}")
        if _ROS_DISTRO_LITERAL.search(body):
            problems.append(f"{rel} pins a literal ROS distro package; use ros-${{ROS_DISTRO}}-…")
    return problems


def _vendor_tree_sha(repo_root: Path, rel: str) -> str | None:
    """sha256 of `git ls-files -s <rel>` (file mode + git blob SHA + path per tracked file).

    Deterministic and offline — reuses git's own content-addressed blob hashes. None when the path
    has no git-tracked files (so the caller can flag a missing/empty vendored subtree).
    """
    result = subprocess.run(
        ["git", "-C", str(repo_root), "ls-files", "-s", rel],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0 or not result.stdout:
        return None
    return hashlib.sha256(result.stdout).hexdigest()


def check_vendor_provenance(repo_root: Path, manifest: dict) -> list[str]:
    """Verify each vendored subtree's content fingerprint against its manifest pin (Hermes Medium #5).

    The *_commit pins in [flight_stack] only *document* upstream provenance; this recomputes a
    tree-hash over the committed copy and compares, so any local edit to the vendored upstream source
    trips CI offline instead of drifting silently from its stated provenance.
    """
    flight = manifest["flight_stack"]
    problems = []
    for rel, key in VENDOR_TREES.items():
        expected = flight.get(key)
        if not expected:
            problems.append(
                f"stack-manifest.toml [flight_stack] is missing vendor tree hash {key!r}"
            )
            continue
        actual = _vendor_tree_sha(repo_root, rel)
        if actual is None:
            problems.append(f"vendored subtree {rel} has no git-tracked files to fingerprint")
        elif actual != expected:
            problems.append(
                f"vendored subtree {rel} drifted from its recorded provenance: "
                f"tree sha {actual} != manifest {key}={expected}"
            )
    return problems


def check_workflow_distro(repo_root: Path, manifest: dict) -> list[str]:
    """ROS CI's `target-ros2-distro` must equal the manifest ROS distro (Hermes round-3 Medium #1).

    The required ROS CI is a manifest consumer like the setup script and Dockerfiles: a distro bump
    in stack-manifest.toml must flow here too, or CI would keep building the old distro while the
    rest of the toolchain moved.
    """
    path = repo_root / ".github" / "workflows" / "ros-ci.yml"
    if not path.exists():
        return [".github/workflows/ros-ci.yml is missing"]
    expected = manifest["middleware"]["ros_distro"]
    match = re.search(r"^\s*target-ros2-distro:\s*(\S+)", path.read_text(), re.MULTILINE)
    if not match:
        return ["ros-ci.yml has no target-ros2-distro to validate against the manifest"]
    actual = match.group(1).strip("\"'")
    if actual != expected:
        return [
            f"ros-ci.yml target-ros2-distro={actual!r} != manifest middleware.ros_distro={expected!r}"
        ]
    return []


def _stale_px4_tokens(text: str, release_line: str) -> list[str]:
    """PX4 version tokens in `text` whose major.minor differs from the manifest release line."""
    return [tok for tok in re.findall(r"v\d+\.\d+", text) if not tok.startswith(f"v{release_line}")]


def check_readme(repo_root: Path, manifest: dict) -> list[str]:
    text = (repo_root / "README.md").read_text()
    match = re.search(r"\| Flight stack \|([^\n|]*)\|", text)
    if not match:
        return ["README 'Stack at a glance' has no Flight stack row to validate"]
    line = px4_release_line(manifest["flight_stack"]["px4_version"])
    stale = _stale_px4_tokens(match.group(1), line)
    if stale:
        return [
            f"README Flight stack row carries a stale PX4 pin {stale}; point to stack-manifest.toml"
        ]
    return []


def check_claudemd(repo_root: Path, manifest: dict) -> list[str]:
    text = (repo_root / "CLAUDE.md").read_text()
    match = re.search(r"\| Flight stack \|([^\n]*)", text)
    if not match:
        return ["CLAUDE.md 'Stack (pinned)' has no Flight stack row to validate"]
    px4_version = manifest["flight_stack"]["px4_version"]
    if px4_version not in match.group(1):
        return [f"CLAUDE.md Flight stack row does not cite the manifest pin {px4_version!r}"]
    return []


def run_checks(repo_root: Path) -> list[str]:
    manifest = load_manifest(repo_root)
    problems = []
    problems += check_px4_msgs_alignment(manifest)
    problems += check_setup_derives(repo_root)
    problems += check_dockerfile_no_defaults(repo_root)
    problems += check_dockerfile_no_literals(repo_root, manifest)
    problems += check_dockerfile_hardcoded_alternatives(repo_root)
    problems += check_vendor_provenance(repo_root, manifest)
    problems += check_workflow_distro(repo_root, manifest)
    problems += check_readme(repo_root, manifest)
    problems += check_claudemd(repo_root, manifest)
    return problems


def main(argv: list[str] | None = None) -> int:
    repo_root = Path(argv[0]) if argv else REPO_ROOT
    problems = run_checks(repo_root)
    if problems:
        print("stack-manifest drift detected:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    print("stack-manifest.toml: no drift in consumers or summaries.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
