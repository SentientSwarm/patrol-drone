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

import re
import sys
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


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


def check_dockerfile_no_defaults(repo_root: Path) -> list[str]:
    text = (repo_root / "docker" / "sim" / "Dockerfile").read_text()
    return [
        f"docker/sim/Dockerfile pins ARG {arg} with a default; inject it from the manifest"
        for arg in ("PX4_VERSION", "PX4_COMMIT", "ROS_BASE_IMAGE")
        if re.search(rf"^ARG {arg}=", text, re.MULTILINE)
    ]


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
