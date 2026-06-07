#!/usr/bin/env python3
"""Emit Docker build args from stack-manifest.toml (the single source of truth; ADR-0004/0005).

The sim/dev Dockerfiles carry no version literals — their ARG values are injected from the
manifest by this script, so an OQ-3 pin change in `stack-manifest.toml` flows into the image
with no duplicated literal to update by hand.

Usage:
    docker build $(scripts/gen_build_args.py) --target px4-build -t patrol-sim:px4-build .
    scripts/gen_build_args.py --env > .env.build   # KEY=VALUE for `docker compose --env-file .env.build`
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANIFEST = REPO_ROOT / "stack-manifest.toml"


def build_args(manifest: dict) -> dict[str, str]:
    """Map manifest values to the Docker build ARGs the sim/dev images consume."""
    container = manifest["container"]
    flight = manifest["flight_stack"]
    simulator = manifest["simulator"]
    middleware = manifest["middleware"]
    bridge = manifest["bridge"]
    ros_base = f"{container['ros_base_image']}@{container['ros_base_digest']}"
    return {
        "ROS_BASE_IMAGE": ros_base,
        "PX4_VERSION": flight["px4_version"],
        "PX4_COMMIT": flight["px4_commit"],
        "GZ_VERSION": simulator["gazebo"],
        "ROS_DISTRO": middleware["ros_distro"],
        "XRCE_AGENT_SOURCE": bridge["uxrce_dds_agent_source"],
        "XRCE_AGENT_VERSION": bridge["uxrce_dds_agent_version"],
        "XRCE_AGENT_COMMIT": bridge["uxrce_dds_agent_commit"],
        # Transitive superbuild dep pins — verified post-fetch by build_xrce_agent.sh (Medium #3)
        "XRCE_FASTCDR_COMMIT": bridge["uxrce_fastcdr_commit"],
        "XRCE_FASTDDS_COMMIT": bridge["uxrce_fastdds_commit"],
        "XRCE_FOONATHAN_COMMIT": bridge["uxrce_foonathan_memory_commit"],
        "XRCE_SPDLOG_COMMIT": bridge["uxrce_spdlog_commit"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env",
        action="store_true",
        help="emit KEY=VALUE lines (for .env.build) instead of --build-arg flags",
    )
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    args = parser.parse_args(argv)

    with args.manifest.open("rb") as fh:
        manifest = tomllib.load(fh)
    pairs = build_args(manifest)

    if args.env:
        print("\n".join(f"{key}={value}" for key, value in pairs.items()))
    else:
        print(" ".join(f"--build-arg {key}={value}" for key, value in pairs.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
