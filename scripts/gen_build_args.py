#!/usr/bin/env python3
"""Emit Docker build args from stack-manifest.toml (the single source of truth; ADR-0004/0005).

The sim/dev Dockerfiles carry no version literals — their ARG values are injected from the
manifest by this script, so an OQ-3 pin change in `stack-manifest.toml` flows into the image
with no duplicated literal to update by hand.

Usage:
    docker build $(scripts/gen_build_args.py) --target px4-build -t patrol-sim:px4-build docker/sim
    scripts/gen_build_args.py --env > docker/sim/.env   # KEY=VALUE form for `docker compose`
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
    ros_base = f"{container['ros_base_image']}@{container['ros_base_digest']}"
    return {
        "ROS_BASE_IMAGE": ros_base,
        "PX4_VERSION": flight["px4_version"],
        "PX4_COMMIT": flight["px4_commit"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env",
        action="store_true",
        help="emit KEY=VALUE lines (for docker/sim/.env) instead of --build-arg flags",
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
