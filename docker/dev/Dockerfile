# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom
# ^ intentional: ROS_BASE_IMAGE has NO default — it is injected (digest-pinned) from
#   stack-manifest.toml by scripts/gen_build_args.py. An empty FROM means "you forgot the
#   build args," which is the correct failure (mirrors docker/sim/Dockerfile; ADR-0005).

# ============================================================================
# patrol-drone — dev container (docset 01-platform, C2 / §4.2.2)
# ----------------------------------------------------------------------------
# Interactive development container. Shares the EXACT digest-pinned base layer with the sim
# container (C1) — one base, two derivatives (the DoD two-container constraint). The source
# tree is bind-mounted at runtime (docker-compose.yml), NOT baked, so edits are live and a
# node author rebuilds in-container with no image rebuild.
#
# Version pins come from stack-manifest.toml via build ARGs (ADR-0004/0005) — no version
# literal is duplicated here. ARGs carry NO defaults; values are injected by
# scripts/gen_build_args.py.
# ============================================================================

ARG ROS_BASE_IMAGE
FROM ${ROS_BASE_IMAGE} AS dev

ARG ROS_DISTRO             # stack-manifest.toml [middleware] ros_distro
ENV DEBIAN_FRONTEND=noninteractive

# Interactive dev tooling on top of the shared base: colcon + vcstool, a debugger/editor, and
# the CycloneDDS RMW (the PX4/ROS 2 community default for the uXRCE-DDS path).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-vcstool \
        gdb vim less \
        "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
 && rm -rf /var/lib/apt/lists/*

# Source is NOT copied — it is bind-mounted at runtime (see docker-compose.yml dev volumes).
WORKDIR /workspace/ros2_ws
CMD ["bash"]
