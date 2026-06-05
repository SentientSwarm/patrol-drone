# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom
# ^ intentional: ROS_BASE_IMAGE has NO default — it is injected (digest-pinned) from
#   stack-manifest.toml by scripts/gen_build_args.py. An empty FROM here means
#   "you forgot the build args," which is the correct failure (see ADR-0005).

# ============================================================================
# patrol-drone — sim container (docset 01-platform, C1 / §4.2.1)
# ----------------------------------------------------------------------------
# Multi-stage, build-from-source (OQ-2 resolution). Version pins come from the
# canonical manifest `stack-manifest.toml` via build ARGs (see ADR-0004/0005) — no
# version literal is duplicated here. The ARGs below have NO defaults: their values
# are injected from the manifest by scripts/gen_build_args.py.
#
# M1 (T1.3): the `px4-build` stage compiles PX4 SITL from source at the pinned
#   tag — this is the M1 exit gate ("the sim multi-stage build stage compiles").
#   M1 smoke:
#     docker build $(scripts/gen_build_args.py) --target px4-build -t patrol-sim:px4-build docker/sim
#
# M2 (T2.3): the `runtime` stage is fleshed out — Gazebo Harmonic + the
#   uXRCE-DDS agent + a headless entrypoint that launches gz_x500 and bridges
#   live /fmu/*. Kept as a commented scaffold below at M1.
# ============================================================================

# Digest-pinned base image — injected from stack-manifest.toml [container]
# (ros_base_image@ros_base_digest) by scripts/gen_build_args.py. No default: pass --build-arg.
ARG ROS_BASE_IMAGE

# ---- Stage 1: px4-build — compile PX4 SITL from source ---------------------
FROM ${ROS_BASE_IMAGE} AS px4-build

# stack-manifest.toml [flight_stack] px4_version / px4_commit — the OQ-3 single edit point.
# No defaults: injected by scripts/gen_build_args.py. Pass --build-arg PX4_VERSION=... PX4_COMMIT=...
ARG PX4_VERSION
ARG PX4_COMMIT
ENV DEBIAN_FRONTEND=noninteractive

# Clone PX4 at the pinned tag, VERIFY the checkout resolves to the pinned commit (mirrors
# setup_phase1.sh — catches an upstream-moved tag before any upstream code runs), install its
# build toolchain (no NuttX — SITL only), then compile the default SITL build. `px4_sitl_default`
# builds the firmware + simulator interface WITHOUT launching a simulator (the design's
# `make px4_sitl gz_x500` launches gz, which a non-interactive build can't do —
# launching gz_x500 is the M2 runtime stage's job).
RUN git clone --recurse-submodules --branch ${PX4_VERSION} --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && test "$(git -C /opt/PX4-Autopilot rev-parse HEAD)" = "${PX4_COMMIT}" \
 && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && make -C /opt/PX4-Autopilot px4_sitl_default \
 && rm -rf /var/lib/apt/lists/*

# ---- Stage 2: runtime — headless SITL + Gazebo + agent (M2 / T2.3) ---------
# TODO(M2/T2.3, §4.2.1): flesh out the runtime stage. NB: like the ARGs above, these carry NO
# defaults — keep them manifest-derived so this scaffold can't reintroduce duplicated literals.
# At M2, extend scripts/gen_build_args.py to also emit ROS_DISTRO ([middleware]) and GZ_VERSION
# ([simulator]) so a copy of this block stays single-source-of-truth (ADR-0004/0005/0006).
#   FROM ${ROS_BASE_IMAGE} AS runtime
#   ARG ROS_DISTRO                                  # stack-manifest.toml [middleware] (inject via gen_build_args.py)
#   ARG GZ_VERSION                                  # stack-manifest.toml [simulator] (inject via gen_build_args.py)
#   RUN apt-get update && apt-get install -y --no-install-recommends \
#         gz-${GZ_VERSION} \
#         ros-${ROS_DISTRO}-micro-xrce-dds-agent \  # stack-manifest.toml [bridge]
#         mesa-utils libgl1-mesa-dri \              # llvmpipe software render (OQ-4)
#    && rm -rf /var/lib/apt/lists/*
#   COPY --from=px4-build /opt/PX4-Autopilot/build /opt/PX4-Autopilot/build
#   COPY --from=px4-build /opt/PX4-Autopilot/Tools /opt/PX4-Autopilot/Tools
#   COPY ros2_ws /opt/ros2_ws                       # vendored px4_msgs land here (T2.1)
#   RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --base-paths /opt/ros2_ws
#   COPY docker/sim/entrypoint.sh /entrypoint.sh    # agent + gz_x500 launch (T2.5)
#   ENTRYPOINT ["/entrypoint.sh"]
