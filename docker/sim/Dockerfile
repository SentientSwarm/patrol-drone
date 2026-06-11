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
# M1 (T1.3): the `px4-build` stage compiles PX4 SITL from source at the pinned tag.
#   (M2 update: Gazebo Harmonic is now installed in this stage too, BEFORE the px4 build, so the
#   gz_bridge module compiles into the SITL build — see the stage-1 note below.) M1 smoke (full
#   build context = repo root because the runtime stage COPYs ros2_ws + entrypoint):
#     docker build $(scripts/gen_build_args.py) --target px4-build -t patrol-sim:px4-build .
#
# M2 (T2.3): the `runtime` stage (FROM px4-build) adds the headless render libs + the
#   uXRCE-DDS agent (built from source, ADR-0007) + the built ros2_ws, and a headless entrypoint
#   that launches gz_x500 and bridges live /fmu/*.
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
ARG GZ_VERSION             # stack-manifest.toml [simulator] — Gazebo installed BEFORE the px4 build
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic FIRST (OSRF apt repo, added explicitly — the base image does not
# guarantee it), so PX4's gz_bridge module is found by cmake and compiled INTO px4_sitl_default
# and the cmake cache records Gazebo as present. (M2 spike: installing gz only in a later stage
# leaves the px4 cmake cache at `gz-transport NOTFOUND`, and `make px4_sitl gz_x500` then fails
# with "Gazebo simulation dependencies not found". Building px4_sitl_default does NOT launch gz.)
# The OSRF signing key is fingerprint-VERIFIED against a pinned trust root before its repo is
# trusted (mirrors setup_phase1.sh::verify_gpg_fingerprint + the *_GPG_FPR constants; ADR-0006).
# The keyring bundles OSRF's current + rotation keys — both fingerprints are pinned; a substituted
# key fails closed. Source: https://packages.osrfoundation.org/gazebo.gpg (uid "OSRF Repository").
ARG OSRF_GPG_FPRS="8856B7EEB6F479799BFEE7FA4149BA44D1BA6112 D2486D2DD83DB69272AFE98867170598AF249743"
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && actual="$(gpg --show-keys --with-colons /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        | awk -F: '/^fpr:/ {print $10}' | sort | paste -sd' ')" \
 && expected="$(printf '%s\n' ${OSRF_GPG_FPRS} | sort | paste -sd' ')" \
 && if [ "${actual}" != "${expected}" ]; then \
        echo "ERROR: OSRF gazebo key fingerprint mismatch (got: ${actual})" >&2; exit 1; fi \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        "gz-${GZ_VERSION}" \
        cppzmq-dev \
 && rm -rf /var/lib/apt/lists/*
# cppzmq-dev: PX4's gz_bridge -> find_package(gz-transport13), whose cmake config links the
# CPPZMQ::CPPZMQ target; the gz-harmonic metapackage does not pull the (header-only) cppzmq
# bindings, so cmake fails generation without it. (pulls libzmq3-dev transitively.)

# Clone PX4 at the pinned tag, VERIFY the checkout resolves to the pinned commit (mirrors
# setup_phase1.sh — catches an upstream-moved tag before any upstream code runs), install its
# build toolchain (no NuttX — SITL only; --no-sim-tools because we installed Gazebo above), then
# compile the default SITL build. With Gazebo present, px4_sitl_default compiles the gz_bridge
# module too WITHOUT launching a simulator — the runtime entrypoint's `make px4_sitl gz_x500`
# then only relinks-if-needed and launches.
RUN git clone --recurse-submodules --branch ${PX4_VERSION} --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && test "$(git -C /opt/PX4-Autopilot rev-parse HEAD)" = "${PX4_COMMIT}" \
 && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && make -C /opt/PX4-Autopilot px4_sitl_default \
 && rm -rf /var/lib/apt/lists/*

# ---- Stage 2: runtime — headless SITL + Gazebo + agent (M2 / T2.3) ---------
# NOTE on the stage base (deviation from design §4.2.1's slim copy — recorded for MZ, §6.5):
#   The design sketches a slim runtime that COPYs only `build/` + `Tools/`. But the launch
#   command `make px4_sitl gz_x500` (used identically on the host and in the entrypoint) needs
#   the PX4 Makefile + source tree + build system (cmake/ninja) present at launch — a runtime
#   with only compiled artifacts cannot drive `make`. To keep the M2 hard floor (a working live
#   `/fmu/*` bridge) correct-by-construction and identical to the proven host path, `runtime`
#   derives FROM the px4-build stage (full source + artifacts + toolchain). The px4 compile
#   layer is cached and not repeated, so this is still cheaper than a from-scratch build.
#   Slimming to a minimal image that launches PX4+gz WITHOUT `make` is a size optimization
#   deferred to MZ, not an M2 exit requirement.
#
#   CONTEXT: this stage COPYs `ros2_ws` and `docker/sim/entrypoint.sh`, so the full build must
#   use the REPO ROOT as build context (docker-compose.yml sets `context: .`). The M1 smoke
#   (`docker build ... docker/sim --target px4-build`) only exercises stage 1 and is unaffected.
#
# ARGs carry NO defaults — values come from scripts/gen_build_args.py (ADR-0004/0005/0006), so
# this stage never reintroduces a duplicated version literal.
FROM px4-build AS runtime

ARG ROS_DISTRO             # stack-manifest.toml [middleware] ros_distro
ARG XRCE_AGENT_SOURCE      # stack-manifest.toml [bridge]     uxrce_dds_agent_source
ARG XRCE_AGENT_VERSION     # stack-manifest.toml [bridge]     uxrce_dds_agent_version
ARG XRCE_AGENT_COMMIT      # stack-manifest.toml [bridge]     uxrce_dds_agent_commit
# Transitive superbuild dep pins — verified pre-build (ls-remote ref) AND post-build (checkout
# commit) by build_xrce_agent.sh (Hermes Medium #1). Both the ref and the commit flow through.
ARG XRCE_FASTCDR_REF       # stack-manifest.toml [bridge]     uxrce_fastcdr_ref
ARG XRCE_FASTCDR_COMMIT    # stack-manifest.toml [bridge]     uxrce_fastcdr_commit
ARG XRCE_FASTDDS_REF       # stack-manifest.toml [bridge]     uxrce_fastdds_ref
ARG XRCE_FASTDDS_COMMIT    # stack-manifest.toml [bridge]     uxrce_fastdds_commit
ARG XRCE_FOONATHAN_REF     # stack-manifest.toml [bridge]     uxrce_foonathan_memory_ref
ARG XRCE_FOONATHAN_COMMIT  # stack-manifest.toml [bridge]     uxrce_foonathan_memory_commit
ARG XRCE_SPDLOG_REF        # stack-manifest.toml [bridge]     uxrce_spdlog_ref
ARG XRCE_SPDLOG_COMMIT     # stack-manifest.toml [bridge]     uxrce_spdlog_commit

# Headless software-render libs (llvmpipe, OQ-4 default so CI/headless runs with no display).
# Gazebo Harmonic itself is already present — installed in the px4-build stage so the gz_bridge
# compiled into the SITL build (inherited here via FROM px4-build).
RUN apt-get update && apt-get install -y --no-install-recommends \
        mesa-utils libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# Pinned SITL test layer: the harness the nightly integration test (AC-5 / MC-10) drives —
# launch_pytest + pytest. Baked into the image (not apt-installed live at test time) so the test
# environment is reproducible from the digest-pinned base + manifest-pinned ROS_DISTRO alone. These
# are unversioned ROS/distro apt names (no manifest version literal), so the manifest-drift gate is
# unaffected. Installed via apt (no pip), matching the rest of the ROS toolchain.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pytest \
        "ros-${ROS_DISTRO}-launch-testing" \
        "ros-${ROS_DISTRO}-launch-pytest" \
 && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent — the native bridge (NOT MAVROS). M2 spike finding (ADR-0007): there is
# no `ros-${ROS_DISTRO}-micro-xrce-dds-agent` apt package in the Jazzy repo, so build it from
# source at the pinned eProsima tag (the PX4-canonical method). The build toolchain is already
# present (this stage is FROM px4-build). The recipe lives in ONE place — scripts/build_xrce_agent.sh
# — invoked here (root, no sudo) and by setup_phase1.sh on the host, so the two agents can't drift.
COPY scripts/build_xrce_agent.sh /tmp/build_xrce_agent.sh
RUN EXPECT_FASTCDR_REF="${XRCE_FASTCDR_REF}" EXPECT_FASTCDR_COMMIT="${XRCE_FASTCDR_COMMIT}" \
    EXPECT_FASTDDS_REF="${XRCE_FASTDDS_REF}" EXPECT_FASTDDS_COMMIT="${XRCE_FASTDDS_COMMIT}" \
    EXPECT_FOONATHAN_REF="${XRCE_FOONATHAN_REF}" EXPECT_FOONATHAN_COMMIT="${XRCE_FOONATHAN_COMMIT}" \
    EXPECT_SPDLOG_REF="${XRCE_SPDLOG_REF}" EXPECT_SPDLOG_COMMIT="${XRCE_SPDLOG_COMMIT}" \
    bash /tmp/build_xrce_agent.sh "${XRCE_AGENT_SOURCE}" "${XRCE_AGENT_VERSION}" "${XRCE_AGENT_COMMIT}" \
 && rm -f /tmp/build_xrce_agent.sh

# Vendored px4_msgs/px4_ros_com (T2.1) + patrol_* shells (T2.2) build into the workspace here.
COPY ros2_ws /opt/ros2_ws
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && cd /opt/ros2_ws && colcon build

COPY docker/sim/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
