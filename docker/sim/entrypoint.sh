#!/usr/bin/env bash
# patrol-drone sim container entrypoint (docset 01-platform, C4 / §4.2.1, §4.2.4).
#
# Brings up the native uXRCE-DDS bridge against PX4 SITL + Gazebo Harmonic:
#   1. Micro XRCE-DDS Agent on UDP-localhost :8888 (the bridge — NOT MAVROS)
#   2. PX4 SITL + gz_x500 (headless by default); PX4's bundled uxrce_dds_client auto-starts
#      in SITL (design A1) and connects to the agent, exposing /fmu/out/* + /fmu/in/* to ROS 2.
#
# Verify from another shell (or `docker compose exec sim`):
#   ros2 topic list | grep fmu
#   ros2 topic hz /fmu/out/vehicle_local_position_v1   # steady ~50 Hz over 60 s (PLAT-2)
#   (PX4 v1.17 advertises message-versioned topic names — note the _v1 suffix.)
#
# NOTE: no `set -u` — ROS's setup.bash references unbound vars (e.g. AMENT_TRACE_SETUP_FILES)
# and would abort under nounset. All our own expansions use ${VAR:-default}, so it isn't needed.
set -eo pipefail

# Headless software-rendering defaults (OQ-4) so the container runs with no display in CI.
# A GPU profile overrides LIBGL_ALWAYS_SOFTWARE=0 via compose env (OQ-5).
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"

# ROS_DISTRO is set by the base image (osrf/ros:<distro>-desktop); default for safety.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source /opt/ros2_ws/install/setup.bash

# Both the agent and PX4 run as supervised background jobs so the entrypoint can (a) forward
# SIGTERM/SIGINT to them on `docker stop` for a prompt graceful shutdown, and (b) exit as soon as
# EITHER dies — so a crashed agent (e.g. UDP :8888 already bound) brings the container down with a
# non-zero status instead of silently running PX4 with no bridge. Compose sets `init: true` so a
# real init (tini) is PID 1, reaping orphans and delivering signals here. PX4 is launched in its
# own process group (setsid) so the trap can kill the whole gz/px4 subtree, not just `make`.
AGENT_PID=""
PX4_PID=""
shutdown() {
  trap '' INT TERM            # idempotent: ignore further signals while tearing down
  if [[ -n "${PX4_PID}" ]]; then kill -TERM -- "-${PX4_PID}" 2>/dev/null || true; fi  # PX4 proc group
  if [[ -n "${AGENT_PID}" ]]; then kill -TERM "${AGENT_PID}" 2>/dev/null || true; fi
}
trap shutdown INT TERM EXIT

# 1. Micro XRCE-DDS Agent (the bridge — C4).
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

# 2. PX4 SITL + Gazebo Harmonic gz_x500 (headless unless a display is attached) in its own process
#    group. The bundled uxrce_dds_client auto-starts in SITL (A1); if a future PX4 tag drops that,
#    add an explicit `uxrce_dds_client start` here (the only entrypoint change OQ-3 could force).
setsid env HEADLESS="${HEADLESS:-1}" make -C /opt/PX4-Autopilot px4_sitl gz_x500 &
PX4_PID=$!

# Block until the first of {agent, PX4} exits, then `shutdown` (EXIT trap) tears down the other.
wait -n

