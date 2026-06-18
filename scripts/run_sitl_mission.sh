#!/usr/bin/env bash
#
# run_sitl_mission.sh — one command to fly + verify the M3 basic mission in SITL (UAT Runner).
#
# Brings the stack up in the CORRECT order, handling the host env quirks so a human never assembles
# them by hand, then self-verifies PASS/FAIL and tears down:
#
#     env_doctor  ->  agent (udp4 :8888)  ->  PX4 SITL + Gazebo (gz_x500)  ->  [QGC]
#                 ->  mission node (mission_basic.launch.py)  ->  verify_mission.py  ->  teardown
#
# This is NOT a second bring-up: the order (agent BEFORE PX4) mirrors docker/sim/entrypoint.sh, the
# bridge wait mirrors .github/workflows/sitl-nightly.yml, the node uses the same launch file as the
# nightly test, and the PASS/FAIL verdict comes from the same shared criteria (verify_mission.py ->
# tests/integration/mission_acceptance.py). So the host path and CI can't drift.
#
# Why the agent is first: PX4's uxrce_dds_client connects to the agent on startup; if the agent
# isn't already listening, the node comes up but receives ZERO telemetry and the drone never moves.
#
# Why QGC: on an interactive host PX4 preflight wants a GCS heartbeat ("No connection to the GCS"
# otherwise), so we launch QGC by default to supply it. The headless nightly arms without QGC inside
# its container; --no-qgc mirrors that fully here — Gazebo runs headless (HEADLESS=1) and the
# X11/QGC doctor checks are skipped (DOCTOR_HEADLESS=1), so it works on a GUI-less host.
#
# No `set -u`: we source ROS's setup.bash (references unbound vars; entrypoint.sh does the same).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Reuse env_doctor's agent resolver + capability gate rather than re-implementing them (one source
# of truth for "is the agent runnable"). Sourcing only DEFINES its functions (env_doctor's main is
# guarded). Source it FIRST so this script's own usage()/main() definitions below take precedence.
# shellcheck source=scripts/env_doctor.sh disable=SC1091
source "${SCRIPT_DIR}/env_doctor.sh"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
QGC_APPIMAGE="${HOME}/Apps/QGroundControl-x86_64.AppImage"
WS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
LOG_DIR="${PATROL_UAT_LOG_DIR:-/tmp/patrol-uat}"
STATUS_TOPIC=""  # resolved from patrol_mission.topics once ROS + the ws are sourced (one source of truth)

WITH_QGC=1
KEEP_UP=0
SKIP_DOCTOR=0
MISSION="basic"  # basic -> mission_basic.launch.py + verify_mission.py; patrol -> the M4 patrol
VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-}"  # default chosen per-mission in main (120 basic / 300 patrol)

AGENT_PID=""
PX4_PID=""
QGC_PID=""
NODE_PID=""

log()  { printf '\033[1;34m[run-sitl]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m     %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[err]\033[0m      %s\n' "$*" >&2; }

usage() {
  cat <<'EOF'
Usage: scripts/run_sitl_mission.sh [options]

Brings up agent + PX4 SITL + (QGC) + the mission node, verifies the M3 acceptance criteria, and
tears down. Logs land in $PATROL_UAT_LOG_DIR (default /tmp/patrol-uat).

  --patrol        Fly the M4 multi-waypoint patrol (mission_patrol.launch.py + verify_patrol.py)
                  instead of the M3 basic mission. 05's recorder is disabled (record:=false) and
                  the checkpoints path is pinned to the repo's sim/config/checkpoints.yaml.
  --no-qgc        Fully headless: skip QGC + the X11/QGC doctor checks and run Gazebo headless
                  (HEADLESS=1) — mirrors the nightly container; works on a GUI-less host.
  --keep-up       Leave the stack running after verifying (inspect in QGC / re-run the verifier).
  --skip-doctor   Skip the env_doctor capability gate.
  --timeout N     Seconds the verifier watches before giving up (default: 120 basic / 300 patrol).
  -h, --help      Show this help.
EOF
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --patrol) MISSION="patrol" ;;
      --no-qgc) WITH_QGC=0 ;;
      --keep-up) KEEP_UP=1 ;;
      --skip-doctor) SKIP_DOCTOR=1 ;;
      --timeout) VERIFY_TIMEOUT="${2:?--timeout needs a value}"; shift ;;
      -h | --help) usage; exit 0 ;;
      *) err "unknown option: $1"; usage; exit 2 ;;
    esac
    shift
  done
}

# shellcheck disable=SC2317,SC2329  # reached via `trap shutdown ...` in main(), not a direct call
shutdown() {
  trap '' INT TERM
  log "tearing down (logs kept in ${LOG_DIR})"
  if [[ -n "${NODE_PID}" ]]; then kill -TERM "${NODE_PID}" 2>/dev/null || true; fi
  if [[ -n "${PX4_PID}" ]]; then kill -TERM -- "-${PX4_PID}" 2>/dev/null || true; fi  # PX4 proc group
  if [[ -n "${QGC_PID}" ]]; then kill -TERM "${QGC_PID}" 2>/dev/null || true; fi
  if [[ -n "${AGENT_PID}" ]]; then kill -TERM "${AGENT_PID}" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}

# Apply the agent PATH/LD_LIBRARY_PATH fixes env_doctor would otherwise only print, so the agent is
# runnable in THIS shell (and the doctor's agent check below then passes). resolve_xrce_agent comes
# from the sourced env_doctor.sh (above) — one source of truth for locating the agent.
prepare_agent_env() {
  if resolve_xrce_agent; then
    local agent_dir
    agent_dir="$(dirname "${XRCE_AGENT_BIN}")"
    export PATH="${agent_dir}:${PATH}"
    [[ -n "${XRCE_AGENT_LIBDIR}" ]] && export LD_LIBRARY_PATH="${XRCE_AGENT_LIBDIR}:${LD_LIBRARY_PATH:-}"
  fi
}

source_ros() {
  local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -r "${ros_setup}" ]] || { err "ROS 2 ${ROS_DISTRO} not sourceable (${ros_setup}); run scripts/env_doctor.sh"; exit 1; }
  [[ -r "${WS_SETUP}" ]] || { err "workspace not built (${WS_SETUP}); run scripts/env_doctor.sh for the build command"; exit 1; }
  # BOTH sources: ROS itself AND the workspace (px4_msgs types). Missing the second -> the node and
  # `ros2 topic` report "message type 'px4_msgs/msg/...' is invalid".
  # shellcheck disable=SC1090,SC1091
  source "${ros_setup}"
  # shellcheck disable=SC1090,SC1091
  source "${WS_SETUP}"
}

wait_for_agent_port() {
  for _ in $(seq 1 30); do
    if ss -lun 2>/dev/null | grep -qE '[:.]8888([^0-9]|$)'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_for_bridge() {
  for _ in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -qx "${STATUS_TOPIC}"; then return 0; fi
    sleep 5
  done
  return 1
}

start_agent() {
  log "starting Micro XRCE-DDS Agent (udp4 :8888) -> ${LOG_DIR}/agent.log"
  MicroXRCEAgent udp4 -p 8888 >"${LOG_DIR}/agent.log" 2>&1 &
  AGENT_PID=$!
  wait_for_agent_port || { err "agent did not open UDP :8888 (see ${LOG_DIR}/agent.log)"; exit 1; }
}

start_px4() {
  # --no-qgc is a fully headless run: default Gazebo to headless (no GUI) so it works on a GUI-less
  # host and matches the nightly container. An explicit HEADLESS= in the environment still wins.
  local headless_default=0
  [[ ${WITH_QGC} -eq 0 ]] && headless_default=1
  local headless="${HEADLESS:-${headless_default}}"
  log "starting PX4 SITL + Gazebo (gz_x500, HEADLESS=${headless}) -> ${LOG_DIR}/px4.log    (first launch may relink; be patient)"
  # Own process group (setsid) so teardown can kill the whole gz/px4 subtree, not just make.
  setsid env HEADLESS="${headless}" make -C "${PX4_DIR}" px4_sitl gz_x500 >"${LOG_DIR}/px4.log" 2>&1 &
  PX4_PID=$!
}

start_qgc() {
  [[ ${WITH_QGC} -eq 1 ]] || { log "QGC disabled (--no-qgc) — running headless"; return 0; }
  if [[ -x "${QGC_APPIMAGE}" ]]; then
    log "starting QGroundControl (GCS heartbeat for offboard arm) -> ${LOG_DIR}/qgc.log"
    "${QGC_APPIMAGE}" >"${LOG_DIR}/qgc.log" 2>&1 &
    QGC_PID=$!
  else
    warn "QGC not runnable at ${QGC_APPIMAGE}; continuing without it"
    warn "  (offboard arm may fail with 'No connection to the GCS' — see scripts/env_doctor.sh)"
  fi
}

start_node() {
  if [[ "${MISSION}" == patrol ]]; then
    # record:=false so no 05 dependency; pin checkpoints to the repo file (resolves regardless of CWD).
    log "launching mission node (ros2 launch patrol_bringup mission_patrol.launch.py record:=false) -> ${LOG_DIR}/node.log"
    ros2 launch patrol_bringup mission_patrol.launch.py record:=false \
      "checkpoints_yaml:=${REPO_ROOT}/sim/config/checkpoints.yaml" >"${LOG_DIR}/node.log" 2>&1 &
  else
    log "launching mission node (ros2 launch patrol_bringup mission_basic.launch.py) -> ${LOG_DIR}/node.log"
    ros2 launch patrol_bringup mission_basic.launch.py >"${LOG_DIR}/node.log" 2>&1 &
  fi
  NODE_PID=$!
}

report_keep_up() {
  trap - EXIT INT TERM  # don't tear down on exit
  local verifier="verify_mission.py"
  [[ "${MISSION}" == patrol ]] && verifier="verify_patrol.py"
  log "stack left running (--keep-up):"
  log "  PIDs: agent=${AGENT_PID} px4=${PX4_PID} node=${NODE_PID} qgc=${QGC_PID:-none}"
  log "  re-run the verifier: (source ROS + ws, then) python3 ${SCRIPT_DIR}/${verifier}"
  log "  tear down:           kill -- -${PX4_PID}; kill ${AGENT_PID} ${NODE_PID} ${QGC_PID:-}"
}

main() {
  parse_args "$@"
  mkdir -p "${LOG_DIR}"
  # --no-qgc is a fully headless run: tell the sourced doctor to skip the X11/QGC checks (Gazebo runs
  # headless via start_px4, PX4 needs no GCS), so the gate doesn't fail on a GUI-less host.
  [[ ${WITH_QGC} -eq 0 ]] && export DOCTOR_HEADLESS=1
  prepare_agent_env
  if [[ ${SKIP_DOCTOR} -eq 0 ]]; then
    run_checks || { err "env_doctor failed — fix the above, or re-run with --skip-doctor"; exit 1; }
  fi
  source_ros
  # Resolve the version-sensitive status topic from patrol_mission.topics now that the ws is on
  # PYTHONPATH — the `_v1` literal lives only in that module, not re-hardcoded here (Hermes Low).
  STATUS_TOPIC="$(python3 -m patrol_mission.topics VEHICLE_STATUS)"

  trap shutdown INT TERM EXIT
  start_agent
  start_px4
  start_qgc
  log "waiting for the live /fmu/* bridge (${STATUS_TOPIC})..."
  wait_for_bridge || { err "bridge did not come up in time (see ${LOG_DIR}/px4.log and agent.log)"; exit 1; }
  start_node

  local verifier="verify_mission.py"
  local default_timeout=120
  if [[ "${MISSION}" == patrol ]]; then
    verifier="verify_patrol.py"
    default_timeout=300
  fi
  VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-${default_timeout}}"
  log "verifying ${MISSION}-mission acceptance criteria (timeout ${VERIFY_TIMEOUT}s)..."
  set +e
  python3 "${SCRIPT_DIR}/${verifier}" --timeout "${VERIFY_TIMEOUT}"
  local verdict=$?
  set -e

  if [[ ${KEEP_UP} -eq 1 ]]; then
    report_keep_up
  fi
  exit "${verdict}"
}

main "$@"
