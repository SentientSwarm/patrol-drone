#!/usr/bin/env bash
#
# run_patrol_world_sitl.sh — bring up the M5 patrol STAGE in SITL: the custom patrol_world + the
# gz_x500_patrol camera airframe + the ROS image bridge, then verify the camera publishes and
# (optionally) fly the M4 patrol so the drone visits each AprilTag checkpoint. The one-command
# manual check for M5 AC-1 (world loads) / AC-4 (camera publishes) / AC-3 (patrol traverses).
#
# WHY A SEPARATE RUNNER FROM run_sitl_mission.sh
#   The M3/M4 runner uses PX4's own `make px4_sitl gz_x500`, which starts Gazebo AND spawns the stock
#   x500 into the DEFAULT world. M5 needs a CUSTOM world and a CUSTOM camera model that is NOT a PX4
#   airframe — and PX4's normal path spawns models only from its own tree keyed by an airframe file
#   (adding one is a PX4 fork, which A2 forbids). The no-fork path is PX4's documented "bring your own
#   model" / attach mode, which inverts the bring-up: WE start Gazebo standalone with patrol_world,
#   spawn gz_x500_patrol ourselves, then start PX4 with PX4_GZ_STANDALONE + PX4_GZ_MODEL_NAME so it
#   ATTACHES to our model (PX4_SIM_MODEL=gz_x500 supplies the x500 airframe params). That topology is
#   different enough to warrant its own runner; the shared primitives (agent resolution, the
#   capability gate) are sourced from env_doctor.sh so they are not duplicated.
#
# STATUS: NIGHTLY / MANUAL. Verified live on an interactive X11 + NVIDIA host (2026-06-21): the
# standalone+attach bring-up works (gz patrol_world + EntityFactory spawn of gz_x500_patrol + PX4
# PX4_GZ_STANDALONE/PX4_GZ_MODEL_NAME attach), the camera publishes a steady ~15 Hz, and the full M4
# patrol arms, dwells at all checkpoints, RTHs and lands (AC-1/AC-3/AC-4 PASS). Two things the live
# run surfaced and that are now handled: (1) the gz camera renders in lockstep with PX4, so first-frame
# lags cold start — verify_camera now polls up to CAMERA_WAIT instead of a single tight echo; (2) PX4
# preflight wants a GCS heartbeat to arm offboard ("No connection to the GCS"), so the patrol path
# launches QGC (start_qgc), same as run_sitl_mission.sh. Not yet run headless/in-container.
#
# No `set -u`: we source ROS's setup.bash (references unbound vars), like run_sitl_mission.sh.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Reuse env_doctor's agent resolver + capability gate (one source of truth). Sourcing only DEFINES
# its functions (its main is guarded). Source FIRST so our usage()/main() below take precedence.
# shellcheck source=scripts/env_doctor.sh disable=SC1091
source "${SCRIPT_DIR}/env_doctor.sh"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
QGC_APPIMAGE="${QGC_APPIMAGE:-${HOME}/Apps/QGroundControl-x86_64.AppImage}"
WS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
LOG_DIR="${PATROL_UAT_LOG_DIR:-}"  # empty -> a fresh mktemp dir, set in main()

WORLD_NAME="patrol_world"
WORLD_SDF="${REPO_ROOT}/sim/worlds/patrol_world.sdf"
MODEL_NAME="gz_x500_patrol"
MODEL_SDF="${REPO_ROOT}/sim/px4_sitl_overrides/gz_x500_patrol/model.sdf"
CHECKPOINTS_YAML="${REPO_ROOT}/sim/config/checkpoints.yaml"
CAMERA_TOPIC="/drone/camera/image_raw"
# PX4's SITL server config wires the gz systems (Physics, SceneBroadcaster, Sensors, ...). We start
# Gazebo ourselves (standalone), so we must point it at the same config or the camera sensor renders
# nothing (no Sensors system).
GZ_SERVER_CONFIG="${PX4_DIR}/src/modules/simulation/gz_bridge/server.config"

WITH_GUI=1
RUN_PATROL=1
KEEP_UP=0
SKIP_DOCTOR=0
VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-300}"
CAMERA_WAIT="${CAMERA_WAIT:-90}"

AGENT_PID=""
GZ_PID=""
GZ_GUI_PID=""
PX4_PID=""
QGC_PID=""
BRIDGE_PID=""
NODE_PID=""

log()  { printf '\033[1;34m[patrol-world]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m        %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[err]\033[0m         %s\n' "$*" >&2; }

usage() {
  cat <<'EOF'
Usage: scripts/run_patrol_world_sitl.sh [options]

Brings up the M5 patrol stage (patrol_world + gz_x500_patrol camera + image bridge), verifies the
camera publishes, optionally flies the M4 patrol, then tears down. Logs in $PATROL_UAT_LOG_DIR
(default: a fresh mktemp dir under $TMPDIR; set $PATROL_UAT_LOG_DIR for a stable path).

  --no-gui        Headless: run Gazebo with no GUI (HEADLESS-style; the camera still renders offscreen).
  --no-patrol     Bring up + verify the camera only; skip flying the M4 patrol.
  --keep-up       Leave the stack running after verifying (inspect in Gazebo / re-run a verifier).
  --skip-doctor   Skip the env_doctor capability gate.
  --timeout N     Seconds the patrol verifier watches before giving up (default 300).
  -h, --help      Show this help.
EOF
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --no-gui) WITH_GUI=0 ;;
      --no-patrol) RUN_PATROL=0 ;;
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
  if [[ -n "${BRIDGE_PID}" ]]; then kill -TERM "${BRIDGE_PID}" 2>/dev/null || true; fi
  if [[ -n "${QGC_PID}" ]]; then kill -TERM "${QGC_PID}" 2>/dev/null || true; fi
  if [[ -n "${PX4_PID}" ]]; then kill -TERM -- "-${PX4_PID}" 2>/dev/null || true; fi  # PX4 proc group
  if [[ -n "${GZ_GUI_PID}" ]]; then kill -TERM "${GZ_GUI_PID}" 2>/dev/null || true; fi
  if [[ -n "${GZ_PID}" ]]; then kill -TERM "${GZ_PID}" 2>/dev/null || true; fi
  if [[ -n "${AGENT_PID}" ]]; then kill -TERM "${AGENT_PID}" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}

# Apply env_doctor's agent PATH/LD_LIBRARY_PATH fixes so the agent is runnable in THIS shell.
prepare_agent_env() {
  if resolve_xrce_agent; then
    local agent_dir
    agent_dir="$(dirname "${XRCE_AGENT_BIN}")"
    export PATH="${agent_dir}:${PATH}"
    [[ -n "${XRCE_AGENT_LIBDIR}" ]] && export LD_LIBRARY_PATH="${XRCE_AGENT_LIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
}

source_ros() {
  local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -r "${ros_setup}" ]] || { err "ROS 2 ${ROS_DISTRO} not sourceable (${ros_setup}); run scripts/env_doctor.sh"; exit 1; }
  [[ -r "${WS_SETUP}" ]]  || { err "workspace not built (${WS_SETUP}); (cd ros2_ws && colcon build)"; exit 1; }
  # shellcheck disable=SC1090,SC1091
  source "${ros_setup}"
  # shellcheck disable=SC1090,SC1091
  source "${WS_SETUP}"
}

# Gazebo must find our world's model:// AprilTag refs and the PX4 x500 base our model merges, and use
# PX4's server config so the camera Sensors system loads. GZ_SIM_RESOURCE_PATH is APPENDED to by PX4's
# gz_env.sh, so a value set here survives.
export_gz_env() {
  export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/sim/models:${REPO_ROOT}/sim/worlds:${REPO_ROOT}/sim/px4_sitl_overrides:${PX4_DIR}/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
  [[ -f "${GZ_SERVER_CONFIG}" ]] && export GZ_SIM_SERVER_CONFIG_PATH="${GZ_SERVER_CONFIG}"
}

start_agent() {
  log "starting Micro XRCE-DDS Agent (udp4 :8888) -> ${LOG_DIR}/agent.log"
  MicroXRCEAgent udp4 -p 8888 >"${LOG_DIR}/agent.log" 2>&1 &
  AGENT_PID=$!
}

start_gz() {
  log "starting Gazebo (standalone server) with ${WORLD_NAME} -> ${LOG_DIR}/gz.log"
  gz sim --verbose=1 -r -s "${WORLD_SDF}" >"${LOG_DIR}/gz.log" 2>&1 &
  GZ_PID=$!
  if [[ ${WITH_GUI} -eq 1 ]]; then
    log "starting Gazebo GUI"
    gz sim -g >"${LOG_DIR}/gz-gui.log" 2>&1 &
    GZ_GUI_PID=$!
  fi
}

wait_for_world() {
  log "waiting for the Gazebo world '${WORLD_NAME}' to be ready..."
  for _ in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "^/world/${WORLD_NAME}/clock"; then return 0; fi
    sleep 1
  done
  return 1
}

# Spawn gz_x500_patrol into the running world at the takeoff origin. This is the BYO-model step PX4's
# attach mode relies on; its EntityFactory request mirrors px4-rc.gzsim's own /create call. THE STEP
# MOST LIKELY TO NEED A TWEAK if the bring-up misbehaves (model name / pose / sdf_filename).
spawn_model() {
  log "spawning ${MODEL_NAME} into ${WORLD_NAME} (gz EntityFactory create)"
  gz service -s "/world/${WORLD_NAME}/create" \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"${MODEL_SDF}\", name: \"${MODEL_NAME}\", pose: {position: {z: 0.2}}" \
    >"${LOG_DIR}/spawn.log" 2>&1
  sleep 2  # let the model register before PX4 attaches
}

# Start PX4 in standalone+attach mode: it does NOT start Gazebo (PX4_GZ_STANDALONE) and attaches to
# our pre-spawned model (PX4_GZ_MODEL_NAME); PX4_SIM_MODEL=gz_x500 (from the make target) gives the
# x500 airframe params. Own process group so teardown kills the whole subtree.
start_px4() {
  local headless=0
  [[ ${WITH_GUI} -eq 0 ]] && headless=1
  log "starting PX4 SITL attaching to ${MODEL_NAME} in ${WORLD_NAME} -> ${LOG_DIR}/px4.log"
  setsid env \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD="${WORLD_NAME}" \
    PX4_GZ_MODEL_NAME="${MODEL_NAME}" \
    HEADLESS="${headless}" \
    make -C "${PX4_DIR}" px4_sitl gz_x500 >"${LOG_DIR}/px4.log" 2>&1 &
  PX4_PID=$!
}

wait_for_bridge() {
  log "waiting for the live /fmu/* uXRCE-DDS bridge..."
  for _ in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q '^/fmu/'; then return 0; fi
    sleep 5
  done
  return 1
}

start_camera_bridge() {
  log "starting camera image bridge (ros2 launch patrol_bringup camera_bridge.launch.py) -> ${LOG_DIR}/bridge.log"
  ros2 launch patrol_bringup camera_bridge.launch.py >"${LOG_DIR}/bridge.log" 2>&1 &
  BRIDGE_PID=$!
}

# Supply the GCS heartbeat PX4 preflight needs to ARM for offboard (otherwise the mission node loops on
# "Preflight Fail: No connection to the GCS" → arming denied). Only the patrol path arms, so the
# camera-only smoke skips it; headless runs skip it too (mirrors run_sitl_mission.sh --no-qgc, where
# PX4 arms without a GCS). Same proven mechanism as the M3/M4 runner.
start_qgc() {
  [[ ${RUN_PATROL} -eq 1 && ${WITH_GUI} -eq 1 ]] || return 0
  if [[ -x "${QGC_APPIMAGE}" ]]; then
    log "starting QGroundControl (GCS heartbeat for offboard arm) -> ${LOG_DIR}/qgc.log"
    "${QGC_APPIMAGE}" >"${LOG_DIR}/qgc.log" 2>&1 &
    QGC_PID=$!
  else
    warn "QGC not runnable at ${QGC_APPIMAGE}; patrol arm may fail with 'No connection to the GCS'"
  fi
}

# Poll for the first camera frame to reach ROS. The gz camera renders in LOCKSTEP with PX4, so during
# cold start (EKF init, lockstep settling) the first frame can lag well past a single echo's window —
# one tight `echo --once` false-negatives a working camera. Retry until CAMERA_WAIT elapses.
wait_for_camera_frame() {
  local deadline=$((SECONDS + CAMERA_WAIT))
  while ((SECONDS < deadline)); do
    if timeout 10 ros2 topic echo --once "${CAMERA_TOPIC}" >/dev/null 2>&1; then return 0; fi
  done
  return 1
}

verify_camera() {
  log "verifying the camera publishes on ${CAMERA_TOPIC} (up to ${CAMERA_WAIT}s for cold start)..."
  if ! wait_for_camera_frame; then
    err "no message on ${CAMERA_TOPIC} within ${CAMERA_WAIT}s — camera not publishing or bridge down (see ${LOG_DIR})"
    return 1
  fi
  local rate
  rate="$(timeout 8 ros2 topic hz "${CAMERA_TOPIC}" 2>&1 | grep -m1 -oE 'average rate: [0-9.]+' || true)"
  log "camera ${CAMERA_TOPIC} steady: ${rate:-(rate sample unavailable, but a frame arrived)}"
  if ros2 topic list 2>/dev/null | grep -qx "${CAMERA_TOPIC}/compressed"; then
    log "compressed companion present: ${CAMERA_TOPIC}/compressed"
  else
    warn "${CAMERA_TOPIC}/compressed not found — install ros-${ROS_DISTRO}-image-transport-plugins for the bag topic"
  fi
  return 0
}

fly_and_verify_patrol() {
  log "flying the M4 patrol over the stage (mission_patrol.launch.py, checkpoints=${CHECKPOINTS_YAML})"
  ros2 launch patrol_bringup mission_patrol.launch.py record:=false \
    "checkpoints_yaml:=${CHECKPOINTS_YAML}" >"${LOG_DIR}/node.log" 2>&1 &
  NODE_PID=$!
  log "verifying patrol acceptance (timeout ${VERIFY_TIMEOUT}s)..."
  python3 "${SCRIPT_DIR}/verify_patrol.py" --timeout "${VERIFY_TIMEOUT}"
}

report_keep_up() {
  trap - EXIT INT TERM
  log "stack left running (--keep-up):"
  log "  PIDs: agent=${AGENT_PID} gz=${GZ_PID} gui=${GZ_GUI_PID:-none} px4=${PX4_PID} qgc=${QGC_PID:-none} bridge=${BRIDGE_PID} node=${NODE_PID:-none}"
  log "  tear down: kill -- -${PX4_PID}; kill ${AGENT_PID} ${GZ_PID} ${GZ_GUI_PID:-} ${QGC_PID:-} ${BRIDGE_PID} ${NODE_PID:-}"
}

main() {
  parse_args "$@"
  if [[ -n "${LOG_DIR}" ]]; then
    mkdir -p "${LOG_DIR}"
  else
    LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/patrol-world-uat.XXXXXX")"
  fi
  export DOCTOR_PATROL_WORLD=1
  [[ ${WITH_GUI} -eq 0 ]] && export DOCTOR_HEADLESS=1
  prepare_agent_env
  if [[ ${SKIP_DOCTOR} -eq 0 ]]; then
    run_checks || { err "env_doctor failed — fix the above, or re-run with --skip-doctor"; exit 1; }
  fi
  source_ros
  export_gz_env

  trap shutdown INT TERM EXIT
  start_agent
  start_gz
  wait_for_world || { err "Gazebo world '${WORLD_NAME}' not ready (see ${LOG_DIR}/gz.log)"; exit 1; }
  spawn_model
  start_px4
  wait_for_bridge || { err "uXRCE-DDS bridge did not come up (see ${LOG_DIR}/px4.log + agent.log)"; exit 1; }
  start_qgc  # GCS heartbeat for the patrol's offboard arm; no-op for camera-only / headless runs
  start_camera_bridge

  set +e
  verify_camera
  local verdict=$?
  if [[ ${verdict} -eq 0 && ${RUN_PATROL} -eq 1 ]]; then
    fly_and_verify_patrol
    verdict=$?
  fi
  set -e

  [[ ${KEEP_UP} -eq 1 ]] && report_keep_up
  exit "${verdict}"
}

main "$@"
