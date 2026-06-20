#!/usr/bin/env bash
#
# env_doctor.sh — M3 UAT capability gate. Verifies the host can actually FLY the basic mission in
# SITL, not merely that packages are installed, and fails LOUD with the exact remedy for each item.
#
# It is the first layer of the Phase 1 UAT harness (Doctor -> Runner -> Verifier -> Runbook):
# run it yourself before a UAT session, or let scripts/run_sitl_mission.sh run it for you. The host
# bootstrap (scripts/setup_phase1.sh) also calls it in --smoke mode as a post-install sanity check.
#
# Checks (full mode):
#   - Micro XRCE-DDS Agent is RUNNABLE  : binary resolvable AND its shared lib loads (ldd)
#   - ROS 2 Jazzy is sourceable         : /opt/ros/jazzy/setup.bash present
#   - the ROS 2 workspace is built       : patrol_mission pkg + mission_basic/mission_patrol launch
#                                          + patrol_mission.yaml installed (the M4 artifacts the
#                                          runner needs; skipped by --smoke)
#   - the display server is X11          : Gazebo Harmonic GUI is unreliable under Wayland here
#   - QGroundControl is present          : ~/Apps/QGroundControl-x86_64.AppImage
#
# --smoke drops the workspace-built check: a built workspace is a milestone DELIVERABLE, not a host
# prerequisite (ADR-0003), so setup_phase1.sh must not "fail" on it right after provisioning.
#
# DOCTOR_HEADLESS=1 (exported by run_sitl_mission.sh --no-qgc) skips the display (X11) and QGC
# checks: a headless run launches Gazebo without a GUI and PX4 without a GCS, so neither is needed.
#
# Sourceable: when sourced (not executed), it only DEFINES functions — so run_sitl_mission.sh can
# reuse resolve_xrce_agent() / run_checks() without re-running the whole gate. No `set -u`: it may
# be sourced into a shell that later sources ROS's setup.bash (which references unbound vars).
set -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The pinned distro (stack-manifest.toml middleware.ros_distro / CLAUDE.md). The doctor checks the
# host against the contract, so this is the value to expect, not a runtime-discovered one.
ROS_DISTRO_EXPECTED="jazzy"
QGC_APPIMAGE="${HOME}/Apps/QGroundControl-x86_64.AppImage"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"

# Set by resolve_xrce_agent(): the agent binary and the dir holding libmicroxrcedds_agent.so*.
XRCE_AGENT_BIN=""
XRCE_AGENT_LIBDIR=""

# ----------------------------------------------------------------------------- reporting
ok()   { printf '  \033[1;32m[ ok ]\033[0m %s\n' "$*"; }
bad()  { printf '  \033[1;31m[FAIL]\033[0m %s\n' "$*" >&2; }
fix()  { printf '         \033[1;33mfix:\033[0m %s\n' "$*" >&2; }
hdr()  { printf '\033[1;34m[env_doctor]\033[0m %s\n' "$*"; }

# ----------------------------------------------------------------------------- XRCE-DDS agent
# resolve_xrce_agent — locate the agent binary + its shared-lib dir without printing. Returns 0 and
# sets XRCE_AGENT_BIN (+ XRCE_AGENT_LIBDIR if found) when a binary exists; 1 if none is found.
# Probes PATH, then the canonical /usr/local install (build_xrce_agent.sh), then the ~/.local layout.
resolve_xrce_agent() {
  XRCE_AGENT_BIN=""
  XRCE_AGENT_LIBDIR=""
  local cand
  for cand in \
    "$(command -v MicroXRCEAgent 2>/dev/null || true)" \
    "/usr/local/bin/MicroXRCEAgent" \
    "${HOME}/.local/bin/MicroXRCEAgent"; do
    if [[ -n "${cand}" && -x "${cand}" ]]; then
      XRCE_AGENT_BIN="${cand}"
      break
    fi
  done
  [[ -z "${XRCE_AGENT_BIN}" ]] && return 1

  # The agent's libs install as a sibling ../lib of its bin/ (build_xrce_agent.sh). Record that dir
  # iff it actually holds libmicroxrcedds_agent.so* — that's what LD_LIBRARY_PATH needs to point at.
  local bindir libdir
  bindir="$(dirname "${XRCE_AGENT_BIN}")"
  if libdir="$(cd "${bindir}/../lib" 2>/dev/null && pwd)" &&
    [[ -n "$(find "${libdir}" -maxdepth 1 -name 'libmicroxrcedds_agent.so*' -print -quit 2>/dev/null)" ]]; then
    XRCE_AGENT_LIBDIR="${libdir}"
  fi
  return 0
}

check_xrce_agent() {
  if ! resolve_xrce_agent; then
    bad "Micro XRCE-DDS Agent not found (looked on PATH, /usr/local/bin, ~/.local/bin)."
    fix "build + install it:  scripts/setup_phase1.sh   (re-run; the agent is the M2 bridge deliverable)"
    return 1
  fi

  local on_path=1 lib_ok=1
  command -v MicroXRCEAgent >/dev/null 2>&1 || on_path=0
  if ldd "${XRCE_AGENT_BIN}" 2>&1 | grep -q 'libmicroxrcedds.*not found'; then
    lib_ok=0
  fi
  if [[ ${on_path} -eq 1 && ${lib_ok} -eq 1 ]]; then
    ok "Micro XRCE-DDS Agent runnable: ${XRCE_AGENT_BIN}"
    return 0
  fi

  bad "Micro XRCE-DDS Agent present (${XRCE_AGENT_BIN}) but not runnable as-is."
  if [[ ${on_path} -eq 0 ]]; then
    fix "put it on PATH:        export PATH=\"$(dirname "${XRCE_AGENT_BIN}"):\$PATH\""
  fi
  if [[ ${lib_ok} -eq 0 ]]; then
    if [[ -n "${XRCE_AGENT_LIBDIR}" ]]; then
      fix "resolve its lib:      export LD_LIBRARY_PATH=\"${XRCE_AGENT_LIBDIR}:\$LD_LIBRARY_PATH\""
      fix "  (make it permanent: echo \"${XRCE_AGENT_LIBDIR}\" | sudo tee /etc/ld.so.conf.d/microxrce.conf && sudo ldconfig)"
    else
      fix "its shared lib (libmicroxrcedds_agent.so*) is missing — re-run scripts/setup_phase1.sh to rebuild the agent."
    fi
  fi
  return 1
}

# ----------------------------------------------------------------------------- ROS / workspace
check_ros() {
  local setup="/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
  if [[ -r "${setup}" ]]; then
    ok "ROS 2 ${ROS_DISTRO_EXPECTED} sourceable: ${setup}"
    return 0
  fi
  bad "ROS 2 ${ROS_DISTRO_EXPECTED} not found at ${setup}"
  fix "install it:  scripts/setup_phase1.sh    then:  source ${setup}"
  return 1
}

check_workspace() {
  local install_dir="${REPO_ROOT}/ros2_ws/install"
  local share="${install_dir}/patrol_bringup/share/patrol_bringup"
  # Check THIS milestone's installed artifacts, not merely that an `install/` tree exists: an older
  # M2/M3-only build has install/ but lacks the M4 patrol launch + route, so the doctor would pass
  # while `run_sitl_mission.sh --patrol` later fails resolving them (review #5). M4 flies
  # mission_patrol.launch.py with the checked-in patrol_mission.yaml route (the basic mission's
  # mission_basic.launch.py is still flown by the no-flag runner, so check both).
  local node_pkg="${install_dir}/patrol_mission"
  local basic_launch="${share}/launch/mission_basic.launch.py"
  local patrol_launch="${share}/launch/mission_patrol.launch.py"
  local patrol_route="${share}/config/patrol_mission.yaml"
  if [[ -d "${node_pkg}" && -f "${basic_launch}" && -f "${patrol_launch}" && -f "${patrol_route}" ]]; then
    ok "ROS 2 workspace built: patrol_mission + mission_basic/mission_patrol launch + patrol_mission.yaml installed (${install_dir})"
    return 0
  fi
  bad "ROS 2 workspace missing M4 artifacts (patrol_mission package, mission_basic/mission_patrol launch, and/or patrol_mission.yaml) — an older M2/M3-only build won't fly the patrol."
  fix "(re)build it (strip the uv venv first; do NOT prefix PYTHONPATH= — it breaks ament_package):"
  fix "  unset VIRTUAL_ENV"
  fix "  export PATH=\"\$(echo \"\$PATH\" | tr ':' '\\n' | grep -v '/patrol-drone/.venv/bin' | paste -sd ':')\""
  fix "  source /opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
  fix "  (cd ${REPO_ROOT}/ros2_ws && colcon build)"
  return 1
}

# ----------------------------------------------------------------------------- display / QGC
check_display() {
  if [[ "${DOCTOR_HEADLESS:-0}" -eq 1 ]]; then
    ok "display server check skipped (headless: no Gazebo GUI to render)"
    return 0
  fi
  local session="${XDG_SESSION_TYPE:-unknown}"
  if [[ "${session}" == "x11" ]]; then
    ok "display server is X11 (\$XDG_SESSION_TYPE=x11)"
    return 0
  fi
  bad "display server is '${session}', not x11 — Gazebo Harmonic rendering is unreliable under Wayland here."
  fix "force Xorg:  scripts/setup_phase1.sh --disable-wayland   then reboot and re-check:  echo \$XDG_SESSION_TYPE"
  return 1
}

check_qgc() {
  if [[ "${DOCTOR_HEADLESS:-0}" -eq 1 ]]; then
    ok "QGroundControl check skipped (headless: PX4 runs without a GCS in this path)"
    return 0
  fi
  if [[ -f "${QGC_APPIMAGE}" ]]; then
    ok "QGroundControl present: ${QGC_APPIMAGE}"
    return 0
  fi
  bad "QGroundControl AppImage not found at ${QGC_APPIMAGE}"
  fix "download it:  scripts/setup_phase1.sh   (or grab it from qgroundcontrol.com and place it there)"
  return 1
}

# --------------------------------------------------------- M5 patrol-world / camera (opt-in)
# These extra checks run only in patrol-world mode (--patrol-world, or DOCTOR_PATROL_WORLD=1 — set by
# run_patrol_world_sitl.sh). They are NOT part of the default gate: the camera bridge needs ROS
# packages (ros_gz_image / image_transport_plugins) that the basic M3/M4 missions don't, so requiring
# them by default would fail a host that can fly the patrol just fine.

# The M5 sim assets are checked-in repo files; also assert the generated world hasn't drifted from
# checkpoints.yaml (the same stdlib gate CI runs), so a stale world is caught before launch.
check_sim_assets() {
  local missing=0 f
  for f in \
    "${REPO_ROOT}/sim/worlds/patrol_world.sdf" \
    "${REPO_ROOT}/sim/px4_sitl_overrides/gz_x500_patrol/model.sdf" \
    "${REPO_ROOT}/sim/config/checkpoints.yaml" \
    "${REPO_ROOT}/sim/models/apriltag_36h11_0/model.sdf"; do
    [[ -f "${f}" ]] || { bad "M5 sim asset missing: ${f}"; missing=1; }
  done
  if [[ ${missing} -ne 0 ]]; then
    fix "M5 assets live under sim/ — re-checkout, or regenerate:  python3 sim/tools/compose_world.py"
    return 1
  fi
  if python3 "${REPO_ROOT}/scripts/check_world_drift.py" >/dev/null 2>&1; then
    ok "M5 sim assets present; patrol_world.sdf in sync with checkpoints.yaml"
    return 0
  fi
  bad "patrol_world.sdf / apriltag models have drifted from their sources"
  fix "regenerate:  python3 sim/tools/compose_world.py && python3 sim/tools/gen_apriltag_models.py"
  return 1
}

# gz_x500_patrol merge-includes PX4's stock x500 model (A2: a camera variant, not a fork), so the
# base model must exist in the PX4 tree.
check_px4_base_model() {
  local x500="${PX4_DIR}/Tools/simulation/gz/models/x500/model.sdf"
  if [[ -f "${x500}" ]]; then
    ok "PX4 x500 base model present (gz_x500_patrol merges it): ${x500}"
    return 0
  fi
  bad "PX4 x500 base model not found at ${x500} — gz_x500_patrol merge-includes it"
  fix "check out PX4-Autopilot v1.17.0 (PX4_DIR=${PX4_DIR}), or set PX4_DIR to your PX4 tree"
  return 1
}

# The M5 launch files must be installed into the workspace (built after M5 landed), or
# run_patrol_world_sitl.sh can't bring up the camera bridge / patrol-over-stage.
check_m5_workspace() {
  local share="${REPO_ROOT}/ros2_ws/install/patrol_bringup/share/patrol_bringup/launch"
  if [[ -f "${share}/camera_bridge.launch.py" && -f "${share}/patrol_world.launch.py" ]]; then
    ok "M5 launch files installed: camera_bridge.launch.py + patrol_world.launch.py"
    return 0
  fi
  bad "M5 launch files (camera_bridge.launch.py / patrol_world.launch.py) not installed — stale build."
  fix "rebuild the workspace (see the workspace-built fix above):  (cd ${REPO_ROOT}/ros2_ws && colcon build)"
  return 1
}

# The camera bridge needs ros_gz_image (image_bridge) + image_transport_plugins (the /compressed
# companion). Query in a subshell that sources ROS so the doctor's own env stays clean.
check_camera_bridge_deps() {
  local setup="/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
  if [[ ! -r "${setup}" ]]; then
    bad "ROS 2 ${ROS_DISTRO_EXPECTED} not sourceable — cannot check camera-bridge packages"
    fix "install ROS:  scripts/setup_phase1.sh"
    return 1
  fi
  local fails=0
  if bash -c "source '${setup}' >/dev/null 2>&1; ros2 pkg executables ros_gz_image 2>/dev/null" |
    grep -q "image_bridge"; then
    ok "ros_gz_image image_bridge available (gz camera -> ROS image topics)"
  else
    bad "ros_gz_image (image_bridge) not found — the camera topics can't be bridged"
    fix "install:  sudo apt install ros-${ROS_DISTRO_EXPECTED}-ros-gz-image"
    fails=1
  fi
  if bash -c "source '${setup}' >/dev/null 2>&1; ros2 pkg prefix compressed_image_transport >/dev/null 2>&1"; then
    ok "compressed_image_transport present (/drone/camera/image_raw/compressed companion)"
  else
    bad "image_transport_plugins (compressed_image_transport) not found — no /compressed companion"
    fix "install:  sudo apt install ros-${ROS_DISTRO_EXPECTED}-image-transport-plugins"
    fails=1
  fi
  return "${fails}"
}

# Aggregate the opt-in M5 checks; returns the failure count.
run_patrol_world_checks() {
  local fails=0
  hdr "M5 patrol-world / camera checks"
  check_sim_assets || fails=$((fails + 1))
  check_px4_base_model || fails=$((fails + 1))
  check_m5_workspace || fails=$((fails + 1))
  check_camera_bridge_deps || fails=$((fails + 1))
  return "${fails}"
}

# ----------------------------------------------------------------------------- driver
# run_checks [smoke] — run every check, aggregate failures, print a summary. smoke=1 skips the
# workspace-built check (a milestone deliverable, not a host prerequisite). When DOCTOR_PATROL_WORLD=1
# (the --patrol-world flag, or set by run_patrol_world_sitl.sh) the opt-in M5 camera/world checks are
# added. Returns 0 iff all pass.
run_checks() {
  local smoke="${1:-0}" fails=0 label=""
  [[ "${smoke}" -eq 1 ]] && label=" (smoke)"
  [[ "${DOCTOR_PATROL_WORLD:-0}" -eq 1 ]] && label="${label} (patrol-world)"
  hdr "patrol-drone host capability check${label}"
  check_xrce_agent || fails=$((fails + 1))
  check_ros || fails=$((fails + 1))
  check_display || fails=$((fails + 1))
  check_qgc || fails=$((fails + 1))
  if [[ "${smoke}" -eq 1 ]]; then
    hdr "smoke mode: skipping the workspace-built check (a milestone deliverable, not a prerequisite)"
  else
    check_workspace || fails=$((fails + 1))
  fi
  if [[ "${DOCTOR_PATROL_WORLD:-0}" -eq 1 ]]; then
    run_patrol_world_checks
    fails=$((fails + $?))
  fi
  echo
  if [[ ${fails} -eq 0 ]]; then
    ok "all checks passed — this host can fly the M3 basic + M4 patrol missions"
    return 0
  fi
  bad "${fails} check(s) failed — apply the fixes above, then re-run scripts/env_doctor.sh"
  return 1
}

usage() {
  cat <<'EOF'
Usage: scripts/env_doctor.sh [--smoke] [--patrol-world]

Verifies the host can fly the M3 basic + M4 patrol missions in SITL and prints the fix for each failure.

  --smoke          Prerequisite checks only (skip the workspace-built check). Used by setup_phase1.sh.
  --patrol-world   Also run the M5 camera/world checks (sim assets + drift, PX4 x500 base model,
                   M5 launch files installed, ros_gz_image / image_transport_plugins). Used by
                   run_patrol_world_sitl.sh.
  -h, --help       Show this help.
EOF
}

main() {
  local smoke=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --smoke) smoke=1 ;;
      --patrol-world) export DOCTOR_PATROL_WORLD=1 ;;
      -h | --help) usage; return 0 ;;
      *) bad "unknown option: $1"; usage; return 2 ;;
    esac
    shift
  done
  run_checks "${smoke}"
}

# Run main() only when executed directly; when sourced, expose the functions (resolve_xrce_agent,
# run_checks, ...) for reuse without running the gate — mirrors setup_phase1.sh.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi
