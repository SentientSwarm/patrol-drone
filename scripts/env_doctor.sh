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
#   - the ROS 2 workspace is built       : patrol_mission pkg + mission_basic.launch.py installed
#                                          (the M3 artifacts the runner needs; skipped by --smoke)
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
  # Check THIS milestone's installed artifacts, not merely that an `install/` tree exists: an older
  # M2-only build has install/ but no patrol_mission package or mission_basic.launch.py, so the
  # doctor would pass while the runner later fails resolving the package/launch (review #5).
  local node_pkg="${install_dir}/patrol_mission"
  local launch="${install_dir}/patrol_bringup/share/patrol_bringup/launch/mission_basic.launch.py"
  if [[ -d "${node_pkg}" && -f "${launch}" ]]; then
    ok "ROS 2 workspace built: patrol_mission + mission_basic.launch.py installed (${install_dir})"
    return 0
  fi
  bad "ROS 2 workspace missing M3 artifacts (patrol_mission package and/or mission_basic.launch.py) — an older M2-only build won't fly the mission."
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

# ----------------------------------------------------------------------------- driver
# run_checks [smoke] — run every check, aggregate failures, print a summary. smoke=1 skips the
# workspace-built check (a milestone deliverable, not a host prerequisite). Returns 0 iff all pass.
run_checks() {
  local smoke="${1:-0}" fails=0 label=""
  [[ "${smoke}" -eq 1 ]] && label=" (smoke)"
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
  echo
  if [[ ${fails} -eq 0 ]]; then
    ok "all checks passed — this host can fly the M3 basic mission"
    return 0
  fi
  bad "${fails} check(s) failed — apply the fixes above, then re-run scripts/env_doctor.sh"
  return 1
}

usage() {
  cat <<'EOF'
Usage: scripts/env_doctor.sh [--smoke]

Verifies the host can fly the M3 basic mission in SITL and prints the exact fix for each failure.

  --smoke      Prerequisite checks only (skip the workspace-built check). Used by setup_phase1.sh.
  -h, --help   Show this help.
EOF
}

main() {
  local smoke=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --smoke) smoke=1 ;;
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
