#!/usr/bin/env bash
#
# setup_phase1.sh — Host bootstrap for patrol-drone Phase 1 (full prerequisite toolchain).
#
# Place at: scripts/setup_phase1.sh   (it locates the repo root as its parent dir)
#
# Scope — installs the full *Phase 1* prerequisite toolchain on Ubuntu 24.04 so a fresh host
# can build and run every Phase 1 milestone (M1–M8). Installing a toolchain is NOT the same as
# doing milestone work — this script provisions prerequisites, not deliverables:
#   - base build/dev apt packages
#   - ROS 2 Jazzy (desktop) + colcon/rosdep + ROS dev tools
#   - ROS runtime packages later milestones need (rosbag2-MCAP, apriltag_ros, ros-gz, cv_bridge)
#   - Micro XRCE-DDS Agent (PX4 <-> ROS 2 bridge), via the ROS 2 apt repo
#   - Docker Engine + Compose (+ optional NVIDIA Container Toolkit with --with-nvidia)
#   - uv (Python manager) and the project's dev venv from pyproject.toml
#   - PX4-Autopilot source checkout (pinned) + PX4's own ubuntu.sh dev-env setup
#   - QGroundControl + Foxglove Studio desktop apps
#
# Deliberately NOT done here — these are repo *deliverables* owned by individual milestones
# (they live in git, produced via the /devloop lifecycle), not host prerequisites:
#   - Vendoring px4_msgs / px4_ros_com into ros2_ws/src/external/   -> 01-platform (M2)
#   - Writing docker/{sim,dev} Dockerfiles + docker compose         -> 01-platform (M2)
#   - `colcon build` of the ros2_ws workspace                       -> per-milestone build
#   - `uv add` of per-milestone Python deps (pyyaml, mcap, ...)     -> land in pyproject.toml
#   - `make px4_sitl gz_x500` launch + 60s hover                    -> manual M1 exit test
#   - Switching the live session to Xorg                            -> login choice (--disable-wayland)
#   - NVIDIA driver install by default                              -> display risk (--with-nvidia)
#
# See docs/decisions/0003-phase1-bootstrap-scope.md for the prerequisites-vs-deliverables call.
#
# Idempotent: safe to re-run. Full install by default; opt out of sections with --skip-* flags.
# Uses sudo for system steps; will prompt.

set -euo pipefail
trap 'err "failed at line ${LINENO}"; exit 1' ERR

# ----------------------------------------------------------------------------- repo root + manifest
# Resolve the repo root from this script's location so the canonical pinned-stack manifest is
# the single source of truth for every version literal below — nothing is hardcoded here (ADR-0004).
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${REPO_ROOT}/stack-manifest.toml"

# manifest_get <dotted.key> — read one value from stack-manifest.toml (stdlib tomllib, Python 3.11+).
# Fails (non-zero) on a missing key, which aborts the deriving assignments below under `set -e`.
manifest_get() {
  python3 - "$1" "${MANIFEST}" <<'PY'
import sys, tomllib
key, path = sys.argv[1], sys.argv[2]
with open(path, "rb") as fh:
    node = tomllib.load(fh)
for part in key.split("."):
    node = node[part]
print(node)
PY
}

# ----------------------------------------------------------------------------- config / flags
# Version literals are DERIVED from stack-manifest.toml — do not hardcode them here (ADR-0004 / ADR-0005).
PX4_VERSION="$(manifest_get flight_stack.px4_version)"
PX4_COMMIT="$(manifest_get flight_stack.px4_commit)"   # cleared when --px4-version overrides the pinned tag
ROS_DISTRO="$(manifest_get middleware.ros_distro)"
UV_VERSION="$(manifest_get tools.uv_version)"
QGC_VERSION="$(manifest_get apps.qgc_version)"
QGC_URL="$(manifest_get apps.qgc_url)"
QGC_SHA256="$(manifest_get apps.qgc_sha256)"
FOXGLOVE_VERSION="$(manifest_get apps.foxglove_version)"
FOXGLOVE_URL="$(manifest_get apps.foxglove_url)"
FOXGLOVE_SHA256="$(manifest_get apps.foxglove_sha256)"
PX4_DIR="${HOME}/PX4-Autopilot"
QGC_DIR="${HOME}/Apps"

WITH_NVIDIA=0
DISABLE_WAYLAND=0
PX4_NO_NUTTX=0
SKIP_PX4=0
SKIP_PYTHON=0
SKIP_ROS=0
SKIP_ROS_PKGS=0
SKIP_XRCE=0
SKIP_DOCKER=0
SKIP_QGC=0
SKIP_FOXGLOVE=0

NEED_REBOOT=0
NEED_RELOGIN=0

# ----------------------------------------------------------------------------- logging
log()  { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m  %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[err]\033[0m   %s\n' "$*" >&2; }
have() { command -v "$1" >/dev/null 2>&1; }

# verify_sha256 <file> <expected-sha256> — abort (and delete the file) on mismatch so a tampered
# or wrong-version download is never installed/used. Empty expected -> warn-and-skip (unpinned).
verify_sha256() {
  local file="$1" expected="$2"
  if [[ -z "${expected}" ]]; then
    warn "No checksum pinned for ${file##*/}; skipping verification."
    return 0
  fi
  if echo "${expected}  ${file}" | sha256sum -c - >/dev/null 2>&1; then
    log "Checksum OK: ${file##*/}"
    return 0
  fi
  err "Checksum MISMATCH for ${file} (expected ${expected}). Refusing to use it."
  rm -f "${file}"
  return 1
}

usage() {
  cat <<'EOF'
Usage: scripts/setup_phase1.sh [options]

Installs the full Phase 1 prerequisite toolchain by default. Use --skip-* to opt out.

  --with-nvidia        Install the proprietary NVIDIA driver (ubuntu-drivers autoinstall)
                       AND the NVIDIA Container Toolkit for GPU passthrough into containers.
                       Requires a reboot afterwards. Off by default (display-break risk).
  --disable-wayland    Set GDM to default to Xorg (edits /etc/gdm3/custom.conf).
                       Takes effect on next login. Off by default.
  --px4-no-nuttx       Pass --no-nuttx to PX4's ubuntu.sh (sim-only; skips Pixhawk toolchain).
                       Leaner for Phase 1. Verify the flag exists for your PX4 version.
  --px4-version <tag>  PX4 tag to check out (default: stack-manifest.toml flight_stack.px4_version).
                       Overriding this also disables the pinned-commit verification.
  --px4-dir <path>     Where to clone PX4-Autopilot (default: ~/PX4-Autopilot).

  --skip-ros           Skip ROS 2 Jazzy + colcon/rosdep install.
  --skip-ros-pkgs      Skip the ROS runtime packages (rosbag2-MCAP, apriltag, ros-gz, ...).
  --skip-xrce          Skip the Micro XRCE-DDS Agent (apt) install.
  --skip-docker        Skip Docker Engine + Compose install.
  --skip-python        Skip uv install + `uv sync`.
  --skip-px4           Skip PX4 clone + ubuntu.sh.
  --skip-qgc           Skip the QGroundControl AppImage download.
  --skip-foxglove      Skip the Foxglove Studio install.
  -h, --help           Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-nvidia)     WITH_NVIDIA=1 ;;
    --disable-wayland) DISABLE_WAYLAND=1 ;;
    --px4-no-nuttx)    PX4_NO_NUTTX=1 ;;
    --px4-version)     PX4_VERSION="${2:?--px4-version needs a value}"; PX4_COMMIT=""; shift ;;
    --px4-dir)         PX4_DIR="${2:?--px4-dir needs a value}"; shift ;;
    --skip-ros)        SKIP_ROS=1 ;;
    --skip-ros-pkgs)   SKIP_ROS_PKGS=1 ;;
    --skip-xrce)       SKIP_XRCE=1 ;;
    --skip-docker)     SKIP_DOCKER=1 ;;
    --skip-python)     SKIP_PYTHON=1 ;;
    --skip-px4)        SKIP_PX4=1 ;;
    --skip-qgc)        SKIP_QGC=1 ;;
    --skip-foxglove)   SKIP_FOXGLOVE=1 ;;
    -h|--help)         usage; exit 0 ;;
    *) err "unknown option: $1"; usage; exit 2 ;;
  esac
  shift
done

# ----------------------------------------------------------------------------- preflight
preflight() {
  if [[ ${EUID} -eq 0 ]]; then
    err "Run as your normal user (not root/sudo). The script calls sudo only where needed."
    exit 1
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${VERSION_ID:-}" != "24.04" ]]; then
      warn "Detected Ubuntu '${VERSION_ID:-unknown}'. This script targets 24.04. Continuing anyway."
    fi
  fi

  local session="${XDG_SESSION_TYPE:-unknown}"
  if [[ "${session}" == "wayland" ]]; then
    warn "Current session is Wayland. Gazebo ('make px4_sitl gz_x500') often fails to render here."
    warn "NOTE (Ubuntu 24.04): there is NO separate 'Ubuntu on Xorg' entry to pick at the login"
    warn "screen -- GDM folds it into plain 'Ubuntu'. Don't hunt for a menu option; the reliable"
    warn "fix is to re-run with --disable-wayland, then reboot and confirm with:"
    warn "echo \$XDG_SESSION_TYPE   (expect: x11)."
  fi

  log "Caching sudo credentials..."
  sudo -v
}

# ----------------------------------------------------------------------------- apt base deps
install_base_packages() {
  log "Installing base build/dev packages..."
  sudo apt-get update -y
  sudo apt-get install -y \
    git curl wget gnupg ca-certificates lsb-release software-properties-common \
    build-essential cmake ninja-build \
    python3-venv python3-pip \
    mesa-utils
}

# ----------------------------------------------------------------------------- ROS 2 Jazzy
install_ros2_jazzy() {
  [[ ${SKIP_ROS} -eq 1 ]] && { log "Skipping ROS 2 Jazzy (--skip-ros)."; return 0; }

  if have ros2 || dpkg -l ros-${ROS_DISTRO}-desktop 2>/dev/null | grep -q '^ii'; then
    log "ROS 2 ${ROS_DISTRO} already installed."
  else
    log "Adding the ROS 2 apt source and installing ROS 2 ${ROS_DISTRO} (desktop)..."
    sudo add-apt-repository -y universe
    # Modern apt-source method: a versioned .deb that drops in the key + repo list.
    local codename ros_apt_ver deb
    # shellcheck disable=SC1091
    codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
    ros_apt_ver="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                   | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    deb="/tmp/ros2-apt-source.deb"
    curl -fL -o "${deb}" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_ver}/ros2-apt-source_${ros_apt_ver}.${codename}_all.deb"
    sudo apt-get install -y "${deb}"
    rm -f "${deb}"
    sudo apt-get update -y
    sudo apt-get install -y \
      ros-${ROS_DISTRO}-desktop ros-dev-tools \
      python3-colcon-common-extensions python3-rosdep python3-vcstool
  fi

  # rosdep: init once (system), then update (per-user, no sudo).
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initialising rosdep..."
    sudo rosdep init
  fi
  log "Updating rosdep database..."
  rosdep update || warn "rosdep update failed (network?); re-run later with: rosdep update"

  # Convenience: source ROS in interactive shells (idempotent, guarded by marker).
  local marker="# >>> patrol-drone ROS 2 ${ROS_DISTRO} >>>"
  if ! grep -qF "${marker}" "${HOME}/.bashrc" 2>/dev/null; then
    log "Adding 'source /opt/ros/${ROS_DISTRO}/setup.bash' to ~/.bashrc..."
    {
      echo ""
      echo "${marker}"
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
      echo "# <<< patrol-drone ROS 2 ${ROS_DISTRO} <<<"
    } >> "${HOME}/.bashrc"
  fi
}

# ----------------------------------------------------------------------------- ROS runtime packages
install_ros_packages() {
  [[ ${SKIP_ROS_PKGS} -eq 1 ]] && { log "Skipping ROS runtime packages (--skip-ros-pkgs)."; return 0; }
  if [[ ${SKIP_ROS} -eq 1 ]] && ! have ros2; then
    warn "ROS not installed (--skip-ros) — skipping ROS runtime packages too."
    return 0
  fi
  log "Installing ROS runtime packages used by later Phase 1 milestones..."
  # NB: plan text says ros-humble-rosbag2-storage-mcap — that's a distro typo; we use jazzy.
  sudo apt-get install -y \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-apriltag ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz-image
}

# ----------------------------------------------------------------------------- Micro XRCE-DDS Agent
install_xrce_agent() {
  [[ ${SKIP_XRCE} -eq 1 ]] && { log "Skipping Micro XRCE-DDS Agent (--skip-xrce)."; return 0; }
  # Installed from the ROS 2 apt repo (configured by install_ros2_jazzy) so the host
  # matches the sim container, which apt-installs ros-${ROS_DISTRO}-micro-xrce-dds-agent
  # (docs/phase1/01-platform/design.md §4.2.1). Provides the `MicroXRCEAgent` binary.
  if [[ ${SKIP_ROS} -eq 1 ]] && ! have ros2; then
    warn "ROS not installed (--skip-ros) — the agent's apt repo is unavailable. Skipping XRCE agent."
    return 0
  fi
  if dpkg -l ros-${ROS_DISTRO}-micro-xrce-dds-agent 2>/dev/null | grep -q '^ii'; then
    log "Micro XRCE-DDS Agent (ros-${ROS_DISTRO}-micro-xrce-dds-agent) already installed."
    return 0
  fi
  log "Installing Micro XRCE-DDS Agent (ros-${ROS_DISTRO}-micro-xrce-dds-agent)..."
  sudo apt-get install -y ros-${ROS_DISTRO}-micro-xrce-dds-agent
}

# ----------------------------------------------------------------------------- Docker Engine + Compose
install_docker() {
  [[ ${SKIP_DOCKER} -eq 1 ]] && { log "Skipping Docker (--skip-docker)."; return 0; }

  if have docker; then
    log "Docker already installed ($(docker --version 2>/dev/null))."
  else
    log "Installing Docker Engine + Compose from the official Docker apt repo..."
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    local codename arch
    # shellcheck disable=SC1091
    codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
    arch="$(dpkg --print-architecture)"
    echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${codename} stable" \
      | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update -y
    sudo apt-get install -y \
      docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  fi

  if id -nG "${USER}" | grep -qw docker; then
    log "User already in 'docker' group."
  else
    log "Adding ${USER} to 'docker' group (lets you run docker without sudo)..."
    sudo usermod -aG docker "${USER}"
    NEED_RELOGIN=1
  fi
}

# ----------------------------------------------------------------------------- QGroundControl prereqs + app
install_qgc_prereqs() {
  log "Installing QGroundControl runtime prerequisites..."
  sudo apt-get install -y \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0

  if id -nG "${USER}" | grep -qw dialout; then
    log "User already in 'dialout' group."
  else
    log "Adding ${USER} to 'dialout' group..."
    sudo usermod -aG dialout "${USER}"
    NEED_RELOGIN=1
  fi

  if dpkg -l 2>/dev/null | grep -q '^ii  *modemmanager'; then
    log "Removing modemmanager (interferes with serial; harmless to remove for SITL)..."
    sudo apt-get remove -y modemmanager
  else
    log "modemmanager not installed — nothing to remove."
  fi
}

download_qgc() {
  [[ ${SKIP_QGC} -eq 1 ]] && { log "Skipping QGroundControl (--skip-qgc)."; return 0; }
  mkdir -p "${QGC_DIR}"
  local target="${QGC_DIR}/QGroundControl-x86_64.AppImage"
  if [[ -f "${target}" ]] && verify_sha256 "${target}" "${QGC_SHA256}"; then
    log "QGC AppImage already present and verified at ${target}."
    chmod +x "${target}"   # ensure it's runnable even if placed here manually
    return 0
  fi
  log "Downloading QGroundControl ${QGC_VERSION}..."
  if curl -fL -o "${target}" "${QGC_URL}"; then
    verify_sha256 "${target}" "${QGC_SHA256}" || return 1   # abort setup on a tampered/wrong asset
    chmod +x "${target}"
    log "QGC saved to ${target}"
  else
    warn "QGC download failed (asset URL may have changed). Grab it from qgroundcontrol.com manually."
    rm -f "${target}"
  fi
}

# ----------------------------------------------------------------------------- Foxglove Studio
install_foxglove() {
  [[ ${SKIP_FOXGLOVE} -eq 1 ]] && { log "Skipping Foxglove Studio (--skip-foxglove)."; return 0; }
  if have foxglove-studio || dpkg -l foxglove-studio 2>/dev/null | grep -q '^ii'; then
    log "Foxglove Studio already installed."
    return 0
  fi
  log "Downloading and installing Foxglove Studio ${FOXGLOVE_VERSION}..."
  local deb="/tmp/foxglove-studio.deb"
  if curl -fL -o "${deb}" "${FOXGLOVE_URL}"; then
    verify_sha256 "${deb}" "${FOXGLOVE_SHA256}" || return 1   # abort setup on a tampered/wrong asset
    sudo apt-get install -y "${deb}"
    rm -f "${deb}"
    log "Foxglove Studio ${FOXGLOVE_VERSION} installed."
  else
    warn "Foxglove download failed. Grab the .deb from foxglove.dev/download manually."
    rm -f "${deb}"
  fi
}

# ----------------------------------------------------------------------------- optional: NVIDIA driver + container toolkit
install_nvidia() {
  [[ ${WITH_NVIDIA} -eq 1 ]] || {
    # Robust detection: nvidia-smi is unambiguous; fall back to lsmod (full path, since
    # /usr/sbin may be off a non-interactive PATH and produce a false "not loaded" warning).
    if ! have nvidia-smi && ! { /usr/sbin/lsmod 2>/dev/null || lsmod 2>/dev/null; } | grep -q '^nvidia'; then
      warn "NVIDIA driver not detected. Gazebo needs the proprietary driver."
      warn "Re-run with --with-nvidia, or install via 'Additional Drivers', then reboot."
    fi
    return 0
  }
  log "Installing proprietary NVIDIA driver (ubuntu-drivers autoinstall)..."
  sudo apt-get install -y ubuntu-drivers-common
  sudo ubuntu-drivers autoinstall
  NEED_REBOOT=1

  install_nvidia_container_toolkit
}

install_nvidia_container_toolkit() {
  if [[ ${SKIP_DOCKER} -eq 1 ]]; then
    warn "Docker skipped (--skip-docker) — skipping NVIDIA Container Toolkit too."
    return 0
  fi
  if dpkg -l nvidia-container-toolkit 2>/dev/null | grep -q '^ii'; then
    log "NVIDIA Container Toolkit already installed."
    return 0
  fi
  log "Installing the NVIDIA Container Toolkit (GPU passthrough for the sim/dev containers)..."
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
  sudo apt-get update -y
  sudo apt-get install -y nvidia-container-toolkit
  if have docker; then
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker || warn "Could not restart docker; restart it after reboot."
  fi
}

# ----------------------------------------------------------------------------- optional: default to Xorg
disable_wayland() {
  [[ ${DISABLE_WAYLAND} -eq 1 ]] || return 0
  local gdm=/etc/gdm3/custom.conf
  if [[ ! -f "${gdm}" ]]; then
    warn "${gdm} not found (GDM may not be the display manager). Skipping Wayland change."
    return 0
  fi
  log "Setting GDM to default to Xorg in ${gdm}..."
  sudo cp "${gdm}" "${gdm}.bak.$(date +%s)"
  if grep -qE '^\s*#?\s*WaylandEnable' "${gdm}"; then
    sudo sed -i -E 's/^\s*#?\s*WaylandEnable\s*=.*/WaylandEnable=false/' "${gdm}"
  else
    sudo sed -i '/^\[daemon\]/a WaylandEnable=false' "${gdm}"
  fi
  NEED_REBOOT=1
}

# ----------------------------------------------------------------------------- uv + python project
install_python_env() {
  [[ ${SKIP_PYTHON} -eq 1 ]] && { log "Skipping Python env (--skip-python)."; return 0; }

  if have uv; then
    log "uv already installed ($(uv --version))."
  else
    log "Installing uv ${UV_VERSION} (version-pinned installer)..."
    curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" | sh
  fi
  export PATH="${HOME}/.local/bin:${PATH}"

  if [[ -f "${REPO_ROOT}/pyproject.toml" ]]; then
    log "Syncing project venv with uv (from ${REPO_ROOT}/pyproject.toml)..."
    ( cd "${REPO_ROOT}" && uv sync )
  else
    warn "No pyproject.toml at repo root (${REPO_ROOT}); skipping 'uv sync'."
  fi
}

# ----------------------------------------------------------------------------- PX4 source + dev env
install_px4() {
  [[ ${SKIP_PX4} -eq 1 ]] && { log "Skipping PX4 (--skip-px4)."; return 0; }

  if [[ -d "${PX4_DIR}/.git" ]]; then
    log "PX4-Autopilot already cloned at ${PX4_DIR}."
  else
    log "Cloning PX4-Autopilot into ${PX4_DIR}..."
    git clone https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}" --recursive
  fi

  log "Checking out ${PX4_VERSION} and syncing submodules to the tag..."
  ( cd "${PX4_DIR}"
    git fetch --tags --quiet
    git checkout "${PX4_VERSION}"
    git submodule update --init --recursive )

  # Supply-chain check: the tag must dereference to the commit pinned in the manifest. Catches an
  # upstream-moved/retagged release. Skipped when --px4-version overrides the pinned tag (PX4_COMMIT="").
  if [[ -n "${PX4_COMMIT}" ]]; then
    local head
    head="$(git -C "${PX4_DIR}" rev-parse HEAD)"
    if [[ "${head}" == "${PX4_COMMIT}" ]]; then
      log "PX4 HEAD matches the pinned commit (${PX4_COMMIT})."
    else
      err "PX4 HEAD ${head} != pinned ${PX4_COMMIT} (stack-manifest.toml flight_stack.px4_commit)."
      err "The ${PX4_VERSION} tag may have moved upstream. Verify, then update the manifest pin. Aborting."
      exit 1
    fi
  fi

  log "Running PX4's ubuntu.sh dev-environment setup (this is the long step)..."
  local px4_args=()
  [[ ${PX4_NO_NUTTX} -eq 1 ]] && px4_args+=(--no-nuttx)
  ( cd "${PX4_DIR}" && bash ./Tools/setup/ubuntu.sh "${px4_args[@]}" )
  NEED_REBOOT=1
}

# ----------------------------------------------------------------------------- summary
print_next_steps() {
  echo
  log "===================== Phase 1 host setup complete ====================="
  echo
  echo "Remaining manual steps:"
  echo
  if [[ ${NEED_REBOOT} -eq 1 ]]; then
    echo "  1. Reboot now (PX4 ubuntu.sh / driver / GDM changes need it)."
  elif [[ ${NEED_RELOGIN} -eq 1 ]]; then
    echo "  1. Log out and back in (docker/dialout group membership)."
  else
    echo "  1. (No reboot/relogin flagged — but harmless to do one.)"
  fi
  if [[ ${DISABLE_WAYLAND} -eq 0 ]]; then
    echo "  2. Confirm the session is Xorg (Gazebo rendering needs it). On Ubuntu 24.04 there"
    echo "     is NO separate 'Ubuntu on Xorg' entry to pick at the login screen -- GDM folds"
    echo "     it into plain 'Ubuntu', which may resolve to either backend. So don't look for"
    echo "     a menu option; just check with the verify below. If it reports 'wayland', force"
    echo "     Xorg by re-running:  scripts/setup_phase1.sh --disable-wayland  then reboot."
  else
    echo "  2. GDM now defaults to Xorg (took effect this login cycle)."
  fi
  echo "     Verify:  echo \$XDG_SESSION_TYPE        # expect: x11"
  echo "              glxinfo | grep 'OpenGL renderer'   # expect: your NVIDIA GPU"
  echo
  echo "  3. Source ROS 2 (added to ~/.bashrc; new shells get it automatically):"
  echo "       source /opt/ros/${ROS_DISTRO}/setup.bash"
  echo
}

main() {
  preflight
  install_base_packages
  install_ros2_jazzy
  install_ros_packages
  install_xrce_agent
  install_docker
  install_qgc_prereqs
  download_qgc
  install_foxglove
  install_nvidia        # driver + container toolkit (only with --with-nvidia)
  disable_wayland
  install_python_env
  install_px4
  print_next_steps
}

main "$@"
