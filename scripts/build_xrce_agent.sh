#!/usr/bin/env bash
# Build + install the Micro XRCE-DDS Agent from source at a pinned eProsima tag.
#
# Single source of the build recipe (ADR-0007): both the host bootstrap
# (scripts/setup_phase1.sh::install_xrce_agent) and the sim container
# (docker/sim/Dockerfile runtime stage) invoke THIS script, so the host and container
# agents are byte-identical and can't drift. The version/commit/source are the manifest
# pins (stack-manifest.toml [bridge]); the caller passes them in.
#
# The cmake SUPERBUILD fetches+builds Fast-DDS/Fast-CDR into build/temp_install and emits the
# MicroXRCEAgent binary + libmicroxrcedds_agent.so in build/ — it has NO top-level `install`
# target. So we install the binary + ALL shared libs under the build tree into /usr/local and
# ldconfig; the agent then resolves its libs via the cache after the build tree is removed.
#
# Usage: build_xrce_agent.sh <source-url> <version-tag> <expected-commit> [sudo]
#   arg4 "sudo" → prefix the /usr/local install steps with sudo (host); omit in Docker (root).
set -eo pipefail

SOURCE="${1:?source url required}"
VERSION="${2:?version tag required}"
COMMIT="${3:?expected commit required}"
SUDO="${4:+sudo}"   # non-empty 4th arg → "sudo"; empty → "" (run as current user/root)

src="$(mktemp -d)"
trap 'rm -rf "${src}"' EXIT

git clone --depth 1 --branch "${VERSION}" "${SOURCE}" "${src}"
# Verify the tag dereferences to the pinned commit (catches an upstream-moved tag).
test "$(git -C "${src}" rev-parse HEAD)" = "${COMMIT}"

cmake -S "${src}" -B "${src}/build" \
    -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "${src}/build" -j"$(nproc)"

# Verify the superbuild's TRANSITIVE deps before installing them (Hermes Medium #3). The cmake
# superbuild fetches Fast-CDR/Fast-DDS/foonathan_memory/spdlog by upstream ref (two are MOVING
# branches) and installs their .so into /usr/local; an unpinned ref could change installed code
# without tripping manifest drift. Each EXPECT_<dep>_COMMIT is the manifest pin (stack-manifest.toml
# [bridge]); empty = not pinned -> skipped. A dep satisfied by a system package is not fetched (no
# checkout) and is skipped with a note. Fail CLOSED on a mismatch — we've built but refuse to install.
verify_transitive() {
  local url="$1" expected="$2" name="$3" dir actual
  [[ -z "${expected}" ]] && return 0
  # Find this dep's checkout among the superbuild's fetched repos by matching the clone URL
  # (robust to the ExternalProject prefix layout, which varies across superbuild versions).
  dir=""
  while IFS= read -r gitdir; do
    local d url_actual
    d="$(dirname "${gitdir}")"
    url_actual="$(git -C "${d}" config --get remote.origin.url 2>/dev/null || true)"
    if [[ "${url_actual%.git}" == "${url%.git}" ]]; then dir="${d}"; break; fi
  done < <(find "${src}/build" -name .git 2>/dev/null)
  if [[ -z "${dir}" ]]; then
    echo "[xrce] NOTE: ${name} not fetched by the superbuild (system-satisfied?) — skipping pin check" >&2
    return 0
  fi
  actual="$(git -C "${dir}" rev-parse HEAD)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "[xrce] ERROR: ${name} transitive pin MISMATCH — fetched ${actual}, manifest pins ${expected}." >&2
    echo "[xrce]   The upstream ref moved or was tampered. Re-resolve and bump stack-manifest.toml" >&2
    echo "[xrce]   [bridge] (git ls-remote <repo> <ref>), then rebuild. Refusing to install." >&2
    return 1
  fi
  echo "[xrce] OK: ${name} @ ${actual} matches the manifest pin." >&2
}
verify_transitive "https://github.com/eProsima/Fast-CDR.git" "${EXPECT_FASTCDR_COMMIT:-}"   "Fast-CDR"
verify_transitive "https://github.com/eProsima/Fast-DDS.git" "${EXPECT_FASTDDS_COMMIT:-}"   "Fast-DDS"
verify_transitive "https://github.com/foonathan/memory.git"  "${EXPECT_FOONATHAN_COMMIT:-}" "foonathan_memory"
verify_transitive "https://github.com/gabime/spdlog.git"     "${EXPECT_SPDLOG_COMMIT:-}"    "spdlog"

${SUDO} install -m755 "${src}/build/MicroXRCEAgent" /usr/local/bin/MicroXRCEAgent
${SUDO} find "${src}/build" -name "*.so*" -exec cp -a {} /usr/local/lib/ \;
${SUDO} ldconfig

# Record the installed commit so a rerun (host setup_phase1.sh::install_xrce_agent) can verify the
# on-disk agent against the manifest pin instead of trusting any MicroXRCEAgent on PATH (Hermes
# Medium #2). Same marker path on the host and in the sim container — they install the same tag.
${SUDO} install -d -m755 /usr/local/share/patrol-drone
printf '%s\n' "${COMMIT}" | ${SUDO} tee /usr/local/share/patrol-drone/xrce-agent.commit > /dev/null
