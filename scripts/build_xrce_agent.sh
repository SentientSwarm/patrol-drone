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

${SUDO} install -m755 "${src}/build/MicroXRCEAgent" /usr/local/bin/MicroXRCEAgent
${SUDO} find "${src}/build" -name "*.so*" -exec cp -a {} /usr/local/lib/ \;
${SUDO} ldconfig
