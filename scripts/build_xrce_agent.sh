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
#   Optional env (the transitive superbuild pins, stack-manifest.toml [bridge]): EXPECT_<DEP>_REF +
#   EXPECT_<DEP>_COMMIT for DEP in FASTCDR/FASTDDS/FOONATHAN/SPDLOG. The _REF enables the PRE-BUILD
#   ls-remote gate; the _COMMIT enables the POST-BUILD checkout gate. A pin with an empty value is
#   skipped (gate degrades gracefully). The caller (setup_phase1.sh / Dockerfile) supplies them.
set -eo pipefail

SOURCE="${1:?source url required}"
VERSION="${2:?version tag required}"
COMMIT="${3:?expected commit required}"
# arg4 non-empty → prefix /usr/local installs with sudo (host); empty (Docker/root) → no prefix.
# An ARRAY, not a ""${sudo_cmd[@]}"" scalar: the empty case expands to ZERO words with no word-splitting, so
# ShellCheck SC2086 stays clean (the repo's action-shellcheck gate scans this first-party script).
sudo_cmd=()
[[ -n "${4:-}" ]] && sudo_cmd=(sudo)

src="$(mktemp -d)"
trap 'rm -rf "${src}"' EXIT

git clone --depth 1 --branch "${VERSION}" "${SOURCE}" "${src}"
# Verify the tag dereferences to the pinned commit (catches an upstream-moved tag).
test "$(git -C "${src}" rev-parse HEAD)" = "${COMMIT}"

# PRE-BUILD supply-chain gate (Hermes Medium #1). The cmake superbuild fetches+builds the transitive
# deps by upstream REF (Fast-CDR/Fast-DDS are MOVING branches), so a moved/compromised ref would
# fetch+configure+BUILD that code before the post-build pin check below could catch it. Here we ask
# the REMOTE what each pinned ref resolves to RIGHT NOW (ls-remote fetches no code) and refuse to run
# cmake at all if it no longer matches the manifest commit. Each EXPECT_<dep>_REF/_COMMIT is the
# manifest pin (stack-manifest.toml [bridge]); a dep with an empty ref OR commit is skipped.
preverify_transitive() {
  local url="$1" ref="$2" expected="$3" name="$4" out resolved
  [[ -z "${ref}" || -z "${expected}" ]] && return 0
  if ! out="$(git ls-remote "${url}" "${ref}" "refs/tags/${ref}^{}" 2>/dev/null)"; then
    echo "[xrce] ERROR (pre-build): cannot reach ${url} to resolve ${name} ref '${ref}'." >&2
    return 1
  fi
  # Match the ref EXACTLY (ls-remote globs: `3.x` also matches `integration/3.x`). Prefer the peeled
  # (^{}) line so an annotated tag resolves to its commit, matching the post-build rev-parse HEAD.
  resolved="$(printf '%s\n' "${out}" | awk -v r="${ref}" '
    $2 == "refs/heads/" r || $2 == "refs/tags/" r { plain=$1 }
    $2 == "refs/tags/" r "^{}" { peeled=$1 }
    END { if (peeled != "") print peeled; else print plain }')"
  if [[ -z "${resolved}" ]]; then
    echo "[xrce] ERROR (pre-build): ${name} ref '${ref}' not found on ${url}. Refusing to build." >&2
    return 1
  fi
  if [[ "${resolved}" != "${expected}" ]]; then
    echo "[xrce] ERROR (pre-build): ${name} ref '${ref}' now resolves to ${resolved}," >&2
    echo "[xrce]   but stack-manifest.toml [bridge] pins ${expected}. The upstream moving ref" >&2
    echo "[xrce]   advanced or was tampered. Re-resolve (git ls-remote ${url} ${ref}), bump the" >&2
    echo "[xrce]   manifest, then rebuild. Refusing to configure/build unverified code." >&2
    return 1
  fi
  echo "[xrce] OK (pre-build): ${name} ref '${ref}' -> ${resolved} matches the manifest pin." >&2
}
preverify_transitive "https://github.com/eProsima/Fast-CDR.git" "${EXPECT_FASTCDR_REF:-}"   "${EXPECT_FASTCDR_COMMIT:-}"   "Fast-CDR"
preverify_transitive "https://github.com/eProsima/Fast-DDS.git" "${EXPECT_FASTDDS_REF:-}"   "${EXPECT_FASTDDS_COMMIT:-}"   "Fast-DDS"
preverify_transitive "https://github.com/foonathan/memory.git"  "${EXPECT_FOONATHAN_REF:-}" "${EXPECT_FOONATHAN_COMMIT:-}" "foonathan_memory"
preverify_transitive "https://github.com/gabime/spdlog.git"     "${EXPECT_SPDLOG_REF:-}"    "${EXPECT_SPDLOG_COMMIT:-}"    "spdlog"

cmake -S "${src}" -B "${src}/build" \
    -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "${src}/build" -j"$(nproc)"

# POST-BUILD re-check of the superbuild's TRANSITIVE deps before installing them (Hermes Medium #1).
# Belt-and-suspenders with the pre-build gate above: this closes the TOCTOU window (the ref could move
# between ls-remote and the superbuild's actual fetch) and catches the superbuild fetching a different
# ref than we pre-verified — by comparing the ACTUALLY checked-out HEAD to the manifest pin. The cmake
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

"${sudo_cmd[@]}" install -d -m755 /usr/local/share/patrol-drone

# Install the agent binary + the superbuild's shared libs into /usr/local, recording an explicit
# installed-file MANIFEST (Hermes Medium #3). The superbuild has no top-level `install` target and its
# .so live only under the build tree, so we copy them out — but copying every `*.so*` pollutes
# /usr/local/lib (on the host, under sudo) with superbuild transitive libs outside apt/rosdep. The
# manifest makes that exact set auditable and reversible (rm $(cat xrce-agent.files)) instead of
# leaving an untracked spray of libraries on a developer host.
installed=("/usr/local/bin/MicroXRCEAgent")
"${sudo_cmd[@]}" install -m755 "${src}/build/MicroXRCEAgent" /usr/local/bin/MicroXRCEAgent
while IFS= read -r so; do
  "${sudo_cmd[@]}" cp -a "${so}" /usr/local/lib/
  installed+=("/usr/local/lib/$(basename "${so}")")
done < <(find "${src}/build" -name "*.so*")
"${sudo_cmd[@]}" ldconfig
printf '%s\n' "${installed[@]}" | "${sudo_cmd[@]}" tee /usr/local/share/patrol-drone/xrce-agent.files > /dev/null

# Record the installed commit so a rerun (host setup_phase1.sh::install_xrce_agent) can verify the
# on-disk agent against the manifest pin instead of trusting any MicroXRCEAgent on PATH (Hermes
# Medium #2). Same marker path on the host and in the sim container — they install the same tag.
printf '%s\n' "${COMMIT}" | "${sudo_cmd[@]}" tee /usr/local/share/patrol-drone/xrce-agent.commit > /dev/null
