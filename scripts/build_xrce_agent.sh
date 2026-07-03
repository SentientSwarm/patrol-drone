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

# ── Testable functions (defined before the imperative body so the unit test can source them) ────────
# The commit a git SUBMODULE checkout is pinned to by its superproject (empty if not a submodule).
# A dep like Fast-CDR can appear TWICE in the fetched tree: once as the agent superbuild's own
# ExternalProject checkout (pinned to the manifest commit) and once as a git submodule VENDORED by
# another fetched dep — Fast-DDS pins its own `thirdparty/fastcdr` submodule to a DIFFERENT, valid
# Fast-CDR (v2.2.6 for Fast-DDS v3.1.3, vs the agent's v2.2.4). That vendored copy is authoritative to
# its superproject, not the manifest — so verify it against the commit the superproject RECORDS for it
# (deterministic for the pinned Fast-DDS tag), not the manifest pin. Walks up to the nearest ancestor
# git repo and reads that path's `git submodule status` gitlink; empty when `dir` is a top-level
# checkout (no submodule ancestor records it), so the caller falls back to the manifest pin.
submodule_pinned_commit() {
  local dir="$1" parent rel status
  parent="$(git -C "${dir}/.." rev-parse --show-toplevel 2>/dev/null || true)"
  [[ -z "${parent}" || "${parent}" == "$(git -C "${dir}" rev-parse --show-toplevel 2>/dev/null)" ]] && return 0
  rel="$(realpath --relative-to="${parent}" "${dir}" 2>/dev/null || true)"
  [[ -z "${rel}" ]] && return 0
  # `git submodule status <path>` prints " <sha> <path> (<describe>)"; take the recorded gitlink sha.
  status="$(git -C "${parent}" submodule status "${rel}" 2>/dev/null || true)"
  [[ -z "${status}" ]] && return 0
  printf '%s\n' "${status}" | awk '{gsub(/^[-+U ]/, "", $1); print $1}'
}

# POST-BUILD re-check of the superbuild's TRANSITIVE deps before installing them (Hermes Medium #1;
# PR #16 / F-05). Belt-and-suspenders with the pre-build gate: this closes the TOCTOU window (the ref
# could move between ls-remote and the superbuild's actual fetch) and catches the superbuild fetching a
# different ref than we pre-verified — by comparing each ACTUALLY checked-out HEAD to its authoritative
# pin. The cmake superbuild fetches Fast-CDR/Fast-DDS/foonathan_memory/spdlog by upstream ref (all four
# pinned to immutable tags) and installs their .so into /usr/local; a retagged ref could change
# installed code without tripping manifest drift.
#
# F-05: a dep can appear in MORE THAN ONE checkout (Fast-DDS vendors its own Fast-CDR submodule). The
# old first-match `break` made the guard order-dependent — a correctly-pinned sibling could satisfy it
# while an unpinned copy sat elsewhere (or the nested copy tripped a spurious MISMATCH). We now iterate
# EVERY URL-matching checkout and verify each against ITS authoritative commit: the manifest pin for a
# top-level checkout, or the superproject-recorded gitlink for a vendored submodule (so Fast-DDS's own
# v2.2.6 Fast-CDR is verified against Fast-DDS's pin, not forced to the agent's v2.2.4). It reads
# ${src}/build (set by the imperative body below). Each EXPECT_<dep>_COMMIT is the manifest pin
# (stack-manifest.toml [bridge]); empty = not pinned -> skipped. A dep satisfied by a system package is
# not fetched (no checkout) and is skipped with a note. Fail CLOSED on a mismatch — built, refuse install.
verify_transitive() {
  local url="$1" expected="$2" name="$3" actual want origin
  [[ -z "${expected}" ]] && return 0
  local -a dirs=()
  while IFS= read -r gitdir; do
    local d url_actual
    d="$(dirname "${gitdir}")"
    url_actual="$(git -C "${d}" config --get remote.origin.url 2>/dev/null || true)"
    [[ "${url_actual%.git}" == "${url%.git}" ]] && dirs+=("${d}")
  done < <(find "${src}/build" -name .git 2>/dev/null)
  if [[ ${#dirs[@]} -eq 0 ]]; then
    echo "[xrce] NOTE: ${name} not fetched by the superbuild (system-satisfied?) — skipping pin check" >&2
    return 0
  fi
  for dir in "${dirs[@]}"; do
    actual="$(git -C "${dir}" rev-parse HEAD)"
    # Authoritative commit for THIS checkout: a vendored submodule answers to its superproject's
    # recorded gitlink; a top-level superbuild checkout answers to the manifest pin.
    want="$(submodule_pinned_commit "${dir}")"
    origin="its superproject's submodule pin"
    if [[ -z "${want}" ]]; then want="${expected}"; origin="the manifest pin"; fi
    if [[ "${actual}" != "${want}" ]]; then
      echo "[xrce] ERROR: ${name} transitive pin MISMATCH — checkout ${dir} at ${actual}," >&2
      echo "[xrce]   ${origin} expects ${want}. An upstream ref moved/was tampered, or a nested" >&2
      echo "[xrce]   vendored copy drifted. Re-resolve (git ls-remote <repo> <ref>), bump the" >&2
      echo "[xrce]   manifest if needed, then rebuild. Refusing to install." >&2
      return 1
    fi
  done
  echo "[xrce] OK: all ${#dirs[@]} ${name} checkout(s) verified against their authoritative pins." >&2
}

# Sourced in lib-only mode (the unit test wants just the functions above): stop before the imperative
# clone/build/install body. `return` works because the test `source`s this file; a direct run leaves
# XRCE_LIB_ONLY unset and proceeds. Must sit AFTER the function defs and BEFORE the arg parsing.
[[ "${XRCE_LIB_ONLY:-}" == "1" ]] && return 0
# ────────────────────────────────────────────────────────────────────────────────────────────────────

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

# PIN THE SUPERBUILD TO IMMUTABLE TAGS (Hermes High, head 8b85069). Upstream, the agent's
# CMakeLists.txt points Fast-DDS/Fast-CDR at the MOVING branches `set(_fastdds_tag 3.x)` /
# `set(_fastcdr_tag 2.2.x)`, so the superbuild fetch is non-reproducible and breaks the pinned build
# when eProsima advances the branch. We rewrite those two GIT_TAG vars to the manifest's immutable
# tags (EXPECT_*_REF, stack-manifest.toml [bridge]) BEFORE configuring, so the superbuild fetches
# exactly the pinned commit. foonathan_memory/spdlog are already tags upstream — no rewrite needed.
# Fail CLOSED if the expected `set(_<dep>_tag ...)` line is absent (the upstream CMake layout changed
# and our reproducibility assumption no longer holds — refuse rather than build a moving branch).
pin_superbuild_tag() {
  local var="$1" ref="$2" cml="${src}/CMakeLists.txt"
  [[ -z "${ref}" ]] && return 0   # gate degrades gracefully when the caller omits the ref
  if ! grep -Eq "^[[:space:]]*set\(${var} [^)]+\)" "${cml}"; then
    echo "[xrce] ERROR (pin): '${var}' not found in the agent CMakeLists.txt — upstream superbuild" >&2
    echo "[xrce]   layout changed; cannot pin it to the immutable tag '${ref}'. Refusing to build a" >&2
    echo "[xrce]   moving branch. Re-check the agent ${VERSION} superbuild and update this script." >&2
    return 1
  fi
  sed -i -E "s|^([[:space:]]*set\()${var} [^)]+(\).*)$|\1${var} ${ref}\2|" "${cml}"
  grep -Eq "^[[:space:]]*set\(${var} ${ref}\)" "${cml}" || {
    echo "[xrce] ERROR (pin): failed to rewrite '${var}' to '${ref}' in the agent CMakeLists.txt." >&2
    return 1
  }
  echo "[xrce] OK (pin): superbuild ${var} pinned to immutable tag '${ref}'." >&2
}
pin_superbuild_tag "_fastcdr_tag" "${EXPECT_FASTCDR_REF:-}"
pin_superbuild_tag "_fastdds_tag" "${EXPECT_FASTDDS_REF:-}"

# PRE-BUILD supply-chain gate (Hermes Medium #1). The cmake superbuild fetches+builds the transitive
# deps by upstream REF (now pinned to immutable tags by pin_superbuild_tag above), so a retagged/
# compromised ref would fetch+configure+BUILD that code before the post-build pin check below could
# catch it. Here we ask
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
# Dedupe by basename over a SORTED find (Hermes Medium #2): the superbuild tree can hold the same
# .so basename under multiple subdirs; an unordered `cp` would be last-write-wins (non-deterministic)
# and could install both a symlink and its target. First-wins over a stable sort makes the installed
# set — and the xrce-agent.files manifest below — reproducible across builds.
declare -A seen_so
while IFS= read -r so; do
  base="$(basename "${so}")"
  [[ -n "${seen_so[${base}]:-}" ]] && continue
  seen_so[${base}]=1
  "${sudo_cmd[@]}" cp -a "${so}" /usr/local/lib/
  installed+=("/usr/local/lib/${base}")
done < <(find "${src}/build" -name "*.so*" | sort)
"${sudo_cmd[@]}" ldconfig
printf '%s\n' "${installed[@]}" | "${sudo_cmd[@]}" tee /usr/local/share/patrol-drone/xrce-agent.files > /dev/null

# Record the installed commit so a rerun (host setup_phase1.sh::install_xrce_agent) can verify the
# on-disk agent against the manifest pin instead of trusting any MicroXRCEAgent on PATH (Hermes
# Medium #2). Same marker path on the host and in the sim container — they install the same tag.
printf '%s\n' "${COMMIT}" | "${sudo_cmd[@]}" tee /usr/local/share/patrol-drone/xrce-agent.commit > /dev/null
