#!/usr/bin/env bash
# Unit test for build_xrce_agent.sh::verify_transitive — the POST-BUILD transitive-pin guard
# (PR #16 / F-05). The regression it locks in: the guard must check EVERY checkout of a dep, not
# just the first one `find` returns. Fast-DDS vendors its OWN Fast-CDR submodule, so Fast-CDR appears
# twice in the fetched tree; the old first-match `break` made the result depend on filesystem order
# (a correctly-pinned sibling could mask a drifted copy). Each checkout is now verified against ITS
# authoritative commit: the manifest pin for a top-level superbuild checkout, or the superproject's
# recorded gitlink for a vendored git submodule.
#
# Self-contained: builds tiny throwaway git repos as fixtures, sources build_xrce_agent.sh in
# XRCE_LIB_ONLY mode (defines the functions, skips the clone/build body), and drives verify_transitive
# directly. Prints PASS + exits 0 on success; FAIL + exits 1 otherwise. Run directly
# (`bash tests/unit/test_verify_transitive.sh`) or via the pytest wrapper (test_verify_transitive.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

_URL="https://github.com/eProsima/Fast-CDR.git"
_PIN="1111111111111111111111111111111111111111"   # the manifest-expected commit for the fixtures

# Make a fixture git repo at ${1} whose single commit's URL is ${_URL}. Echoes the commit sha.
make_checkout() {
  local dir="$1" content="${2:-x}"
  mkdir -p "${dir}"
  git -C "${dir}" init -q
  git -C "${dir}" config user.email t@t
  git -C "${dir}" config user.name t
  git -C "${dir}" remote add origin "${_URL}"
  printf '%s\n' "${content}" > "${dir}/f"
  git -C "${dir}" add -A
  git -C "${dir}" -c commit.gpgsign=false commit -qm "c"
  git -C "${dir}" rev-parse HEAD
}

# Source the recipe in lib-only mode: defines verify_transitive + submodule_pinned_commit, skips the
# imperative body (no clone/build). `src` is what verify_transitive scans (${src}/build).
# shellcheck disable=SC1091
XRCE_LIB_ONLY=1 source "${REPO_ROOT}/scripts/build_xrce_agent.sh"

# --- Case 1: two independent top-level checkouts, BOTH at the pin -> pass (all verified). -----------
src="${tmp}/case_ok"
mkdir -p "${src}/build/a" "${src}/build/b"
h1="$(make_checkout "${src}/build/a/fastcdr" one)"
git -C "${src}/build/a/fastcdr" reset -q --hard "${h1}"
h2="$(cd "${src}/build/a/fastcdr" && git rev-parse HEAD)"
# Second checkout: reset it to the SAME sha by cloning the first (same tree/commit).
git clone -q "${src}/build/a/fastcdr" "${src}/build/b/fastcdr"
git -C "${src}/build/b/fastcdr" remote set-url origin "${_URL}"
# Both HEADs equal; use that shared sha as the expected pin.
pin="$(git -C "${src}/build/a/fastcdr" rev-parse HEAD)"
verify_transitive "${_URL}" "${pin}" "Fast-CDR" >/dev/null 2>&1 \
  || fail "two checkouts both at the pin must pass"

# --- Case 2: two top-level checkouts at DIFFERENT HEADs -> fail (closes the first-match hole). ------
src="${tmp}/case_mismatch"
mkdir -p "${src}/build/a" "${src}/build/b"
pin="$(make_checkout "${src}/build/a/fastcdr" first)"       # this one matches the pin
make_checkout "${src}/build/b/fastcdr" second >/dev/null    # this one does NOT
verify_transitive "${_URL}" "${pin}" "Fast-CDR" >/dev/null 2>&1 \
  && fail "a second checkout off the pin must fail even when the first matches"

# --- Case 3: a single top-level checkout at the pin -> pass (the common one-copy case). -------------
src="${tmp}/case_single"
mkdir -p "${src}/build"
pin="$(make_checkout "${src}/build/fastcdr" solo)"
verify_transitive "${_URL}" "${pin}" "Fast-CDR" >/dev/null 2>&1 \
  || fail "a single checkout at the pin must pass"

# --- Case 4: no checkout of this dep -> skipped (system-satisfied), returns 0. ----------------------
src="${tmp}/case_absent"
mkdir -p "${src}/build"
verify_transitive "${_URL}" "${_PIN}" "Fast-CDR" >/dev/null 2>&1 \
  || fail "an absent dep (no checkout) must be skipped, not failed"

# --- Case 5: a VENDORED submodule at a different-but-superproject-pinned commit -> pass. ------------
# The real shape: a superproject (stand-in for Fast-DDS) pins a Fast-CDR submodule to commit X while
# the top-level superbuild Fast-CDR sits at the manifest pin. The submodule must be verified against
# the superproject's recorded gitlink (X), NOT forced to the manifest pin.
src="${tmp}/case_submodule"
mkdir -p "${src}/build"
manifest_pin="$(make_checkout "${src}/build/fastcdr" toplevel)"   # top-level @ manifest pin
sub_src="${tmp}/subsrc"                                            # the submodule's upstream (a real Fast-CDR-URL repo)
make_checkout "${sub_src}" vendored >/dev/null
super="${src}/build/fastdds"                                      # the superproject that vendors it
mkdir -p "${super}"
git -C "${super}" init -q
git -C "${super}" config user.email t@t
git -C "${super}" config user.name t
git -C "${super}" -c protocol.file.allow=always -c commit.gpgsign=false \
  submodule add -q "${sub_src}" thirdparty/fastcdr
git -C "${super}" -c commit.gpgsign=false commit -qm "vendor fastcdr"
git -C "${super}/thirdparty/fastcdr" remote set-url origin "${_URL}"
verify_transitive "${_URL}" "${manifest_pin}" "Fast-CDR" >/dev/null 2>&1 \
  || fail "a submodule at its superproject-pinned commit must pass (not forced to the manifest pin)"

echo "PASS: verify_transitive checks every checkout against its authoritative pin (top-level + vendored)"
