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
make_checkout "${src}/build/a/fastcdr" one >/dev/null
# Second checkout: clone the first so both sit at the SAME sha (same tree/commit).
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

# --- Case 4a: no checkout AND no system provenance -> FAIL CLOSED (F-02). ---------------------------
# Zero checkouts can mean "discovery failed" just as easily as "system-satisfied"; without a positive
# dpkg signal the gate must refuse. Override _system_provides deterministically rather than relying on
# the host's dpkg state — a ROS host may genuinely have libfastcdr installed, which would turn this
# into a host-dependent false pass.
src="${tmp}/case_absent"
mkdir -p "${src}/build"
_system_provides() { return 1; }
verify_transitive "${_URL}" "${_PIN}" "Fast-CDR" >/dev/null 2>&1 \
  && fail "an absent dep with NO system provenance must FAIL CLOSED, not be skipped"

# --- Case 4b: no checkout but POSITIVE system provenance -> accepted (returns 0). -------------------
# The legitimate system-satisfied path: dpkg owns the dep's lib, so the superbuild skipping the fetch
# is fine. Same deterministic override, inverted. (Cases 5/6 below have checkouts, so the lingering
# override is never called again.)
_system_provides() { return 0; }
verify_transitive "${_URL}" "${_PIN}" "Fast-CDR" >/dev/null 2>&1 \
  || fail "an absent dep WITH system provenance must be accepted, not failed"

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

# --- Case 6: a vendored submodule whose WORKTREE moved off the recorded gitlink -> FAIL. ------------
# The exact tamper case the gate exists for (Mira High, review 4728294643): the nested checkout
# advances (or is replaced) while the superproject still records the original pin. `git submodule
# status` would report the moved HEAD — stripping its `+` drift marker made the guard self-compare
# and pass; the recorded-gitlink (`ls-tree HEAD`) read must catch it.
git -C "${super}/thirdparty/fastcdr" -c user.email=t@t -c user.name=t -c commit.gpgsign=false \
  commit -q --allow-empty -m "tampered"
verify_transitive "${_URL}" "${manifest_pin}" "Fast-CDR" >/dev/null 2>&1 \
  && fail "a vendored submodule whose worktree drifted off the recorded gitlink must FAIL"

echo "PASS: verify_transitive checks every checkout against its RECORDED authoritative pin (top-level + vendored + tampered) and fails CLOSED on absence without system provenance"
