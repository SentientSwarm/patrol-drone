#!/usr/bin/env bash
# Unit test for run_patrol_world_sitl.sh::require_finalize_wait — the FINALIZE_WAIT lower bound
# (PR #16 / F-03, Mira Medium). The regression it locks in: FINALIZE_WAIT must be STRICTLY GREATER
# than record.launch.py's _RECORDER_SIGTERM_TIMEOUT_S (60 s). The only prior validation (require_uint)
# proved non-negative-integer but not the ordering the runner's own L77-L81 comment requires, so an
# operator could export FINALIZE_WAIT=0 (or any value <= 60) and reintroduce the sidecar-loss race:
# with 0, wait_for_pgroup_exit's `for _ in $(seq 1 0)` loop body never runs, returns 1 immediately,
# and stop_launch_group escalates straight to a group SIGTERM — killing `ros2 launch` before its
# OnProcessExit sidecar handler runs (the missing-.meta.json bug). require_finalize_wait now rejects
# any value <= the recorder timeout, and derives that bound by scraping the launch constant so the
# coupling can't rot into a third hardcoded copy.
#
# Self-contained: sources run_patrol_world_sitl.sh in a way that only DEFINES its functions (its
# bottom `main` is guarded by BASH_SOURCE != $0), neutralizes the runner's inherited `set -eo
# pipefail`, and drives require_finalize_wait / recorder_sigterm_timeout directly — each in a subshell
# so their deliberate `exit 2` doesn't kill this harness. Prints PASS + exits 0 on success; FAIL +
# exits 1 otherwise. Run directly (`bash tests/unit/test_finalize_wait_bound.sh`) or via the pytest
# wrapper (test_finalize_wait_bound.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

# Source the runner for its function defs only. Its `main` is BASH_SOURCE-guarded (side-effect-free
# to source) and it sets `set -eo pipefail` at the top — drop `-e` immediately after so the deliberate
# `exit 2` rejections below are captured as exit codes, not fatal, exactly as test_stop_launch_group.sh
# drives its target under explicit return-code checks.
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/run_patrol_world_sitl.sh"
set +e

command -v grep >/dev/null 2>&1 || fail "grep required to scrape the recorder sigterm timeout"

# --- The scrape reads the launch constant (locks it against a rename). ------------------------------
timeout="$(recorder_sigterm_timeout)"
[[ "${timeout}" == "60" ]] \
  || fail "recorder_sigterm_timeout should scrape 60 from record.launch.py (got '${timeout}')"

# --- Undersized overrides are rejected (exit 2); valid overrides pass (exit 0). ---------------------
# 0/30 are below the 60 s recorder budget; 60 is equal (not strictly greater) so also rejected; 61 is
# the strictly-greater boundary and 90 is the default — both valid. Each runs in a subshell so its
# `exit 2` (on rejection) doesn't abort this test.
assert_exit() {  # assert_exit EXPECTED VALUE
  ( require_finalize_wait "${2}" ) >/dev/null 2>&1
  local rc=$?
  [[ "${rc}" -eq "${1}" ]] \
    || fail "require_finalize_wait ${2} should exit ${1}, got ${rc}"
}

assert_exit 2 0    # the seq 1 0 degenerate case — immediate SIGTERM escalation, sidecar lost
assert_exit 2 30   # <= 60
assert_exit 2 60   # equal is not strictly greater
assert_exit 0 61   # strictly greater — the boundary
assert_exit 0 90   # the runner default

# --- The scrape fails loud (exit 2) when the constant is absent — the branch most likely to rot. ----
# Point RECORD_LAUNCH at a file with no _RECORDER_SIGTERM_TIMEOUT_S so the coupling can never be
# validated against a silent fallback (a stale 0 would let every FINALIZE_WAIT through).
missing_const_file="$(mktemp)"
echo "no recorder constant here" > "${missing_const_file}"
RECORD_LAUNCH="${missing_const_file}" bash -c '
  set -uo pipefail
  source "'"${REPO_ROOT}"'/scripts/run_patrol_world_sitl.sh"
  set +e
  RECORD_LAUNCH="'"${missing_const_file}"'"
  ( recorder_sigterm_timeout ) >/dev/null 2>&1
  exit $?
'
rc=$?
rm -f "${missing_const_file}"
[[ "${rc}" -eq 2 ]] \
  || fail "recorder_sigterm_timeout must exit 2 when _RECORDER_SIGTERM_TIMEOUT_S is absent (got ${rc})"

echo "PASS: require_finalize_wait enforces FINALIZE_WAIT > recorder sigterm timeout (scraped, fail-loud)"
