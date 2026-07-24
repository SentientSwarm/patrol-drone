#!/usr/bin/env bash
# Unit test for run_patrol_world_sitl.sh::wait_for_pgroup_exit — the record-path finalize wait
# (PR #16 / F-01, Hermes High). The regression it locks in: the bounded wait must poll the whole
# PROCESS GROUP, not just the `ros2 launch` leader. `stop_launch_group` group-signals the recorder
# (kill -INT -- -PGID) but must then wait until the `ros2 bag record` grandchild is gone — the leader
# (`ros2 launch`) can exit while the recorder is still flushing a large MCAP. The old helper probed
# the positive leader PID (`kill -0 PGID`) and returned the instant the leader died, tearing down the
# recorder's data sources mid-flush (the exact `positive_pid_alive=no / process_group_alive=yes` state
# Hermes reproduced). wait_for_pgroup_exit now probes `pgrep -g PGID`, so it holds until the group
# empties.
#
# Self-contained: sources run_patrol_world_sitl.sh in a way that only DEFINES its functions (its
# bottom `main` is guarded by BASH_SOURCE != $0), neutralizes the runner's inherited `set -eo
# pipefail` so deliberate non-zero return-code assertions don't abort the shell, and drives
# wait_for_pgroup_exit directly. Prints PASS + exits 0 on success; FAIL + exits 1 otherwise. Run
# directly (`bash tests/unit/test_stop_launch_group.sh`) or via the pytest wrapper
# (test_stop_launch_group.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

# Track process groups we spawn so cleanup kills the leader AND any still-sleeping child.
_pgids=()
cleanup() {
  local pgid
  for pgid in "${_pgids[@]:-}"; do
    if [[ -n "${pgid}" ]]; then kill -- "-${pgid}" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT

# Source the runner for its function defs only. The runner sources env_doctor.sh at load time (whose
# own `main` is BASH_SOURCE-guarded, so that is side-effect-free) and sets `set -eo pipefail` at the
# top — which the sourcing shell inherits. Drop `-e` immediately after so the deliberate timeout
# assertion below (wait_for_pgroup_exit returns 1 on purpose) is captured, not fatal, exactly as
# test_verify_transitive.sh drives its target under explicit return-code checks.
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/run_patrol_world_sitl.sh"
set +e

command -v pgrep >/dev/null 2>&1 || fail "pgrep (procps-ng) required for the process-group wait test"

# Spawn a process GROUP whose leader exits promptly while a child keeps sleeping. setsid makes the
# new shell a group leader (PGID == its PID); it backgrounds a long sleep (the stand-in for the still
# -flushing `ros2 bag record` grandchild) and then exits, so the group outlives its leader.
setsid bash -c 'sleep 30 & echo $$; wait' >/dev/null 2>&1 &
leader_pid=$!
# The leader's PID is its PGID (setsid). Give it a moment to establish the group + background child.
sleep 1
pgid="${leader_pid}"
_pgids+=("${pgid}")

# --- Case 1: the leader exits but the child sleeps on -> the wait MUST time out (return non-zero). --
# `setsid bash -c '... & wait'` keeps the leader alive until the child exits, so force the split:
# kill ONLY the leader (positive PID), leaving the group's sleeping child alive.
kill "${leader_pid}" 2>/dev/null
sleep 1
# Contrast probe: this is the precise state Hermes flagged — leader gone, group still alive.
kill -0 "${leader_pid}" 2>/dev/null \
  && fail "test setup: leader ${leader_pid} should be gone after kill"
pgrep -g "${pgid}" >/dev/null 2>&1 \
  || fail "test setup: group ${pgid} should still be alive (sleeping child) after the leader exits"

wait_for_pgroup_exit "${pgid}" 3
rc=$?
[[ ${rc} -ne 0 ]] \
  || fail "wait_for_pgroup_exit must NOT return 0 while a group child is still alive (leader gone)"

# --- Case 2: the whole group is gone -> the wait returns 0 promptly. -------------------------------
kill -- "-${pgid}" 2>/dev/null
# Poll until the group is actually reaped, then assert the wait reports it gone within its budget.
for _ in $(seq 1 5); do pgrep -g "${pgid}" >/dev/null 2>&1 || break; sleep 1; done
wait_for_pgroup_exit "${pgid}" 3
rc=$?
[[ ${rc} -eq 0 ]] \
  || fail "wait_for_pgroup_exit must return 0 once the whole group has exited (got ${rc})"

# --- Case 3: a group that IGNORES SIGINT+SIGTERM makes stop_launch_group return non-zero, and a
# SIGKILL reap then clears it (PR #16 / F-01, Mira High). This is the double-timeout branch:
# graceful_stop_mission must NOT blank NODE_PID here (stop_launch_group returned non-zero → group
# still live), so shutdown()'s SIGKILL rung reaps it instead of skipping a live `ros2 launch` +
# `ros2 bag record` group while PX4/gz are torn down. Override FINALIZE_WAIT so stop_launch_group's
# two bounded waits are ~1s each — the default 90s would hang this test ~180s; stop_launch_group
# re-reads FINALIZE_WAIT as a global and does NOT re-validate the >60s bound (that guard lives in
# require_finalize_wait / main only, neither of which runs when the script is sourced for its defs).
# shellcheck disable=SC2034  # consumed as a shell global by the sourced stop_launch_group (above)
FINALIZE_WAIT=1
setsid bash -c "trap '' INT TERM; sleep 30" &  # group leader that traps-ignores BOTH signals
ig_pid=$!
sleep 1
ig_pgid="${ig_pid}"
_pgids+=("${ig_pgid}")
pgrep -g "${ig_pgid}" >/dev/null 2>&1 \
  || fail "test setup: ignore-signals group ${ig_pgid} should be alive before stop_launch_group"
stop_launch_group "${ig_pgid}"
rc=$?
[[ ${rc} -ne 0 ]] \
  || fail "stop_launch_group must return non-zero when the group survives SIGINT+SIGTERM (got ${rc})"
# The shutdown() SIGKILL rung: a group that ignored SIGINT+SIGTERM must still be reapable by SIGKILL.
kill -KILL -- "-${ig_pgid}" 2>/dev/null || true
wait_for_pgroup_exit "${ig_pgid}" 5 \
  || fail "SIGKILL reap must clear a group that ignored SIGINT+SIGTERM"

echo "PASS: wait_for_pgroup_exit waits on the process GROUP, not just the leader PID"
