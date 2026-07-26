#!/usr/bin/env bash
# Unit test for run_patrol_world_sitl.sh::report_keep_up — the --keep-up teardown hint (F-11).
#
# The regression it locks in: the hint must never interpolate a `:-0` default into a signal target.
# On the --no-patrol path no mission node is ever launched, so NODE_PID is unset; the old
# `kill -INT -- -${NODE_PID:-0}` rendered `kill -INT -- -0`, and POSIX defines a target of -0 as THE
# CALLER'S OWN PROCESS GROUP. An operator copy-pasting the runner's own instruction therefore SIGINTs
# their shell and every job in it, while the real stack (agent still holding port 8888, gz, PX4,
# bridge) survives — resurfacing on the next run as the classic stale-stack symptoms. Two failures in
# one: it does the wrong destructive thing AND omits the right one.
#
# Self-contained: sources run_patrol_world_sitl.sh in a way that only DEFINES its functions (its
# bottom `main` is guarded by BASH_SOURCE != $0), neutralizes the runner's inherited `set -eo
# pipefail`, and drives report_keep_up directly with stub PIDs. No processes are spawned and no
# signals are sent — the assertions are purely on the emitted hint text. Prints PASS + exits 0 on
# success; FAIL + exits 1 otherwise. Run directly (`bash tests/unit/test_keep_up_teardown_hint.sh`)
# or via the pytest wrapper (test_keep_up_teardown_hint.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

# Source the runner for its function defs only, then drop `-e` so a failed assertion is reported by
# this script rather than aborting the shell mid-check (mirrors test_stop_launch_group.sh).
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/run_patrol_world_sitl.sh"
set +e

# Stub the stack PIDs the hint interpolates. These are never signalled — report_keep_up only prints.
# shellcheck disable=SC2034  # all consumed as shell globals by the sourced report_keep_up
{
  AGENT_PID=19994
  GZ_PID=19995
  GZ_GUI_PID=19996
  PX4_PID=20223
  BRIDGE_PID=21209
}

# --- Case 1: --no-patrol (NODE_PID unset) -> the hint must omit the node rung entirely. -----------
unset NODE_PID
hint_no_node="$(report_keep_up 2>/dev/null)"

# The bug, stated exactly: a bare -0 target anywhere in the hint is the caller's own process group.
grep -Eq -- '-INT -- -0(\s|;|$)' <<<"${hint_no_node}" \
  && fail "hint targets process group -0 (the caller's own shell) when NODE_PID is unset: ${hint_no_node}"
grep -q -- 'kill -INT' <<<"${hint_no_node}" \
  && fail "hint emits a node-group kill with no node group to kill: ${hint_no_node}"

# It must still tear down the stack that IS running — the second half of the F-11 defect was that the
# hint left the agent (port 8888), gz, PX4 and the bridge up.
grep -q -- "kill -- -${PX4_PID}" <<<"${hint_no_node}" \
  || fail "hint must still group-kill PX4 on the --no-patrol path: ${hint_no_node}"
for pid in "${AGENT_PID}" "${GZ_PID}" "${BRIDGE_PID}"; do
  grep -q -- "${pid}" <<<"${hint_no_node}" \
    || fail "hint dropped stack PID ${pid} on the --no-patrol path: ${hint_no_node}"
done

# --- Case 2: a patrol ran (NODE_PID set) -> the node-group rung comes back, with the real PGID. ----
NODE_PID=20777
hint_with_node="$(report_keep_up 2>/dev/null)"

grep -q -- "kill -INT -- -${NODE_PID}" <<<"${hint_with_node}" \
  || fail "hint must group-INT the node (recorder finalize) when NODE_PID is set: ${hint_with_node}"
grep -Eq -- '-INT -- -0(\s|;|$)' <<<"${hint_with_node}" \
  && fail "hint targets process group -0 even with NODE_PID set: ${hint_with_node}"

echo "PASS: report_keep_up never targets process group -0, and emits the node rung only when there is a node group"
