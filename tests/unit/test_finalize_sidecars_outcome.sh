#!/usr/bin/env bash
# Unit test for run_patrol_world_sitl.sh::finalize_bag_sidecars — the sidecar-finalize OUTCOME gate
# (PR #16, Mira Medium, review 4728294643). The regression it locks in: a finalized bag
# (metadata.yaml present) that ends the finalize pass WITHOUT its <bag>.meta.json sidecar must make
# finalize_bag_sidecars return non-zero — not warn-and-succeed — because a sidecar-less bag is
# permanently stranded (upload_daemon.is_complete() refuses it forever). The gate checks the
# ARTIFACT (the sidecar exists afterward), not the finalize command's exit code, so a finalize that
# "succeeded" without writing anything (no staging crumb) is caught too.
#
# Self-contained: sources run_patrol_world_sitl.sh for its function defs only (its bottom `main` is
# BASH_SOURCE-guarded), neutralizes the inherited `set -e` so deliberate non-zero return-code
# assertions don't abort the shell (the test_stop_launch_group.sh idiom), and drives
# finalize_bag_sidecars against fixture run-roots — with python3 stubbed via PATH where the
# finalize step must be forced to fail. Prints PASS + exits 0 on success; FAIL + exits 1 otherwise.
# Run directly (`bash tests/unit/test_finalize_sidecars_outcome.sh`) or via the pytest wrapper
# (test_finalize_sidecars_outcome.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

# Source the runner for its function defs only; drop the inherited `-e` so deliberate non-zero
# returns are captured, not fatal — exactly as test_stop_launch_group.sh drives its target.
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/run_patrol_world_sitl.sh"
set +e

# A REAL finalized bag dir named ${2} under run_root ${1}: metadata.yaml AND an .mcap payload — the
# predicate find_finalized_bags applies (the shell twin of _shared.bag_layout.is_valid_bag_dir, F-02).
make_finalized_bag() {
  local run_root="$1" name="$2"
  make_metadata_only_bag "${run_root}" "${name}"
  : > "${run_root}/${name}/${name}_0.mcap"
}

# A metadata-only bag dir: the recorder was killed after metadata.yaml was written but before the
# MCAP flushed. NOT a finalized bag — it must never be discovered as one (F-02).
make_metadata_only_bag() {
  local run_root="$1" name="$2"
  mkdir -p "${run_root}/${name}"
  : > "${run_root}/${name}/metadata.yaml"
}

# python3 stubs: one that fails outright (patrol_logging unimportable / crash), and one that
# "succeeds" while writing nothing (the handler-wrote-neither shape the outcome gate must catch).
stub_fail="${tmp}/stub_fail"
stub_noop="${tmp}/stub_noop"
mkdir -p "${stub_fail}" "${stub_noop}"
printf '#!/usr/bin/env bash\nexit 1\n' > "${stub_fail}/python3"
printf '#!/usr/bin/env bash\nexit 0\n' > "${stub_noop}/python3"
chmod +x "${stub_fail}/python3" "${stub_noop}/python3"

# --- Case 1: every finalized bag already has its sidecar -> 0 (idempotent happy path). -------------
root1="${tmp}/run1"
make_finalized_bag "${root1}" "patrol_ok"
: > "${root1}/patrol_ok.meta.json"
finalize_bag_sidecars "${root1}"
rc=$?
[[ ${rc} -eq 0 ]] || fail "an all-sidecars-present run_root must return 0 (got ${rc})"

# --- Case 2: finalized bag, no sidecar, the finalize command FAILS -> non-zero. --------------------
root2="${tmp}/run2"
make_finalized_bag "${root2}" "patrol_broken"
# shellcheck disable=SC2030  # PATH is deliberately scoped to the subshell: stub python3 to fail
(PATH="${stub_fail}:${PATH}" && finalize_bag_sidecars "${root2}")
rc=$?
[[ ${rc} -ne 0 ]] || fail "a finalize FAILURE leaving no sidecar must return non-zero"
[[ ! -f "${root2}/patrol_broken.meta.json" ]] || fail "test setup: the failing stub must not write a sidecar"

# --- Case 3: the finalize command exits 0 but writes NOTHING -> non-zero (artifact check bites). ---
root3="${tmp}/run3"
make_finalized_bag "${root3}" "patrol_silent"
# shellcheck disable=SC2030,SC2031  # PATH is deliberately scoped to the subshell: stub python3 to no-op
(PATH="${stub_noop}:${PATH}" && finalize_bag_sidecars "${root3}")
rc=$?
[[ ${rc} -ne 0 ]] || fail "an exit-0 finalize that wrote NO sidecar must still return non-zero (outcome, not exit code)"

# --- Case 4: no finalized bags at all -> 0 (the --no-patrol / camera-only paths must not fail). ----
root4="${tmp}/run4"
mkdir -p "${root4}"
finalize_bag_sidecars "${root4}"
rc=$?
[[ ${rc} -eq 0 ]] || fail "a run_root with no finalized bags must return 0 (got ${rc})"

# --- Case 5: <bag>.meta.json is a SYMLINK -> non-zero (parity with bag_layout.is_regular_file, F-02). -
# A planted symlink must not satisfy the outcome gate: -f follows it, but the runner also rejects -L,
# matching recorder._is_regular_file and upload_daemon.is_complete (which refuse a symlinked sidecar).
root5="${tmp}/run5"
make_finalized_bag "${root5}" "patrol_link"
ln -s /etc/hostname "${root5}/patrol_link.meta.json"  # a symlink, not a regular sidecar
# shellcheck disable=SC2030,SC2031  # PATH scoped to the subshell: stub python3 so finalize no-ops
(PATH="${stub_noop}:${PATH}" && finalize_bag_sidecars "${root5}")
rc=$?
[[ ${rc} -ne 0 ]] || fail "a symlinked <bag>.meta.json must not satisfy the outcome gate (F-02)"

# --- Case 6: a metadata-only bag is NOT a finalized bag (F-02, review 4754192970). ----------------
# The uploader requires a real .mcap (_shared.bag_layout.is_valid_bag_dir), so a bag discovered on
# metadata.yaml alone reported "finalized" at the runner and was then skipped FOREVER by the
# uploader — silently stranded. find_finalized_bags now applies the same predicate: given one real
# bag and one metadata-only dir, it must emit only the real one.
root6="${tmp}/run6"
make_finalized_bag "${root6}" "patrol_real"
make_metadata_only_bag "${root6}" "patrol_metadata_only"
found="$(find_finalized_bags "${root6}")"
[[ "${found}" == "${root6}/patrol_real" ]] || fail "find_finalized_bags must emit ONLY the real bag (got '${found}')"

# --- Case 7: a run_root holding ONLY a metadata-only bag yields no finalized bag at all. -----------
# graceful_stop_mission's gate is `[[ -z "$(find_finalized_bags … | head -n1)" ]]` -> the run now
# FAILS at the producer instead of stranding the bag at the uploader. finalize_bag_sidecars writes no
# sidecar for it (it is not a bag) and so has nothing to report missing.
root7="${tmp}/run7"
make_metadata_only_bag "${root7}" "patrol_stranded"
[[ -z "$(find_finalized_bags "${root7}" | head -n1)" ]] || fail "a metadata-only bag must not satisfy the finalized-bag gate"

# Every candidate here is skipped, so this deterministically exercises the empty result: it must EXIT
# 0, because "no bags" is reported via stdout, never via exit status. assert_bag_has_compressed_imagery
# calls find_finalized_bags in a plain assignment, which under `set -eo pipefail` would abort the whole
# SITL run rather than reach its own "no finalized bag" error branch if a non-zero ever leaked out
# (hence the explicit `return 0`). Locked in so a future edit to the loop body can't regress it.
find_finalized_bags "${root7}" >/dev/null
rc=$?
[[ ${rc} -eq 0 ]] || fail "find_finalized_bags must exit 0 when every candidate is skipped (got ${rc})"
# shellcheck disable=SC2031  # PATH scoped to the subshell: stub python3 so finalize itself no-ops
(PATH="${stub_noop}:${PATH}" && finalize_bag_sidecars "${root7}")
rc=$?
[[ ${rc} -eq 0 ]] || fail "finalize_bag_sidecars must not discover a metadata-only dir as a bag (got ${rc})"
[[ ! -f "${root7}/patrol_stranded.meta.json" ]] || fail "a metadata-only bag must not get a sidecar"

echo "PASS: finalize_bag_sidecars fails when a finalized bag ends without its sidecar (outcome gate)"
