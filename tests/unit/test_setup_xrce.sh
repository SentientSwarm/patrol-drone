#!/usr/bin/env bash
# Unit test for setup_phase1.sh::install_xrce_agent fatal-by-default behavior (Hermes Medium #1).
#
# The Micro XRCE-DDS Agent is the core M2 bridge deliverable, so a BUILD FAILURE must abort setup
# (non-zero) by default, and only be downgraded to a warning under --allow-missing-xrce. We assert
# both branches by sourcing the real script (its bottom `main` is guarded so sourcing is safe),
# stubbing the build recipe to fail, and calling install_xrce_agent directly.
#
# Self-contained: prints PASS and exits 0 on success; prints FAIL and exits 1 otherwise. Run
# directly (`bash tests/unit/test_setup_xrce.sh`) or via the pytest wrapper (test_setup_xrce.py).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
mkdir -p "${tmp}/scripts"
# Stub build recipe that always fails — exercises the build-failure path without a real clone/build.
printf '#!/usr/bin/env bash\nexit 1\n' > "${tmp}/scripts/build_xrce_agent.sh"
chmod +x "${tmp}/scripts/build_xrce_agent.sh"

# Source the real script with no args so its arg-parse loop is a no-op and `main` stays unrun
# (guarded by BASH_SOURCE != $0). It sets `set -euo pipefail` + an ERR trap in this shell.
set --
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/setup_phase1.sh"
set +e
trap - ERR                       # drop inherited strict-mode/trap so we can capture return codes

have() { return 1; }             # force the build path: pretend no MicroXRCEAgent is installed
REPO_ROOT="${tmp}"               # point install_xrce_agent at the failing stub

# shellcheck disable=SC2034  # ALLOW_MISSING_XRCE is read by install_xrce_agent (sourced above)
ALLOW_MISSING_XRCE=0
install_xrce_agent >/dev/null 2>&1
rc=$?
[[ ${rc} -ne 0 ]] || fail "build failure must be fatal by default (install_xrce_agent returned ${rc})"

# shellcheck disable=SC2034  # ditto — consumed by the sourced install_xrce_agent
ALLOW_MISSING_XRCE=1
install_xrce_agent >/dev/null 2>&1
rc=$?
[[ ${rc} -eq 0 ]] || fail "--allow-missing-xrce must downgrade build failure to a warning (returned ${rc})"

echo "PASS: install_xrce_agent is fatal by default and non-fatal with --allow-missing-xrce"
