# CI Workflows Design — Two-Layer Architecture (Phase 1)

**Status:** Draft for review
**Date:** 2026-06-03
**Scope:** Phase 1 (pre-hardware, simulation-only)
**Pillars:** Testing · Linting · Complexity · Type safety

All tool/action versions in this document were verified live against GitHub/PyPI
primary sources on 2026-06-03. One correction was applied during verification
(`actions/checkout` commit SHA — see §2).

---

## 1. Goals and constraints

Add GitHub Actions CI covering four pillars — **testing, linting, complexity,
type safety** — for a ROS 2 Jazzy + Python 3.12 monorepo that is currently a
**skeleton with no source code yet**. The design must:

- Be **green on today's empty repo** and activate automatically as code lands in M3+.
- Keep the fast feedback loop **fast** (pure-Python checks in seconds on a plain runner).
- Keep the slow/flaky **SITL tier out of per-PR CI** (the project plan flags it as the
  least-confident area; software-rendered Gazebo on hosted runners is heavy).
- Be **reproducible** (pinned tool versions; pinned, SHA-referenced actions).
- Follow the repo working agreement: branch-and-PR everything, tests before merge.

### Decisions taken (this design)

| Decision | Choice |
|---|---|
| CI scope | **Two-layer, ROS-aware** (fast pure-Python layer + colcon/ament layer); SITL scaffold only |
| Complexity tool | **xenon** hard gate (radon-based), `--max-absolute B` |
| Python tooling env | **uv** (`astral-sh/setup-uv`) |
| Linter / formatter | **ruff** (lint + format check) |
| Type checker | **mypy** |
| ROS layer | **`ros-tooling/action-ros-ci`** in a Jazzy (Noble) container |
| Local hooks | **None** — CI-only (no `.pre-commit-config.yaml`) |
| Layer-B coverage | Informational only (gate lives in Layer A) |
| arm64 (Jetson) | Deferred to nightly; not per-PR in Phase 1 |
| `ament_copyright` | Off |

---

## 2. File layout and version pins

```
.github/
  workflows/
    python-quality.yml   # Layer A — ruff, mypy, xenon, pytest, shellcheck (parallel jobs)
    ros-ci.yml           # Layer B — action-ros-ci colcon build+test in a Jazzy container
    sitl-nightly.yml     # Scaffold only — workflow_dispatch + nightly cron, NOT required
  dependabot.yml         # keep action SHA pins fresh (github-actions ecosystem)
pyproject.toml           # root: ruff/mypy/pytest config + dev dependency-group pins
uv.lock                  # committed lockfile — identical tool versions in CI and local
docs/design/2026-06-03-ci-workflows-design.md   # this document
```

Separate workflow files (mirroring Nav2 / MoveIt2) give each pillar its own
branch-protection **status-check name** and let the heavy ROS job carry different
triggers/concurrency than the cheap Python jobs. Within `python-quality.yml`,
each pillar is a **separate job** (not a step) so they run in parallel and a
failure names the exact pillar.

### Version pin reference (verified 2026-06-03)

| Tool / Action | Tag | Commit SHA (hardened pin) |
|---|---|---|
| `actions/checkout` | `v6.0.3` | `df4cb1c069e1874edd31b4311f1884172cec0e10` |
| `astral-sh/setup-uv` | `v8.2.0` | `fac544c07dec837d0ccb6301d7b5580bf5edae39` |
| `ros-tooling/action-ros-ci` | `0.4.8` | `3a640b10f09b756dabe556dac5413aba369f71b0` |
| `ros-tooling/setup-ros` | `0.7.18` | `77bcad67a6cb15f6192d61464d99bbab658e4ca9` |
| `ludeeus/action-shellcheck` | `2.0.0` | `00cae500b08a931fb5698e11e79bfbd38e612a38` |
| ruff | `0.15.15` | (PyPI; pinned in `uv.lock`) |
| mypy | `2.1.0` | (PyPI; requires-python >=3.10) |
| xenon | `0.9.3` | (PyPI; depends radon>=4,<7) |
| radon | `6.0.1` | (PyPI; reporting only) |
| pytest | `9.0.3` | (PyPI; requires-python >=3.10) |

**Correction applied during verification:** the draft pinned `actions/checkout`
to `9f698171…`, which is the **annotated-tag-object** SHA, not a commit. A
GitHub Actions `@<sha>` ref resolves against commits, so that pin would not
resolve. The correct commit SHA for `v6.0.3` is
`df4cb1c069e1874edd31b4311f1884172cec0e10`.

**Pinning policy.** Pin third-party actions to the **commit SHA + a trailing
`# vX.Y.Z` comment** — the only immutable reference (relevant given the recent
`tj-actions/changed-files` and `trivy-action` supply-chain compromises).
`setup-uv` deliberately **stopped publishing moving `@v8`/`@v8.0` tags** at
v8.0.0, so a bare `@v8` will not resolve — use the full immutable tag or the SHA.
`.github/dependabot.yml` (github-actions ecosystem) opens bump PRs so the pins
stay current.

> The `setup-uv`, `action-ros-ci`, `setup-ros`, and `action-shellcheck` tags are
> **lightweight** (tag SHA == commit SHA). Only `actions/checkout` uses
> **annotated** tags, which is why its SHA needed dereferencing.

---

## 3. Layer A — `python-quality.yml` (fast pure-Python)

Runner pinned to `ubuntu-24.04` (matches the ADR-0001 Python 3.12 / Jazzy
runtime; avoids `ubuntu-latest` drift). Triggered on `pull_request` **and**
`push: branches: [main]` (the push trigger is mandatory — a status check only
appears in the branch-protection picker after it has run on the default branch).
Top-level least-privilege `permissions`, `concurrency` with cancel-in-progress,
per-job `timeout-minutes`.

```yaml
name: Python Quality
on:
  pull_request:
  push:
    branches: [main]
permissions:
  contents: read
concurrency:
  group: pyq-${{ github.workflow }}-${{ github.head_ref || github.ref }}
  cancel-in-progress: true

jobs:
  lint:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: astral-sh/setup-uv@fac544c07dec837d0ccb6301d7b5580bf5edae39 # v8.2.0
        with:
          enable-cache: true
          python-version: "3.12"
      # ruff is self-contained — run ephemerally via uvx (no project sync needed).
      - name: Ruff lint
        run: uvx ruff@0.15.15 check --output-format=github .
      - name: Ruff format check
        run: uvx ruff@0.15.15 format --check .

  types:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: astral-sh/setup-uv@fac544c07dec837d0ccb6301d7b5580bf5edae39 # v8.2.0
        with:
          enable-cache: true
          cache-dependency-glob: "uv.lock"
          python-version: "3.12"
      - run: uv sync --locked --group dev
      # Bootstrap guard: mypy errors on "no .py files." Run only once first-party
      # Python exists; until then, skip with a notice (see §8).
      - id: pyfiles
        shell: bash
        run: |
          if [ -n "$(find ros2_ws/src analysis scripts tests -name '*.py' 2>/dev/null | head -n1)" ]; then
            echo "present=true" >> "$GITHUB_OUTPUT"
          else
            echo "present=false" >> "$GITHUB_OUTPUT"
            echo "::notice::no first-party Python yet — skipping mypy"
          fi
      - if: steps.pyfiles.outputs.present == 'true'
        name: mypy
        run: uv run mypy .

  complexity:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: astral-sh/setup-uv@fac544c07dec837d0ccb6301d7b5580bf5edae39 # v8.2.0
        with:
          enable-cache: true
          python-version: "3.12"
      # HARD GATE — fail if any single function exceeds rank B (CC > 10).
      # No average gates: an average couples a PR to unrelated, previously-accepted
      # code and can flip green->red on additive effects. Gate per-function only.
      # xenon has no config file; thresholds live here as CLI flags (single source of truth).
      - name: Complexity hard gate (xenon)
        run: >
          uvx --with radon==6.0.1 xenon@0.9.3
          --max-absolute B
          -i "external,build,install,log"
          analysis scripts tests ros2_ws/src
      # Non-gating report into the job summary (xenon is the sole authority).
      - name: Complexity report (radon, non-gating)
        if: always()
        run: |
          uvx radon@6.0.1 cc -s -a --md analysis scripts tests ros2_ws/src >> "$GITHUB_STEP_SUMMARY" || true
          uvx radon@6.0.1 mi -s analysis scripts tests ros2_ws/src >> "$GITHUB_STEP_SUMMARY" || true

  test:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: astral-sh/setup-uv@fac544c07dec837d0ccb6301d7b5580bf5edae39 # v8.2.0
        with:
          enable-cache: true
          cache-dependency-glob: "uv.lock"
          python-version: "3.12"
      - run: uv sync --locked --group dev
      # Exit code 5 = "no tests collected" — treated as pass until the first unit test exists.
      - name: Unit tests
        run: uv run pytest tests/unit || { code=$?; [ "$code" -eq 5 ] && echo "::notice::no unit tests yet" || exit "$code"; }

  shellcheck:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: ludeeus/action-shellcheck@00cae500b08a931fb5698e11e79bfbd38e612a38 # 2.0.0
```

### Tool-runner rule

- **`uvx`** for self-contained tools (`ruff`, `xenon`, `radon`) — ephemeral,
  isolated, no project sync needed.
- **`uv run`** for tools that must import the project's own modules (`mypy`,
  `pytest`) — runs after `uv sync --locked`. `--locked` makes CI **fail** on a
  stale `uv.lock` rather than silently re-resolving.
- The dev group (§4.1) pins `ruff`/`mypy`/`pytest` to the same versions as the
  `uvx` invocations, so `uv run` and `uvx` never diverge.
- **Never** use `--fix` or a mutating `ruff format` in CI — `ruff check` is
  read-only and `ruff format --check` is verify-only (exits non-zero when a file
  *would* be reformatted, without modifying it).

### shellcheck (in scope), yamllint (deferred)

`scripts/` already ships shell (`push-to-github.sh`, `set -euo pipefail`) and
more arrives in later milestones (bag wrapper, upload daemon) — so shellcheck is
justified now, not premature. `yamllint` is **deferred**: no runtime-parsed YAML
configs exist yet (they arrive in M4+). When config-driven mission/world YAML
lands, prefer a `yaml.safe_load()` unit test of the specific checked-in configs
over a blanket linter.

---

## 4. `pyproject.toml` (root, single source for Layer A)

A single root `pyproject.toml` owns Layer-A tool config and dev-dependency pins.
The colcon layer ignores it and uses each ROS package's
`setup.py`/`setup.cfg`/`CMakeLists.txt`.

```toml
[project]
name = "patrol-drone-tooling"
version = "0.0.0"
description = "Root tooling project for Layer-A CI (lint/type/complexity/test config)."
requires-python = ">=3.12"

[tool.uv]
package = false   # virtual root: install deps/groups, do not build this as a package
```

> `package = false` keeps uv from trying to build the repo root as a wheel while
> still letting `uv sync --group dev` populate the tooling venv. First-party
> imports for the ROS-free modules are handled via `mypy_path` / pytest
> `pythonpath` (§4.4–4.5), settled when the first package exists in M3.

### 4.1 Dev dependency group (PEP 735)

```toml
[dependency-groups]
dev = [
    "ruff==0.15.15",
    "mypy==2.1.0",
    "pytest==9.0.3",
    "xenon==0.9.3",
    "radon==6.0.1",
]
```

Use PEP 735 `[dependency-groups].dev` — **not** `[project.optional-dependencies]`
— because extras ship in published package metadata, whereas dependency groups
are the correct non-published home for dev/CI tooling.
`uv sync --locked --group dev` installs exactly these, reproducibly.

### 4.2 ruff (lint + format; C901 complements xenon)

```toml
[tool.ruff]
line-length = 100
target-version = "py312"
src = ["ros2_ws/src", "analysis", "scripts", "tests"]
extend-exclude = ["ros2_ws/src/external"]   # vendored px4_msgs — not our code

[tool.ruff.lint]
select = ["E", "W", "F", "I", "B", "C4", "UP", "SIM", "RUF", "PL", "PERF", "PT", "C901"]
ignore = [
    "E501",     # line length owned by the formatter
    "PLR0913",  # "too many arguments" — overlaps complexity; xenon owns aggregate gates
]

[tool.ruff.lint.mccabe]
max-complexity = 10   # CC > 10 == radon rank C, the same boundary as xenon -b B

[tool.ruff.lint.per-file-ignores]
"tests/**/*.py" = ["S101", "PLR2004"]
"**/__init__.py" = ["F401"]

[tool.ruff.format]
quote-style = "double"
indent-style = "space"
```

`C901` at `max-complexity = 10` and the xenon `-b B` gate sit on the same
CC=10/11 boundary, so the inline ruff hint and the hard gate never contradict.

### 4.3 xenon / radon — complexity hard gate (CLI only)

xenon does **not** read thresholds from `pyproject.toml` or `setup.cfg`;
thresholds are CLI-flag-only. The flags in the `complexity` job (§3) are the
single source of truth.

- **Semantics (verified against `xenon/core.py`):** the check is **strictly
  greater-than** (`rank > threshold`). Rank→CC map: A=1–5, B=6–10, C=11–20,
  D=21–30, E=31–40, F=41+. So `--max-absolute B` **allows** rank A and B and
  fails only on C/D/E/F.
- **Chosen policy (greenfield, mission-critical):** `--max-absolute B` only — no
  single function worse than CC≤10. **No average gates** (`--max-average` and
  `--max-modules` are intentionally omitted): an average couples a PR to the
  complexity of unrelated, previously-accepted code, so an otherwise-fine change
  can flip the build red through additive effects alone. Gating per-function
  keeps the signal local to the code under change. (Re-add `--max-modules A` only
  if a per-module average is later wanted.)
- **Exit codes:** 0 = pass, 1 = at least one infraction (fails the step
  naturally — do **not** append `|| true`).
- **Scope:** pass explicit first-party dirs; vendored/generated trees excluded
  via `-i "external,build,install,log"` plus not listing them.
- **radon 6.0.1** is reporting-only (markdown into `$GITHUB_STEP_SUMMARY`),
  pinned via `uvx --with radon==6.0.1` so the report engine matches the gate
  engine, and wrapped in `|| true` so it never gates.

### 4.4 mypy — type safety scoped without a ROS install

```toml
[tool.mypy]
python_version = "3.12"
warn_unused_configs = true
warn_redundant_casts = true
warn_unused_ignores = true
warn_return_any = true
check_untyped_defs = true
no_implicit_optional = true
ignore_missing_imports = false        # strict globally; only ROS roots exempted below
exclude = [
    "^\\.venv/",                       # uv's tooling venv on the runner
    "^ros2_ws/(build|install|log)/",   # colcon artifacts, if ever present
    "^ros2_ws/src/external/",          # vendored px4_msgs / px4_ros_com
    "^tests/integration/",             # live ROS env — Layer B / nightly only
    "^tests/replay/",
]
# Put the ROS-free module source on the path so `import patrol_mission` resolves
# in the fast layer (confirmed at M3 when the package exists).
# mypy_path = "ros2_ws/src/patrol_mission"

# Tighten the ROS-free mission core (highest bug risk).
[[tool.mypy.overrides]]
module = "patrol_mission.*"
disallow_untyped_defs = true

# ROS packages ship no PEP 561 stubs and aren't installed on the Layer-A runner.
# Treat their imports as Any rather than erroring. The trailing .* is MANDATORY —
# a bare `std_msgs` would not cover the submodule `std_msgs.msg`.
[[tool.mypy.overrides]]
module = [
    "rclpy", "rclpy.*",
    "px4_msgs", "px4_msgs.*",
    "px4_ros_com", "px4_ros_com.*",
    "std_msgs", "std_msgs.*",
    "sensor_msgs", "sensor_msgs.*",
    "geometry_msgs", "geometry_msgs.*",
    "builtin_interfaces", "builtin_interfaces.*",
    "patrol_interfaces", "patrol_interfaces.*",
    "rosidl_runtime_py", "rosidl_runtime_py.*",
    "ament_index_python", "ament_index_python.*",
]
ignore_missing_imports = true
```

**How Layer A type-checks ROS-touching code without ROS:** keep
`ignore_missing_imports = false` globally (so typos in your *own* imports still
fail) and exempt only the explicit ROS import roots. ROS 2 Jazzy / `px4_msgs`
ship no type stubs, so this is the standard pattern. Strictness
(`disallow_untyped_defs`) applies only to the ROS-free `patrol_mission.*` core;
`analysis/` (notebooks/scratch) stays relaxed. The `exclude` regex plus the
per-module overrides keep mypy from ever needing rclpy at import time.

### 4.5 pytest — testing scoped to the ROS-free suite

```toml
[tool.pytest.ini_options]
minversion = "9.0"
testpaths = ["tests/unit"]
norecursedirs = ["tests/integration", "tests/replay", "ros2_ws/src/external"]
addopts = "-ra --strict-markers --strict-config"
# pythonpath = ["ros2_ws/src/patrol_mission"]   # enable at M3 so `import patrol_mission` works
markers = [
    "ros: test requires a sourced ROS 2 environment (Layer B / nightly only)",
]
```

`testpaths` defaults a bare `pytest` to the ROS-free suite, and `norecursedirs`
blocks recursion into the ROS tiers — belt-and-suspenders so Layer A never
imports `rclpy`, keeping it in the <5s budget. Per `tests/README.md`, `unit/` is
"No ROS, no Gazebo, no PX4"; `integration/` (real SITL) and `replay/` belong to
Layer B / nightly.

**Architectural precondition (the repo plan already mandates it):**
`MissionStateMachine` lives in a pure-Python module
(`ros2_ws/src/patrol_mission/patrol_mission/state_machine.py`) with **no**
`import rclpy` and no runtime `from patrol_interfaces.msg import …`; the rclpy
node imports it. A module is "fast-layer eligible" only if its import graph never
reaches rclpy or a generated message package — use dataclasses / `TypedDict` or
`if TYPE_CHECKING:` guards for ROS message types in the ROS-free module.

---

## 5. Layer B — `ros-ci.yml` (colcon / ament in a Jazzy container)

```yaml
name: ROS CI
on:
  pull_request:
    paths: ['ros2_ws/**', '.github/workflows/ros-ci.yml']
  push:
    branches: [main]
    paths: ['ros2_ws/**', '.github/workflows/ros-ci.yml']
permissions:
  contents: read
concurrency:
  group: ros2-${{ github.workflow }}-${{ github.head_ref || github.ref }}
  cancel-in-progress: true

jobs:
  colcon_build_test:
    runs-on: ubuntu-24.04
    timeout-minutes: 45
    container:
      image: rostooling/setup-ros-docker:ubuntu-noble-latest   # Noble = 24.04; pin sha256 at impl
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      # Bootstrap guard: no ROS packages exist until M3. Skip the build cleanly so the
      # job is GREEN on the empty repo and auto-activates when the first package.xml lands.
      - id: pkgs
        shell: bash
        run: |
          if [ -n "$(find ros2_ws/src -name 'package.xml' 2>/dev/null | head -n1)" ]; then
            echo "present=true" >> "$GITHUB_OUTPUT"
          else
            echo "present=false" >> "$GITHUB_OUTPUT"
            echo "::notice::no ROS packages yet — skipping colcon build/test"
          fi
      - if: steps.pkgs.outputs.present == 'true'
        uses: ros-tooling/action-ros-ci@3a640b10f09b756dabe556dac5413aba369f71b0 # 0.4.8
        id: action_ros_ci
        with:
          target-ros2-distro: jazzy
          package-name: patrol_interfaces patrol_mission patrol_perception patrol_bringup
          # rosdep-skip-keys: px4_msgs   # uncomment ONLY if rosdep flags px4_msgs (see below)
      - if: ${{ always() && steps.pkgs.outputs.present == 'true' }}
        uses: actions/upload-artifact@v4   # current major; pin exact SHA at implementation
        with:
          name: colcon-logs
          path: ${{ steps.action_ros_ci.outputs.ros-workspace-directory-name }}/log
```

- **Container approach** (`rostooling/setup-ros-docker:ubuntu-noble-latest`) is
  the action-ros-ci README's first-class recommendation. `setup-ros` is **not**
  needed in the container path (the image ships ROS dev tools + rosdep). Add
  `setup-ros@0.7.18` only if you ever run container-less on a bare runner. Pin
  the image's `sha256` digest at implementation (`:latest` is mutable).
- **Do not hand-roll colcon steps.** action-ros-ci internally runs
  `colcon build → colcon test → colcon test-result` and **propagates the
  non-zero exit** — fixing the well-known trap that `colcon test` itself exits 0
  even on test failure. If you ever bypass the action, you MUST add
  `colcon test-result --all --verbose`.
- **Vendored px4_msgs** stays in `ros2_ws/src/external` and is built from source
  via `--packages-up-to`. **Do not** add a `vcs-repo-file-url` entry (that pulls
  fresh deps; px4_msgs is intentionally pinned to the matching PX4 branch). Add
  the narrow `rosdep-skip-keys: px4_msgs` **only if** the first CI run flags it
  as an unknown key. **Never** use `skip-rosdep-install: true` (breaks
  legitimate system deps like rosidl).
- **ament Python linters disabled in favor of ruff** (no double-linting):
  - *ament_python packages* (`patrol_mission`, `patrol_perception`,
    `patrol_bringup`): do **not** generate `test/test_flake8.py` /
    `test/test_pep257.py`, and omit their `<test_depend>` lines from
    `package.xml`. Do not add `ament_lint_common`.
  - *ament_cmake package* (`patrol_interfaces`): if `ament_lint_auto` is ever
    added, exclude the Python linters in `CMakeLists.txt`:
    ```cmake
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
      ament_lint_auto_find_test_dependencies()
    endif()
    ```
    Keep `ament_xmllint` (validates `package.xml`; no Layer-A overlap).
- **Coverage** is informational. action-ros-ci produces coverage by default; the
  hard coverage gate lives in Layer A (pytest on `tests/unit`, repo target
  >80% of state transitions). If uploading, use a **current**
  `codecov/codecov-action` major — do **not** copy the README's stale `@v1.2.1`.

---

## 6. `sitl-nightly.yml` — deferred scaffold (NOT a per-PR / required check)

```yaml
name: SITL Nightly (scaffold)
on:
  workflow_dispatch:
  schedule:
    - cron: '17 4 * * *'   # avoid :00 (congested); runs from default branch only
permissions:
  contents: read
jobs:
  sitl_integration:
    runs-on: ubuntu-24.04
    timeout-minutes: 60
    # container: ghcr.io/sentientswarm/patrol-drone-sim:latest  # docker/sim image (M3+)
    steps:
      - run: echo "TODO M3+: PX4 SITL + Gazebo Harmonic, run canonical mission, assert on bag"
```

`workflow_dispatch` (manual / self-test) + `schedule` (nightly, non-`:00` cron
minute since top-of-hour slots are congested). Scheduled workflows run **only**
from the default branch and GitHub can take 15+ minutes to register a new cron.
This stays **informational forever** — never a required check. Per-PR
`colcon test` only runs package test suites and never launches Gazebo/PX4 as
long as those suites don't spawn the simulator — that invariant keeps SITL out
of per-PR CI.

---

## 7. Branch protection (configure in GitHub settings — recommendation)

Require on `main`: the four Layer-A jobs (**lint**, **types**, **complexity**,
**test**), **shellcheck**, and Layer-B **colcon_build_test**. Do **not** require
`sitl-nightly`. Every required workflow already includes
`on: push: branches: [main]` (a check appears in the picker only after running on
the default branch). Enable "Require branches to be up to date before merging."

> Because `ros-ci.yml` is `paths:`-filtered, on pure-Python-only PRs the
> `colcon_build_test` check reports as skipped. GitHub treats a skipped
> path-filtered required check as success, which is the intended behavior here.

---

## 8. Bootstrap behavior (no code yet → green today)

The repo has zero `.py` files and zero ROS packages. The design is green on this
state and activates incrementally:

- **ruff** `check .` / `format --check .` → pass with zero matching files.
- **mypy** → bootstrap guard finds no first-party `.py`, skips with a notice
  (mypy errors on "no files," so we guard rather than run it on an empty tree).
- **xenon** over the listed dirs → no functions found. Pass.
- **pytest** `tests/unit` → exit code 5 ("no tests collected"), explicitly
  treated as pass until the first unit test exists.
- **shellcheck** → lints existing `scripts/*.sh`. Pass (assuming clean).
- **Layer B** → bootstrap guard finds no `package.xml`, skips the build. Green.

As M3 lands the first packages and tests, every job begins exercising real code
with no workflow changes (only un-commenting `mypy_path` / pytest `pythonpath`,
and possibly the `rosdep-skip-keys` line).

---

## 9. Open questions (resolve as code lands)

1. **px4_msgs rosdep key** — does the vendored `package.xml` name match what the
   consuming packages depend on? Determines whether `rosdep-skip-keys: px4_msgs`
   is needed. Validate on the first Layer-B run (M3).
2. **ROS-free import strategy** — `mypy_path` + pytest `pythonpath` (recommended,
   lighter) vs a small editable install. Spike in M3.
3. **px4_msgs build time** (rosidl generation) may dominate Layer-B; consider
   caching/splitting once real timings exist.
4. **ADR-0002** — capture the settled CI decision (two-layer, xenon, tool stack)
   as a short ADR per the repo working agreement, after implementation.

## 10. Verify at implementation time

1. `actions/upload-artifact` — pin exact current major + commit SHA.
2. `codecov/codecov-action` — pin exact current major + SHA *if* uploading
   Layer-B coverage; do not copy the README's stale `@v1.2.1`.
3. `rostooling/setup-ros-docker:ubuntu-noble-latest` — resolve and pin the
   immutable `sha256` digest.
4. Package names in `package-name` — confirm they match the actual M3 package
   directory names so the test set is complete.
5. Re-confirm `setup-uv` is still on the immutable-tag policy (no `@v8`) and the
   pinned SHA matches the intended release.

---

## Primary sources

- action-ros-ci: <https://github.com/ros-tooling/action-ros-ci> (action.yml / README @ 0.4.8)
- setup-ros: <https://github.com/ros-tooling/setup-ros/releases/tag/0.7.18>
- setup-uv: <https://github.com/astral-sh/setup-uv/releases/tag/v8.0.0> (immutable-tags rationale), <https://docs.astral.sh/uv/guides/integration/github/>
- actions/checkout SHA dereference: <https://api.github.com/repos/actions/checkout/git/refs/tags/v6.0.3>
- ruff: <https://docs.astral.sh/ruff/settings/>, <https://docs.astral.sh/ruff/rules/complex-structure/>, <https://docs.astral.sh/ruff/formatter/>
- mypy: <https://mypy.readthedocs.io/en/stable/config_file.html>, <https://mypy.readthedocs.io/en/stable/running_mypy.html>
- xenon: <https://github.com/rubik/xenon> (core.py semantics), <https://pypi.org/pypi/xenon/0.9.3/json>
- radon: <https://radon.readthedocs.io/en/latest/commandline.html>
- pytest: <https://docs.pytest.org/en/stable/reference/customize.html>
- ament linters: <https://github.com/ament/ament_lint>
- GitHub Actions security: <https://docs.github.com/en/actions/reference/security/secure-use>
- Patterns: <https://github.com/ros-navigation/navigation2/tree/main/.github/workflows>, <https://github.com/moveit/moveit2/tree/main/.github/workflows>
- Repo context: ADR-0001 (`docs/decisions/0001-distro-and-os.md`), `tests/README.md`, `ros2_ws/README.md`
