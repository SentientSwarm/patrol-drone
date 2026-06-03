# CI Workflows Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add GitHub Actions CI covering testing, linting, complexity, and type safety for the patrol-drone ROS 2 / Python monorepo, green on today's empty skeleton and auto-activating as code lands.

**Architecture:** Two layers. Layer A (`python-quality.yml`) runs fast pure-Python checks (ruff, mypy, xenon, pytest, shellcheck) on a plain `ubuntu-24.04` runner via `uv`. Layer B (`ros-ci.yml`) builds/tests the colcon workspace in a ROS 2 Jazzy container via `action-ros-ci`. SITL is a deferred manual/nightly scaffold (`sitl-nightly.yml`), never a per-PR check. Config lives in a root `pyproject.toml` + committed `uv.lock`. All actions are SHA-pinned; `dependabot.yml` keeps them fresh.

**Tech Stack:** GitHub Actions · uv 0.9+ · ruff 0.15.15 · mypy 2.1.0 · xenon 0.9.3 / radon 6.0.1 · pytest 9.0.3 · ros-tooling/action-ros-ci 0.4.8 · ROS 2 Jazzy.

**Source spec:** `docs/design/2026-06-03-ci-workflows-design.md` (read it before starting — it has the rationale behind every choice).

**Branch:** Work on `ci/github-actions` (already created; the design spec is committed there).

**Note on TDD framing:** This plan produces declarative config, not application logic, so each task's "test" is *running the real tool and asserting its observed behavior* (exit code / output) rather than a unit test. Where a gate is involved (xenon), the plan includes a step that deliberately trips the gate to prove it bites, then cleans up.

---

### Task 0: Prerequisites

**Files:** none

- [ ] **Step 1: Confirm you're on the feature branch**

Run: `git -C /Users/jxstanford/devel/SentientSwarm/patrol-drone branch --show-current`
Expected: `ci/github-actions`

- [ ] **Step 2: Confirm `uv` is installed (≥0.9)**

Run: `uv --version`
Expected: prints `uv 0.9.x` or newer.
If "command not found": install with `curl -LsSf https://astral.sh/uv/install.sh | sh` (then restart the shell) or `brew install uv`.

- [ ] **Step 3: Confirm the working tree is clean**

Run: `git -C /Users/jxstanford/devel/SentientSwarm/patrol-drone status --short`
Expected: empty output (the committed design spec is the only prior change on this branch).

---

### Task 1: Root `pyproject.toml` foundation + lockfile

Creates the tooling project uv recognizes, pins the dev tools, and generates the committed lockfile. No tool config yet — just enough for `uv sync` to work.

**Files:**
- Create: `pyproject.toml`
- Create (generated): `uv.lock`

- [ ] **Step 1: Create `pyproject.toml` with the project + dev group**

```toml
[project]
name = "patrol-drone-tooling"
version = "0.0.0"
description = "Root tooling project for Layer-A CI (lint/type/complexity/test config)."
requires-python = ">=3.12"

[tool.uv]
package = false   # virtual root: install deps/groups, do not build this as a package

[dependency-groups]
dev = [
    "ruff==0.15.15",
    "mypy==2.1.0",
    "pytest==9.0.3",
    "xenon==0.9.3",
    "radon==6.0.1",
]
```

- [ ] **Step 2: Generate the lockfile**

Run: `uv lock`
Expected: creates `uv.lock`; output ends with `Resolved N packages`. No error.

- [ ] **Step 3: Verify a locked sync installs the dev group**

Run: `uv sync --locked --group dev && uv run mypy --version`
Expected: sync completes, then prints `mypy 2.1.0 (compiled: ...)`. Confirms the lockfile is consistent and the dev tools install.

- [ ] **Step 4: Ignore the local venv**

Append `.venv/` to `.gitignore` if not already present.
Run: `grep -qxF '.venv/' .gitignore || printf '\n# uv tooling venv\n.venv/\n' >> .gitignore`
Then run: `git -C /Users/jxstanford/devel/SentientSwarm/patrol-drone status --short`
Expected: shows `pyproject.toml`, `uv.lock`, and possibly `.gitignore` as changes — but **not** `.venv/`.

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml uv.lock .gitignore
git commit -m "Add root tooling pyproject and uv lockfile"
```

---

### Task 2: ruff config (linting + formatting)

**Files:**
- Modify: `pyproject.toml` (append ruff tables)

- [ ] **Step 1: Append the ruff config to `pyproject.toml`**

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

- [ ] **Step 2: Verify ruff config parses and lint is green on the empty tree**

Run: `uvx ruff@0.15.15 check --output-format=github .`
Expected: prints `All checks passed!` (or no output), exit 0. (No `.py` files yet → nothing to flag, and an invalid config would error loudly.)

- [ ] **Step 3: Verify the format check is green**

Run: `uvx ruff@0.15.15 format --check .`
Expected: `0 files would be reformatted` (or "No Python files found"), exit 0.

- [ ] **Step 4: Verify the config is actually loaded (not silently ignored)**

Run: `uvx ruff@0.15.15 check --show-settings . | grep -E "line-length|mccabe" | head`
Expected: shows `line_length = 100` and a `mccabe` section with `max_complexity = 10`. Confirms our `[tool.ruff]` is in effect.

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml
git commit -m "Configure ruff lint and format for CI"
```

---

### Task 3: mypy config (type safety)

**Files:**
- Modify: `pyproject.toml` (append mypy tables)

- [ ] **Step 1: Append the mypy config to `pyproject.toml`**

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
# mypy_path = "ros2_ws/src/patrol_mission"   # enable at M3 so `import patrol_mission` resolves

# Tighten the ROS-free mission core (highest bug risk).
[[tool.mypy.overrides]]
module = "patrol_mission.*"
disallow_untyped_defs = true

# ROS packages ship no PEP 561 stubs and aren't installed on the Layer-A runner.
# The trailing .* is MANDATORY — a bare `std_msgs` would not cover `std_msgs.msg`.
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

- [ ] **Step 2: Sync (mypy must be installed to run)**

Run: `uv sync --locked --group dev`
Expected: completes without error.

- [ ] **Step 3: Verify the mypy config parses and mypy runs (snippet, no repo code needed)**

Run: `uv run mypy -c "x: int = 1"`
Expected: `Success: no issues found in 1 source file`, exit 0. (Confirms `[tool.mypy]` parses — a malformed config or bad regex would error here.)

- [ ] **Step 4: Verify mypy actually enforces types (the gate bites)**

Run: `uv run mypy -c "x: int = 'not an int'"; echo "exit=$?"`
Expected: reports `error: Incompatible types in assignment ...` and `exit=1`. Confirms mypy is enforcing, not no-opping.

- [ ] **Step 5: Confirm the empty-tree behavior that the CI guard handles**

Run: `uv run mypy . ; echo "exit=$?"`
Expected: errors with `There are no .py[i] files in directory '.'` and a non-zero exit. **This is expected** and is exactly why `python-quality.yml` guards the `mypy .` step behind a "first-party Python exists?" check (Task 6). No action needed here — this step just documents the behavior.

- [ ] **Step 6: Commit**

```bash
git add pyproject.toml
git commit -m "Configure mypy with ROS-aware import handling"
```

---

### Task 4: Complexity gate (xenon) — config-free, verified by behavior

xenon has no config file; its thresholds are CLI flags in the workflow. This task verifies the exact CLI the workflow will run is green on the empty tree **and** that the gate actually fails on a too-complex function.

**Files:** none (verification only; the flags live in `python-quality.yml`, Task 6)

- [ ] **Step 1: Verify the gate command is green on the empty tree**

Run:
```bash
uvx --with radon==6.0.1 xenon@0.9.3 --max-absolute B -i "external,build,install,log" analysis scripts tests ros2_ws/src ; echo "exit=$?"
```
Expected: no output, `exit=0`. (No functions exist yet → no infractions.)

- [ ] **Step 2: Prove the gate bites (deliberately complex function)**

Run:
```bash
mkdir -p /tmp/xenon-probe && cat > /tmp/xenon-probe/bad.py <<'PY'
def tangled(a, b, c, d):
    total = 0
    for i in range(a):
        if i % 2 == 0:
            for j in range(b):
                if j > c:
                    while d > 0:
                        if d % 3 == 0:
                            total += 1
                        elif d % 5 == 0:
                            total += 2
                        else:
                            total -= 1
                        d -= 1
    return total
PY
uvx --with radon==6.0.1 xenon@0.9.3 --max-absolute B /tmp/xenon-probe ; echo "exit=$?"
```
Expected: prints a `block "...:tangled" has a rank of C` (or worse) message and `exit=1`. Confirms `--max-absolute B` fails on rank C+.

- [ ] **Step 3: Clean up the probe**

Run: `rm -rf /tmp/xenon-probe`
Expected: no output, exit 0. (Probe was outside the repo — nothing to commit.)

- [ ] **Step 4: Verify the radon report command works (non-gating)**

Run: `uvx radon@6.0.1 cc -s -a analysis scripts tests ros2_ws/src ; echo "exit=$?"`
Expected: minimal/no output and `exit=0`. (This is the report engine the workflow writes into the job summary.)

No commit for this task — the xenon/radon invocations are codified in `python-quality.yml` (Task 6).

---

### Task 5: pytest config (testing)

**Files:**
- Modify: `pyproject.toml` (append pytest table)

- [ ] **Step 1: Append the pytest config to `pyproject.toml`**

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

- [ ] **Step 2: Sync and verify the no-tests-yet exit code**

Run: `uv sync --locked --group dev && (uv run pytest tests/unit; echo "exit=$?")`
Expected: pytest prints `no tests ran` and `exit=5`. (Exit 5 = "no tests collected"; the workflow treats it as pass until the first test exists.)

- [ ] **Step 3: Verify the workflow's exit-5 guard turns that into success**

Run:
```bash
uv run pytest tests/unit || { code=$?; [ "$code" -eq 5 ] && echo "::notice::no unit tests yet" || exit "$code"; } ; echo "guarded-exit=$?"
```
Expected: prints the `::notice::` line and `guarded-exit=0`. (This is the exact shell the `test` job uses.)

- [ ] **Step 4: Verify `--strict-config` didn't reject the config**

The fact that Step 2 ran at all (rather than erroring with `INTERNALERROR`/`unknown config option`) confirms `[tool.pytest.ini_options]` parsed under `--strict-config`. No extra command needed.

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml
git commit -m "Configure pytest for the ROS-free unit suite"
```

---

### Task 6: `python-quality.yml` (Layer A workflow)

**Files:**
- Create: `.github/workflows/python-quality.yml`

- [ ] **Step 1: Create the workflow file**

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
      # Python exists; until then, skip with a notice.
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
      # HARD GATE — fail if any single function exceeds rank B (CC > 10). No average gates.
      - name: Complexity hard gate (xenon)
        run: >
          uvx --with radon==6.0.1 xenon@0.9.3
          --max-absolute B
          -i "external,build,install,log"
          analysis scripts tests ros2_ws/src
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
      - name: Unit tests
        run: uv run pytest tests/unit || { code=$?; [ "$code" -eq 5 ] && echo "::notice::no unit tests yet" || exit "$code"; }

  shellcheck:
    runs-on: ubuntu-24.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      - uses: ludeeus/action-shellcheck@00cae500b08a931fb5698e11e79bfbd38e612a38 # 2.0.0
```

- [ ] **Step 2: Install actionlint (one-time, for local workflow validation)**

Run: `command -v actionlint || brew install actionlint`
Expected: `actionlint` is on PATH afterward. (If you can't install it, fall back to the YAML parse in Step 3b; actionlint is strongly preferred because it validates Actions schema, not just YAML syntax.)

- [ ] **Step 3a: Lint the workflow with actionlint**

Run: `actionlint .github/workflows/python-quality.yml`
Expected: no output, exit 0. (actionlint flags bad `uses:` refs, unknown keys, shell issues.)

- [ ] **Step 3b: Fallback — validate YAML well-formedness if actionlint is unavailable**

Run: `uv run python -c "import yaml,sys; yaml.safe_load(open('.github/workflows/python-quality.yml')); print('yaml ok')"`
Expected: prints `yaml ok`, exit 0.

- [ ] **Step 4: Commit**

```bash
git add .github/workflows/python-quality.yml
git commit -m "Add Layer-A python quality workflow"
```

---

### Task 7: `ros-ci.yml` (Layer B workflow)

**Files:**
- Create: `.github/workflows/ros-ci.yml`

- [ ] **Step 1: Resolve and pin the current `actions/upload-artifact` SHA**

Run:
```bash
gh api repos/actions/upload-artifact/releases/latest --jq '.tag_name'
gh api repos/actions/upload-artifact/git/refs/tags/$(gh api repos/actions/upload-artifact/releases/latest --jq '.tag_name') --jq '.object'
```
Expected: prints the latest tag (e.g. `v4.x.y`) and an object. If `.object.type` is `"commit"`, use `.object.sha` directly. If it's `"tag"` (annotated), dereference: `gh api repos/actions/upload-artifact/git/tags/<that-sha> --jq '.object.sha'` to get the **commit** SHA. Record the commit SHA + tag for Step 2. (Same annotated-vs-lightweight trap that bit `actions/checkout` in the spec.)

- [ ] **Step 2: Create the workflow file** (substitute the resolved `upload-artifact` SHA + tag comment for `<SHA>`/`<TAG>`)

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
      image: rostooling/setup-ros-docker:ubuntu-noble-latest   # Noble = 24.04
    steps:
      - uses: actions/checkout@df4cb1c069e1874edd31b4311f1884172cec0e10 # v6.0.3
      # Bootstrap guard: no ROS packages exist until M3. Skip cleanly so the job is
      # GREEN on the empty repo and auto-activates when the first package.xml lands.
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
          # rosdep-skip-keys: px4_msgs   # uncomment ONLY if rosdep flags px4_msgs
      - if: ${{ always() && steps.pkgs.outputs.present == 'true' }}
        uses: actions/upload-artifact@<SHA> # <TAG>
        with:
          name: colcon-logs
          path: ${{ steps.action_ros_ci.outputs.ros-workspace-directory-name }}/log
```

- [ ] **Step 3: Lint the workflow**

Run: `actionlint .github/workflows/ros-ci.yml`
Expected: no output, exit 0.
(Fallback if no actionlint: `uv run python -c "import yaml; yaml.safe_load(open('.github/workflows/ros-ci.yml')); print('yaml ok')"`.)

- [ ] **Step 4: Sanity-check the bootstrap guard logic locally**

Run: `[ -n "$(find ros2_ws/src -name 'package.xml' 2>/dev/null | head -n1)" ] && echo present=true || echo present=false`
Expected: `present=false` (no packages yet → the job will skip the build and report green).

- [ ] **Step 5: Commit**

```bash
git add .github/workflows/ros-ci.yml
git commit -m "Add Layer-B colcon ROS CI workflow"
```

---

### Task 8: `sitl-nightly.yml` (deferred scaffold)

**Files:**
- Create: `.github/workflows/sitl-nightly.yml`

- [ ] **Step 1: Create the scaffold workflow**

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

- [ ] **Step 2: Lint the workflow**

Run: `actionlint .github/workflows/sitl-nightly.yml`
Expected: no output, exit 0.
(Fallback: `uv run python -c "import yaml; yaml.safe_load(open('.github/workflows/sitl-nightly.yml')); print('yaml ok')"`.)

- [ ] **Step 3: Commit**

```bash
git add .github/workflows/sitl-nightly.yml
git commit -m "Add deferred SITL nightly workflow scaffold"
```

---

### Task 9: `dependabot.yml` (keep action pins fresh)

**Files:**
- Create: `.github/dependabot.yml`

- [ ] **Step 1: Create the Dependabot config**

```yaml
version: 2
updates:
  - package-ecosystem: github-actions
    directory: "/"
    schedule:
      interval: weekly
    commit-message:
      prefix: ci
```

- [ ] **Step 2: Validate YAML well-formedness**

Run: `uv run python -c "import yaml; yaml.safe_load(open('.github/dependabot.yml')); print('yaml ok')"`
Expected: prints `yaml ok`, exit 0. (Dependabot's full schema is validated by GitHub on push; YAML well-formedness is the local floor.)

- [ ] **Step 3: Commit**

```bash
git add .github/dependabot.yml
git commit -m "Add Dependabot to update GitHub Actions pins"
```

---

### Task 10: Push, open PR, and confirm CI is green

> **Outward-facing — get the user's go-ahead before pushing.** This publishes the branch and triggers GitHub Actions.

**Files:** none

- [ ] **Step 1: Confirm a GitHub remote exists**

Run: `git -C /Users/jxstanford/devel/SentientSwarm/patrol-drone remote -v`
Expected: an `origin` pointing at the GitHub repo. If none, the workflows can't run on GitHub yet — stop and surface this to the user (the local verifications in Tasks 1–9 are the confidence floor until a remote exists).

- [ ] **Step 2: Push the branch**

Run: `git push -u origin ci/github-actions`
Expected: branch pushed; GitHub prints a PR-creation URL.

- [ ] **Step 3: Open a PR**

Run:
```bash
gh pr create --base main --head ci/github-actions \
  --title "Add CI workflows (testing, linting, complexity, type safety)" \
  --body "Two-layer CI per docs/design/2026-06-03-ci-workflows-design.md. Layer A (ruff/mypy/xenon/pytest/shellcheck) + Layer B (colcon via action-ros-ci) + deferred SITL nightly scaffold. Green on the current empty skeleton; activates as M3 packages land."
```
Expected: prints the new PR URL.

- [ ] **Step 4: Watch the runs to completion**

Run: `gh pr checks ci/github-actions --watch`
Expected: every check resolves to ✓ — `lint`, `types`, `complexity`, `test`, `shellcheck` (Layer A), and `colcon_build_test` (Layer B, which skips the build with a notice and reports success). The nightly does not run on PRs.

- [ ] **Step 5: If any check is red, debug from the logs (do not merge red)**

Run: `gh run view --log-failed` (or open the run from the PR). Fix the offending file, commit, push; the checks re-run. Repeat until green. Common first-run issues to check: an action SHA typo, a runner-only path difference, or `uv.lock` drift (re-run `uv lock` and commit if so).

No new commit unless a fix was needed.

---

### Task 11: ADR-0002 — record the CI decision

The repo working agreement: non-obvious technical calls get a short ADR. Capture this one now that it's implemented.

**Files:**
- Create: `docs/decisions/0002-ci-architecture.md`

- [ ] **Step 1: Create the ADR** (match the ADR-0001 structure)

```markdown
# ADR-0002: Two-layer CI with a xenon complexity gate

**Status:** Accepted
**Date:** 2026-06-03
**Deciders:** Project team

## Context

The repo needs CI for four quality pillars — testing, linting, complexity, and
type safety — for a ROS 2 Jazzy + Python 3.12 monorepo whose code splits into
ROS-independent Python (mission state machine, analysis, scripts) and
ROS-dependent nodes that import rclpy and generated message packages. The latter
cannot be imported, type-checked, or unit-tested without a full ROS 2
environment. SITL integration tests are slow and flaky on hosted runners.

## Decision

Adopt a **two-layer CI**:

- **Layer A (`python-quality.yml`)** — fast pure-Python checks on a plain
  `ubuntu-24.04` runner via `uv`: ruff (lint + format), mypy (types, with ROS
  import roots exempted), a **xenon** per-function complexity hard gate
  (`--max-absolute B`, no average gates), pytest on the ROS-free unit suite, and
  shellcheck.
- **Layer B (`ros-ci.yml`)** — `colcon build` + `colcon test` in a ROS 2 Jazzy
  container via `ros-tooling/action-ros-ci`; ament's Python linters disabled in
  favor of ruff.
- **SITL (`sitl-nightly.yml`)** — deferred scaffold on manual/nightly triggers;
  never a per-PR or required check.

Tool/action versions are pinned (actions to commit SHAs; Dependabot bumps them).

## Consequences

### Positive
- Fast feedback on the bulk of the code without a ROS toolchain.
- The slow/flaky SITL tier is isolated from per-PR CI.
- Per-function complexity gating avoids coupling a PR to unrelated, previously
  accepted code (an average gate would flip green→red on additive effects).
- Green on the empty skeleton; jobs auto-activate as packages and tests land.

### Negative
- Two layers to maintain; a small class of cross-layer issues (e.g. a typo in a
  rclpy call path) is only caught in Layer B.
- mypy on ROS-touching code relies on import-root exemptions rather than real
  stubs, so type coverage of the ROS boundary is shallow until stubs exist.

### Neutral
- xenon thresholds live as CLI flags (no config file); the single source of
  truth is `python-quality.yml`.

## References
- Design spec: `docs/design/2026-06-03-ci-workflows-design.md`
- ADR-0001 (distro/OS): `docs/decisions/0001-distro-and-os.md`
```

- [ ] **Step 2: Commit**

```bash
git add docs/decisions/0002-ci-architecture.md
git commit -m "Record CI architecture decision as ADR-0002"
```

- [ ] **Step 3: Push the ADR to the PR**

Run: `git push`
Expected: the ADR commit appears on the PR; Layer-A checks re-run and stay green.

---

## Notes carried forward to M3 (not part of this plan)

These are tracked in the spec (§9–§10) and become live when the first ROS
packages exist — do **not** attempt them now:

1. Enable `mypy_path` / pytest `pythonpath` for the ROS-free `patrol_mission`
   modules (spike: editable install vs path entry).
2. Confirm whether `rosdep-skip-keys: px4_msgs` is needed on the first real
   Layer-B build.
3. Pin the `rostooling/setup-ros-docker` image to a `sha256` digest.
4. Confirm the `package-name` list matches the actual M3 package directory names.
5. Configure branch protection in GitHub settings: require `lint`, `types`,
   `complexity`, `test`, `shellcheck`, `colcon_build_test`; not the nightly.
6. Disable ament's Python linters (flake8 / pep257) in the M3 ROS packages in
   favor of ruff, keeping `ament_xmllint` (see spec §5 for the exact mechanism).
