# Replay regression test + reference bag

The replay regression test (`test_replay_regression.py`) is the CI guard the recorded bag becomes:
it plays a small checked-in reference bag through `ros2 bag play`, counts messages per topic, and
asserts the curated subset in [`assertions.yaml`](assertions.yaml) appears at the expected rates.
A later-phase change that drops a recorded topic fails this test in CI before it reaches hardware
(docset 05-logging-replay, M8 / LR-5; design §4.2.5).

## Layout

| File | What |
|------|------|
| `test_replay_regression.py` | The ROS-lane test: play the reference bag → count via rclpy subscribers → evaluate. Runs in CI's `replay-regression` lane (needs a sourced ROS env). |
| `replay_assertions.py` | The **ROS-free** comparator (presence/count + ±tol rate band) + the `assertions.yaml` loader. Unit-tested in `tests/unit/test_replay_assertions.py` (Layer-A). |
| `rate_report.py` | The **ROS-free** RTF-robust rate analyzer for the *live-bag* witness (unique-stamp rate + a bag-consistency guard). Reuses `replay_assertions.evaluate`. Unit-tested in `tests/unit/test_rate_report.py`. |
| `verify_live_bag.py` | The **manual live-bag witness** (`analysis/e2e_check.md` step 4). Run explicitly under system python + sourced ROS — NOT collected by pytest. |
| `assertions.yaml` | The curated asserted subset + expected rates (measured from the reference bag). |
| `reference/patrol_reference/` | The checked-in reference bag (MCAP), tracked via **Git LFS** (`.gitattributes`). |
| `reference/make_reference_bag.py` | The regeneration script (LR-9) — re-run to refresh the baseline. |

## Live-bag witness vs the CI lane

The **CI lane** (`test_replay_regression.py`) plays the checked-in **reference** bag — recorded at
RTF ≈ 1.0 with a clean, self-consistent MCAP — so `count / bag-info-duration` is the true rate and the
`assertions.yaml` bands (measured at RTF ≈ 1.0) apply directly. This lane is deliberately fixed and
must not be loosened for a loaded host.

A **freshly-recorded live** bag is different. On a GUI-loaded host the sim can't hold real-time; the
record path may double-deliver rendered frames and write an inconsistent MCAP summary, so a raw
`ros2 bag info` Hz reads ~2x high (measured: camera 30.3 Hz vs a true 15.15 Hz — see
[ADR-0013](../../docs/decisions/0013-m8-live-bag-rate-witness.md)). The manual witness
`verify_live_bag.py` handles this: it first **hard-fails** (exit 2) any bag whose `ros2 bag info`
counts disagree with its message stream or whose frames are duplicated (a "re-record" signal), then
measures each topic's **true** rate from its own de-duplicated message timestamps (RTF-invariant,
type-agnostic via `log_time`). So a clean bag on a loaded host still passes and a suspect bag never
silently passes — without touching the CI lane or widening `assertions.yaml`. Do NOT verify a live
bag by eyeballing `ros2 bag info` Hz; use `verify_live_bag.py`.

## Reference bag provenance (LR-9)

| Field | Value |
|-------|-------|
| **Source mission** | a full M7 patrol bag with non-zero checkpoint captures (post-[ADR-0012](../../docs/decisions/0012-m4-dwell-pose-camera-framing-fix.md)); reference produced from `patrol_20260627T140037Z_20260627_140037` (15 captures / 161 s). |
| **Trim window** | 20 s, anchored 8 s before the first `/patrol/checkpoint_capture` so the slice contains captures. |
| **Camera downsample** | every 5th `CompressedImage` (imagery present but small). |
| **Result** | ~4.6 MiB MCAP, 20 s, ~2000 msgs; all asserted topics non-zero (incl. `/patrol/checkpoint_capture` = 10). |
| **VC** | Git LFS (`*.mcap` under `reference/`); the repo holds a pointer, CI checks out with `lfs: true`. |

### Regenerate the reference bag

When the recorded topic set legitimately changes (so the baseline must too):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tests/replay/reference/make_reference_bag.py \
    --source ~/patrol_bags/<a-full-patrol-bag-with-captures> \
    --out tests/replay/reference/patrol_reference \
    --seconds 20 --camera-every 5 --lead 8
# then re-measure rates and update assertions.yaml's expected_hz, and re-commit (LFS):
ros2 bag info tests/replay/reference/patrol_reference
git add tests/replay/reference/patrol_reference assertions.yaml
git commit -m "M8: refresh replay reference bag + assertion rates"
```

## Git LFS prerequisite

The reference bag is binary and tracked via Git LFS. **One-time setup on a fresh host:**

```bash
sudo apt-get install -y git-lfs   # apt candidate 3.4.x (noble-updates/universe)
git lfs install                   # registers the LFS filters in this repo's git config
# .gitattributes already declares tests/replay/reference/*.mcap as LFS-tracked.
git add tests/replay/reference/patrol_reference/*.mcap   # stored as an LFS pointer
```

If the LFS pointer fails to resolve in CI (checkout without `lfs: true`), the replay test
**fails loudly** — it does not skip — because a missing reference bag means no regression coverage
(design §4.4.5).
