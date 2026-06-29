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
| `assertions.yaml` | The curated asserted subset + expected rates (measured from the reference bag). |
| `reference/patrol_reference/` | The checked-in reference bag (MCAP), tracked via **Git LFS** (`.gitattributes`). |
| `reference/make_reference_bag.py` | The regeneration script (LR-9) — re-run to refresh the baseline. |

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
