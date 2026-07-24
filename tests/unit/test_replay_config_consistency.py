"""Forward-compat config-consistency guard for the replay gate (F-06, Mira review 4731322384).

The replay regression tests (tests/replay/test_replay_regression.py) play the FROZEN checked-in
reference bag — that proves BACKWARD compatibility (a known-good artifact still passes the
comparator) but structurally cannot catch a topic dropped or renamed in the CURRENT recorder
config: the frozen bag always contains the old topics under their old names. This Layer-A guard
closes that gap at the source: every topic asserted in tests/replay/assertions.yaml must still be
covered by ros2_ws/src/patrol_logging/config/recorded_topics.yaml — either as an explicit recorded
topic or matched by a recorded regex (e.g. ``/fmu/out/.*`` covers
``/fmu/out/vehicle_local_position_v1``). ROS-free (yaml + re + the comparator's ``load_specs``), so
it runs in the fast unit tier and catches the drift even in a run that never materializes the bag.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml
from replay_assertions import load_specs

_REPO = Path(__file__).resolve().parents[2]
_ASSERTIONS = _REPO / "tests" / "replay" / "assertions.yaml"
_RECORDED_TOPICS = _REPO / "ros2_ws" / "src" / "patrol_logging" / "config" / "recorded_topics.yaml"


def _configured_topics() -> tuple[set[str], list[str]]:
    """The recorder's current config: (explicit exact topic names, regex patterns)."""
    cfg = yaml.safe_load(_RECORDED_TOPICS.read_text())
    return set(cfg.get("topics", [])), list(cfg.get("regexes", []))


# TS-21 (F-06): FORWARD-compat guard. The reference-bag tests prove BACKWARD compat; this proves the
# CURRENT recorder config still covers every asserted topic, so dropping/renaming a topic in
# recorded_topics.yaml fails HERE at its source — no SITL, no bag. An asserted topic is "covered" if
# it is an explicit recorded topic OR matched by a recorded regex.
def test_asserted_topics_are_in_current_recorder_config() -> None:
    exact, regexes = _configured_topics()
    patterns = [re.compile(r) for r in regexes]
    asserted = {s.topic for s in load_specs(_ASSERTIONS)}
    uncovered = {
        t for t in asserted if t not in exact and not any(p.fullmatch(t) for p in patterns)
    }
    assert not uncovered, (
        f"assertions.yaml asserts topics the recorder no longer records: {sorted(uncovered)} — "
        "a dropped/renamed topic in recorded_topics.yaml would make the replay gate assert a topic "
        "the current config can't produce (F-06 forward-compat)."
    )
