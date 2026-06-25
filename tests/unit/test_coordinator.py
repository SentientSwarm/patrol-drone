"""Unit tests for CaptureCoordinator orchestration + the AC-6 latch (M6.B, T B.3 — design §4.2.8).

Layer-A: no rclpy/Gazebo/PX4. All collaborators are injected fakes, so the latch logic, the ADR-A
tag-in-view gate, and the "latch only on success -> skips stay retryable" invariant (§4.2.8) are
exercised directly (AC-5/AC-6). The detection's id/family/decision_margin duck-type
apriltag_msgs/msg/AprilTagDetection (T B.1).
"""

from dataclasses import replace
from types import SimpleNamespace

import pytest
from patrol_perception.checkpoint_resolver import CheckpointResolverError
from patrol_perception.coordinator import CaptureCoordinator, CapturePipeline, FreshnessWindows
from patrol_perception.samplers import PoseSample

# Receipt timestamp the fakes stamp on their buffered values (ADR-B freshness window). The coordinator
# ages each buffer against its clock; with the default fixed clock below the buffers read as "fresh".
_RX_FRESH = (123, 0)
# Generous default windows so the happy-path fakes never trip the freshness gate; individual stale
# tests pass a tighter window (via _tight_window) or an older receipt time to drive the stale branches.
_FRESH_WINDOWS = FreshnessWindows(detection_s=100.0, frame_s=100.0, pose_s=100.0)


def _tight_window(stale_stream):
    """A FreshnessWindows like _FRESH_WINDOWS but with one stream's window tightened to 0.5 s, so an
    old receipt time on exactly that stream reads as stale while the other two stay fresh."""
    field = f"{stale_stream}_s"
    return replace(_FRESH_WINDOWS, **{field: 0.5})


def _detection(tag_id=0, family="tag36h11"):
    return SimpleNamespace(id=tag_id, family=family, decision_margin=42.0, hamming=0)


def _pose():
    return PoseSample(position=(1.0, 2.0, 3.0), orientation=(0.0, 0.0, 0.0, 1.0), frame_id="w")


def _frame_fake(frame):
    """A FrameSampler fake: take_latest() returns (image_msg, bytes, received_at) | None (Fix 2)."""
    take_latest = (lambda: None) if frame is None else (lambda: (*frame, _RX_FRESH))
    return SimpleNamespace(take_latest=take_latest)


def _pose_fake(pose):
    """A PoseSampler fake: sample() returns (PoseSample, received_at) | None (Fix 2)."""
    sample = (lambda: None) if pose is None else (lambda: (pose, _RX_FRESH))
    return SimpleNamespace(sample=sample)


class _DetectionBufferFake:
    """A stateful detection LatestBuffer fake (Fix 2): ``latest_at()`` returns
    ``(detections, received_at) | None``, and ``update(None)`` genuinely *expires* it — mirroring the
    real LatestBuffer so the after-capture detection-expiry (decision #6) is honestly exercised, not
    masked by a no-op fake."""

    def __init__(self, detections):
        self._detections = detections

    def update(self, value):
        self._detections = value

    def latest_at(self):
        if not self._detections:
            return None
        return self._detections, _RX_FRESH


def _detection_fake(detections):
    return _DetectionBufferFake(detections)


class _Recorder:
    """Captures published messages and written records so tests can assert cardinality/content."""

    def __init__(self):
        self.published = []
        self.written = []


_UNSET = object()


def _make_coordinator(recorder, *, detections=None, freshness=_FRESH_WINDOWS):
    """Build a coordinator wired to fresh-stream fakes. ``detections`` None => no tag in view (gate
    skip); ``freshness`` overrides the per-stream windows (default: all generous/fresh). Tests that
    need an *absent* or *stale* frame/pose reassign ``coord._frame_sampler`` / ``coord._pose_sampler``
    on the instance (see the no-frame / no-pose / stale-stream cases)."""
    resolver = SimpleNamespace(resolve=lambda det: ("checkpoint_alpha", {"tag_id": str(det.id)}))
    builder = SimpleNamespace(
        build_message=lambda rec: SimpleNamespace(checkpoint_id=rec.checkpoint_id),
        build_sidecar=lambda rec: {"checkpoint_id": rec.checkpoint_id},
    )
    return CaptureCoordinator(
        pipeline=CapturePipeline(
            frame_sampler=_frame_fake(("img", b"bytes")),
            pose_sampler=_pose_fake(_pose()),
            detection_buffer=_detection_fake(detections),
            resolver=resolver,
            builder=builder,
            publisher=SimpleNamespace(publish=recorder.published.append),
            writer=None,  # M6.C wires the CaptureWriter; M6.B publishes only
        ),
        clock=lambda: (123, 456),
        freshness=freshness,
        mission_id="run42",
    )


# --- happy path: one capture, token latched ---


def test_successful_trigger_publishes_one_capture():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1
    assert rec.published[0].checkpoint_id == "checkpoint_alpha"


# --- AC-6: duplicate trigger for a latched visit is suppressed ---


def test_duplicate_trigger_same_token_suppressed():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord.on_trigger(visit_token=1)
    coord.on_trigger(visit_token=1)  # same visit -> no second capture
    assert len(rec.published) == 1


def test_new_token_rearms_after_success():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord.on_trigger(visit_token=1)
    # A successful capture expires the detection buffer (decision #6); in the live node the detector
    # keeps streaming, so a fresh detection is buffered before the next checkpoint's trigger.
    coord._detection_buffer.update([_detection()])
    coord.on_trigger(visit_token=2)  # next checkpoint / re-visit -> captures again
    assert len(rec.published) == 2


# --- skips leave the token UNLATCHED (retryable), produce zero captures (§4.2.8) ---


def test_no_tag_in_view_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=None)  # ADR-A gate: nothing in view
    coord.on_trigger(visit_token=1)
    assert rec.published == []
    # re-trigger same token after a tag comes into view -> retry succeeds (not latched by the skip)
    coord._detection_buffer = _detection_fake([_detection()])
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_no_frame_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord._frame_sampler = _frame_fake(None)  # no frame buffered yet (absent stream)
    coord.on_trigger(visit_token=1)
    assert rec.published == []
    coord._frame_sampler = _frame_fake(("img", b"bytes"))
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_no_pose_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord._pose_sampler = _pose_fake(None)  # no pose buffered yet (absent stream)
    coord.on_trigger(visit_token=1)
    assert rec.published == []


def test_unmapped_tag_id_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection(tag_id=99)])
    # resolver rejects the unmapped tag (AC-4: no fabricated checkpoint_id)

    def _raise(_det):
        raise CheckpointResolverError("unmapped tag_id 99")

    coord._resolver = SimpleNamespace(resolve=_raise)
    coord.on_trigger(visit_token=1)
    assert rec.published == []
    # the visit is not latched -> a later resolvable trigger for the same token retries
    coord._resolver = SimpleNamespace(resolve=lambda det: ("checkpoint_alpha", {}))
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def _spy_builder(captured: list) -> SimpleNamespace:
    """A builder fake that records each CaptureRecord it is asked to build (shared by tests)."""

    def _build_message(record):
        captured.append(record)
        return SimpleNamespace(checkpoint_id=record.checkpoint_id)

    return SimpleNamespace(build_message=_build_message, build_sidecar=lambda _: {})


def _raising_writer() -> SimpleNamespace:
    """A writer fake whose write() raises OSError (disk full / unwritable root) — shared by the
    §4.4.5 degradation tests so they don't copy-paste the raising setup (CodeScene duplication)."""

    def _write(_record, _image_bytes):
        raise OSError("disk full")

    return SimpleNamespace(write=_write)


def test_writer_path_image_path_flows_into_published_record():
    # M6.B->M6.C seam: when a CaptureWriter is wired, its returned path lands in the published msg.
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])

    def _write(record, _image_bytes):
        rec.written.append(record)
        return "/run/000_alpha.png"

    coord._writer = SimpleNamespace(write=_write)
    captured: list = []
    coord._builder = _spy_builder(captured)
    coord.on_trigger(visit_token=1)
    assert len(rec.written) == 1
    assert captured[0].image_path == "/run/000_alpha.png"
    assert captured[0].metadata["image_write_status"] == "ok"  # success path flags the write


# --- §4.4.5: a writer failure still publishes (topic is the bag's source of truth), flagged ---


def test_writer_failure_still_publishes_flagged_unwritten():
    # design §4.4.5 row 1: an OSError on write() must NOT suppress the publish; the message goes out
    # with an empty image_path and image_write_status="failed" (continue patrol).
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord._writer = _raising_writer()
    captured: list = []
    coord._builder = _spy_builder(captured)
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1  # exactly one capture published despite the write failure
    assert captured[0].image_path == ""  # flagged unwritten
    assert captured[0].metadata["image_write_status"] == "failed"


def test_writer_failure_latches_visit_no_republish():
    # AC-6 on the degraded path: a published-but-unwritten capture still latches the visit, so a
    # re-trigger for the same token does NOT publish a second time.
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord._writer = _raising_writer()
    coord.on_trigger(visit_token=1)
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_metadata_merges_mission_id_waypoint_index_and_detection():
    # T C.3 / PCAP-6/PCAP-7: the record metadata carries mission_id + waypoint_index (the visit
    # token) merged with the resolver's detection metadata (tag_id, confidence).
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection(tag_id=7)])
    captured: list = []
    coord._builder = _spy_builder(captured)
    coord.on_trigger(visit_token=3)
    md = captured[0].metadata
    assert md["mission_id"] == "run42"
    assert md["waypoint_index"] == "3"
    assert md["tag_id"] == "7"  # resolver-sourced metadata is preserved


def test_capture_record_carries_resolved_id_position_and_frame():
    # The record threads the resolved checkpoint_id, the sampled world/ENU position, the pose
    # frame_id, and the clock stamp. (Orientation-frame correctness is owned/asserted by the
    # PoseSampler tests — the coordinator just forwards PoseSample.orientation verbatim.)
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    captured: list = []
    coord._builder = _spy_builder(captured)
    coord.on_trigger(visit_token=1)
    record = captured[0]
    assert record.checkpoint_id == "checkpoint_alpha"
    assert record.position == (1.0, 2.0, 3.0)
    assert record.orientation == (0.0, 0.0, 0.0, 1.0)  # forwarded from the PoseSample unchanged
    assert record.frame_id == "w"
    assert record.stamp_sec == 123
    assert record.stamp_nanosec == 456


# --- ADR-B freshness gate: a STALE-but-present buffer is skipped exactly like an absent one ---
# These guard Hermes 4566902694: a stalled detector/frame/pose stream must NOT publish a stale tuple
# for a different visit. The coordinator clock is (123, 456); a receipt time well in the past makes
# the chosen stream's buffer exceed its (tight) freshness window -> skip, token unlatched (§4.4.5).

_OLD_RX = (0, 0)  # received "long ago" relative to the coordinator clock (123, 456) -> ~123 s old


def _stale_one_stream_coordinator(rec, stale_stream):
    """Coordinator with all streams present + a tight window on ``stale_stream`` whose buffer is old,
    so exactly that one stream reads as stale (the other two stay fresh)."""
    coord = _make_coordinator(rec, detections=[_detection()], freshness=_tight_window(stale_stream))
    if stale_stream == "detection":
        coord._detection_buffer = SimpleNamespace(
            latest_at=lambda: ([_detection()], _OLD_RX), update=lambda _v: None
        )
    elif stale_stream == "frame":
        coord._frame_sampler = SimpleNamespace(take_latest=lambda: ("img", b"bytes", _OLD_RX))
    else:  # pose
        coord._pose_sampler = SimpleNamespace(sample=lambda: (_pose(), _OLD_RX))
    return coord


@pytest.mark.parametrize("stale_stream", ["detection", "frame", "pose"])
def test_stale_stream_skips_and_stays_retryable(stale_stream):
    # A stalled detection/frame/pose stream skips the visit (zero publishes) and leaves the token
    # UNLATCHED, so a re-trigger retries once the stream resumes (mirrors the absent-buffer skips).
    rec = _Recorder()
    coord = _stale_one_stream_coordinator(rec, stale_stream)
    coord.on_trigger(visit_token=1)
    assert rec.published == []  # stale tuple is NOT published for this visit
    # the skip did not latch -> once all three streams are fresh, the same token captures once
    fresh = _make_coordinator(rec, detections=[_detection()])
    fresh.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_repeated_visit_unchanged_buffers_not_republished_for_new_token():
    # The exact probe Hermes reproduced: visit 1 captures, then WITHOUT any new detection/frame/pose
    # the next visit token must NOT re-publish the stale tuple (both visits were ['checkpoint_alpha']).
    # Here the detection buffer is expired on the first capture (decision #6) AND no stream refreshes,
    # so visit 2 finds no in-view tag and skips -> exactly one publish total, not two.
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1
    coord.on_trigger(visit_token=2)  # new token, but the buffers were never refreshed
    assert len(rec.published) == 1  # the stale tuple is NOT re-published for visit 2


def test_detection_buffer_expired_after_successful_capture():
    # decision #6 / §4.4.6: a successful capture clears the detection buffer (update(None)) so a
    # no-longer-visible tag can't be reused on the next trigger, even within its freshness window.
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=[_detection()])
    cleared: list = []
    coord._detection_buffer = SimpleNamespace(
        latest_at=lambda: ([_detection()], _RX_FRESH), update=cleared.append
    )
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1
    assert cleared == [None]  # the buffer was expired with update(None) after latching
