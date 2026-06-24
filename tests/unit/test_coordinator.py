"""Unit tests for CaptureCoordinator orchestration + the AC-6 latch (M6.B, T B.3 — design §4.2.8).

Layer-A: no rclpy/Gazebo/PX4. All collaborators are injected fakes, so the latch logic, the ADR-A
tag-in-view gate, and the "latch only on success -> skips stay retryable" invariant (§4.2.8) are
exercised directly (AC-5/AC-6). The detection's id/family/decision_margin duck-type
apriltag_msgs/msg/AprilTagDetection (T B.1).
"""

from types import SimpleNamespace

from patrol_perception.checkpoint_resolver import CheckpointResolverError
from patrol_perception.coordinator import CaptureCoordinator, CapturePipeline
from patrol_perception.samplers import PoseSample


def _detection(tag_id=0, family="tag36h11"):
    return SimpleNamespace(id=tag_id, family=family, decision_margin=42.0, hamming=0)


def _pose():
    return PoseSample(position=(1.0, 2.0, 3.0), orientation=(0.0, 0.0, 0.0, 1.0), frame_id="w")


class _Recorder:
    """Captures published messages and written records so tests can assert cardinality/content."""

    def __init__(self):
        self.published = []
        self.written = []


_UNSET = object()


def _make_coordinator(recorder, *, frame=("img", b"bytes"), pose=_UNSET, detections=None):
    """Build a coordinator wired to fakes. ``detections`` None => no tag in view (gate skip)."""
    pose = _pose() if pose is _UNSET else pose
    resolver = SimpleNamespace(resolve=lambda det: ("checkpoint_alpha", {"tag_id": str(det.id)}))
    builder = SimpleNamespace(
        build_message=lambda rec: SimpleNamespace(checkpoint_id=rec.checkpoint_id),
        build_sidecar=lambda rec: {"checkpoint_id": rec.checkpoint_id},
    )
    return CaptureCoordinator(
        pipeline=CapturePipeline(
            frame_sampler=SimpleNamespace(take_latest=lambda: frame),
            pose_sampler=SimpleNamespace(sample=lambda: pose),
            detection_buffer=SimpleNamespace(latest=lambda: detections),
            resolver=resolver,
            builder=builder,
            publisher=SimpleNamespace(publish=recorder.published.append),
            writer=None,  # M6.C wires the CaptureWriter; M6.B publishes only
        ),
        clock=lambda: (123, 456),
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
    coord.on_trigger(visit_token=2)  # next checkpoint / re-visit -> captures again
    assert len(rec.published) == 2


# --- skips leave the token UNLATCHED (retryable), produce zero captures (§4.2.8) ---


def test_no_tag_in_view_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, detections=None)  # ADR-A gate: nothing in view
    coord.on_trigger(visit_token=1)
    assert rec.published == []
    # re-trigger same token after a tag comes into view -> retry succeeds (not latched by the skip)
    coord._detection_buffer = SimpleNamespace(latest=lambda: [_detection()])
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_no_frame_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, frame=None, detections=[_detection()])
    coord.on_trigger(visit_token=1)
    assert rec.published == []
    coord._frame_sampler = SimpleNamespace(take_latest=lambda: ("img", b"bytes"))
    coord.on_trigger(visit_token=1)
    assert len(rec.published) == 1


def test_no_pose_skips_and_stays_retryable():
    rec = _Recorder()
    coord = _make_coordinator(rec, pose=None, detections=[_detection()])
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
