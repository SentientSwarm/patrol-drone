"""Shared rclpy context-lifecycle stand-ins for the node-glue seam tests (F-02).

Both entrypoints — ``patrol_mission.node.main`` and ``patrol_perception.perception_node.main`` —
must exit **0** on a clean teardown. The runner group-SIGINTs the mission launch once
``verify_patrol.py`` observes the landing, so the normal end of a *successful* patrol is an external
shutdown: ``rclpy.spin`` raises ``ExternalShutdownException`` because the context went down beneath
it. A launch that always exits non-zero on the happy path trains you to ignore its exit code, so a
genuine crash during teardown becomes indistinguishable from a normal run.

The fake models the *real* context semantics rather than asserting on call counts, so the tests fail
for the same reason the live nodes did:

* ``shutdown()`` raises ``RCLError`` when the context is already down (the real
  ``rcl_shutdown already called on the given context`` at ``rcl/init.c:333``);
* ``try_shutdown()`` is the idempotent form and is a no-op in that state.

Lives here, not in either test module, because the two glue suites would otherwise carry
near-identical copies of the fake and its assertions (CodeScene duplication) — while the *node*
patches stay genuinely independent: the mission node needed both the ``except`` clause and the
``try_shutdown`` swap, the perception node only the ``except`` clause.
"""

from __future__ import annotations

from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from typing import Any


class RCLError(RuntimeError):
    """Stand-in for ``rclpy._rclpy_pybind11.RCLError`` (raised by a double ``shutdown()``)."""


class ExternalShutdownException(Exception):
    """Stand-in for ``rclpy.executors.ExternalShutdownException``.

    Test modules must stub ``rclpy.executors`` with *this* class so the node module's
    ``from rclpy.executors import ExternalShutdownException`` binds to the object the fake raises.
    """


class FakeRclpy:
    """Minimal ``rclpy`` module surface that tracks whether the context is up."""

    def __init__(self) -> None:
        self.context_up = False
        self.spin_raises: BaseException | None = None
        self.shutdown_calls = 0
        self.try_shutdown_calls = 0

    def init(self, **_kw: Any) -> None:
        # Both entrypoints call `rclpy.init(args=args)`; the kwarg is absorbed and ignored.
        self.context_up = True

    def spin(self, _node: Any) -> None:
        if self.spin_raises is None:
            return
        # Ordering is the whole point: an EXTERNAL shutdown takes the context down *before* spin
        # reports it, so the `finally` then runs against an already-down context. A plain
        # KeyboardInterrupt leaves the context up, which is why that path never showed the bug.
        if isinstance(self.spin_raises, ExternalShutdownException):
            self.context_up = False
        raise self.spin_raises

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        if not self.context_up:
            raise RCLError(
                "failed to shutdown: rcl_shutdown already called on the given context, "
                "at ./src/rcl/init.c:333"
            )
        self.context_up = False

    def try_shutdown(self) -> None:
        self.try_shutdown_calls += 1
        self.context_up = False

    def as_module(self, name: str = "rclpy") -> ModuleType:
        """This fake exposed as a ``sys.modules`` entry the node module can import."""
        mod = ModuleType(name)
        for attr in ("init", "spin", "shutdown", "try_shutdown"):
            setattr(mod, attr, getattr(self, attr))
        return mod


def executors_module() -> ModuleType:
    """A ``rclpy.executors`` stub exporting the exception class the node catches."""
    mod = ModuleType("rclpy.executors")
    mod.ExternalShutdownException = ExternalShutdownException  # type: ignore[attr-defined]
    return mod


class DestroyRecordingNode:
    """Records ``destroy_node()`` so the teardown order can be asserted."""

    def __init__(self) -> None:
        self.destroyed = 0

    def destroy_node(self) -> None:
        self.destroyed += 1


def run_main_under(
    main: Callable[..., Any],
    fake: FakeRclpy,
    node_ctor_patch: Callable[[Callable[[], Any]], None],
    *,
    spin_raises: BaseException | None,
) -> SimpleNamespace:
    """Drive ``main()`` with ``spin`` raising ``spin_raises``; return the observed teardown facts.

    ``node_ctor_patch`` installs a no-op node factory so ``main`` never builds the real node (which
    would need params, YAML, and publishers). Any exception escaping ``main`` propagates — that IS
    the failure mode under test (a non-zero process exit).
    """
    node = DestroyRecordingNode()
    node_ctor_patch(lambda: node)
    fake.spin_raises = spin_raises
    main()
    return SimpleNamespace(
        destroyed=node.destroyed,
        context_up=fake.context_up,
        shutdown_calls=fake.shutdown_calls,
        try_shutdown_calls=fake.try_shutdown_calls,
    )
