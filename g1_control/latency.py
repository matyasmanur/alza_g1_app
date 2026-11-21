"""Opt-in timing for isolated, settled-to-settled manual commands."""

from dataclasses import dataclass
from threading import Lock
from time import monotonic

from g1_control.transport import Angles, Feedback

MOTION_THRESHOLD = 0.01


@dataclass
class Sample:
    session: str
    command_id: int
    index: int
    received_at: float
    measured: float
    commanded: float
    direction: int
    written_at: float | None = None
    observed_at: float | None = None


class LatencyTrace:
    """Associate an accepted intent, SDK write and measured response by command ID."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._commanded: Angles | None = None
        self._sample: Sample | None = None

    def start(self, session: str, command_id: int, index: int, target: float,
              feedback: Feedback, received_at: float) -> None:
        """Start timing from the measured position captured on receipt."""
        with self._lock:
            self._sample = None
            measured = feedback.position[index]
            if self._commanded is not None and abs(target - measured) > MOTION_THRESHOLD:
                self._sample = Sample(session, command_id, index, received_at, measured,
                                      self._commanded[index], 1 if target > measured else -1)

    def written(self, position: Angles, enabled: bool) -> None:
        """Timestamp the first successful SDK write that changes the joint target."""
        now = monotonic()
        with self._lock:
            self._commanded = position if enabled else None
            sample = self._sample
            if not enabled:
                self._sample = None
            elif (sample is not None and sample.written_at is None
                  and sample.direction * (position[sample.index] - sample.commanded) > 1e-6):
                sample.written_at = now

    def observe(self, feedback: Feedback) -> None:
        """Record the first valid DDS sample crossing the directional threshold."""
        with self._lock:
            sample = self._sample
            if (sample is not None and sample.written_at is not None and sample.observed_at is None
                and feedback.received_at >= sample.written_at
                and sample.direction * (feedback.position[sample.index] - sample.measured) >= MOTION_THRESHOLD):
                sample.observed_at = feedback.received_at

    def take(self, session: str) -> dict | None:
        """Return one completed measurement to the session that requested it."""
        with self._lock:
            sample = self._sample
            if sample is None or sample.session != session or sample.observed_at is None:
                return None
            self._sample = None
            return {
                "type": "timing", "id": sample.command_id,
                "receive_to_write_ms": (sample.written_at - sample.received_at) * 1000,
                "receive_to_motion_ms": (sample.observed_at - sample.received_at) * 1000,
            }

    def cancel(self, session: str) -> None:
        """Discard incomplete samples when the operator cancels or replaces motion."""
        with self._lock:
            if self._sample is not None and self._sample.session == session:
                self._sample = None
