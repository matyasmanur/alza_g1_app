"""Bounded joint-space paths and a two-route gesture planner."""

from dataclasses import dataclass
from math import ceil, sqrt
from typing import Callable

from g1_control.joints import JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED
from g1_control.transport import Angles, Feedback

READY = (0.0,) * len(JOINT_NAMES)
SAMPLE_STEP = 0.05
Clearance = Callable[[Angles, Angles], str | None]


@dataclass(frozen=True)
class Keyframe:
    """An arm pose and the minimum travel time from the preceding pose."""

    pose: Angles
    seconds: float


@dataclass(frozen=True)
class Segment:
    """Synchronize all joints with a quintic rest-to-rest interpolation."""

    start: Angles
    end: Angles
    seconds: float

    @classmethod
    def between(cls, start: Angles, frame: Keyframe):
        """Stretch timing to respect the peak velocity and acceleration."""
        distance = max(abs(a - b) for a, b in zip(start, frame.pose))
        seconds = max(frame.seconds, 1.875 * distance / MAX_SPEED,
                      sqrt(5.774 * distance / MAX_ACCELERATION))
        return cls(start, frame.pose, seconds)

    def sample(self, elapsed: float) -> tuple[Angles, Angles]:
        t = min(elapsed / self.seconds, 1.0)
        fraction = t**3 * (10 - 15*t + 6*t*t)
        rate = 30 * t*t * (1 - t)**2 / self.seconds
        position = tuple(a + (b - a)*fraction for a, b in zip(self.start, self.end))
        velocity = tuple((b - a)*rate for a, b in zip(self.start, self.end))
        return position, velocity


@dataclass(frozen=True)
class PlanRequest:
    """Immutable controller snapshot for off-loop planning."""

    name: str
    feedback: Feedback
    commanded: Angles
    frames: tuple[Keyframe, ...]


class UnsafePath(ValueError):
    """No checked route could be accepted."""


def path_issue(clearance: Clearance, start: Angles, end: Angles, lower: Angles) -> str | None:
    """Sample a straight joint-space segment, including both endpoints."""
    parts = max(1, ceil(max(abs(a - b) for a, b in zip(start, end)) / SAMPLE_STEP))
    for index in range(parts + 1):
        fraction = index / parts
        pose = tuple(a + (b - a)*fraction for a, b in zip(start, end))
        issue = clearance(pose, lower)
        if issue:
            return issue
    return None


def plan(request: PlanRequest, clearance: Clearance | None) -> tuple[Segment, ...]:
    """Try a direct route, then via Ready, and check the full return path."""
    ready = Keyframe(READY, 0.35)
    reason = None
    for frames in (request.frames + (ready,), (ready,) + request.frames + (ready,)):
        start = request.commanded
        result = []
        if clearance is not None:
            reason = path_issue(clearance, request.feedback.position, frames[0].pose,
                                request.feedback.lower_position)
        if reason:
            continue
        for frame in frames:
            if clearance is not None:
                reason = path_issue(clearance, start, frame.pose, request.feedback.lower_position)
                if reason:
                    break
            result.append(Segment.between(start, frame))
            start = frame.pose
        else:
            return tuple(result)
    raise UnsafePath(f"No clear route to complete the gesture. {reason}")
