"""Bounded joint-space paths and a two-route gesture planner."""

from dataclasses import dataclass
from math import ceil, inf, sqrt
from typing import Callable

from g1_control.joints import JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED
from g1_control.transport import Angles, Feedback

READY = (0.0,) * len(JOINT_NAMES)
SAMPLE_STEP = 0.05
MARGIN_METRES = 0.012
Clearance = Callable[[Angles, Angles], str | None]


class Issue(str):
    """A clearance violation that also reports how far inside the margin the pose is, in metres."""

    def __new__(cls, text: str, depth: float):
        issue = super().__new__(cls, text)
        issue.depth = depth
        return issue


class Escape:
    """Tolerate poses inside the clearance margin only while they keep moving out of it.

    Measured poses sag into the margin, and a pose that may not move away can never recover.
    """

    def __init__(self, slack: float = 0.0005) -> None:
        self._slack = slack
        self._depth = inf

    def blocks(self, issue: str | None) -> bool:
        """Report a pose deeper inside the margin than the shallowest pose before it."""
        if issue and not hasattr(issue, "depth"):
            return True
        depth = issue.depth if issue else 0.0
        blocked = depth > self._depth + self._slack
        self._depth = min(self._depth, depth)
        return blocked


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
    end: Angles = READY
    release: bool = False


class UnsafePath(ValueError):
    """No checked route could be accepted."""


def path_issue(clearance: Clearance, start: Angles, end: Angles, lower: Angles) -> str | None:
    """Sample a straight joint-space segment. It may leave the margin but must end clear of it."""
    parts = max(1, ceil(max(abs(a - b) for a, b in zip(start, end)) / SAMPLE_STEP))
    escape = Escape()
    for index in range(parts + 1):
        fraction = index / parts
        pose = tuple(a + (b - a)*fraction for a, b in zip(start, end))
        issue = clearance(pose, lower)
        if escape.blocks(issue) or (issue and index == parts):
            return issue
    return None


def unwind(clearance: Clearance, start: Angles, lower: Angles) -> tuple[Keyframe, ...]:
    """Reach Ready one joint at a time, taking any joint whose own path is clear."""
    pose = list(start)
    frames = []
    moved = True
    while moved:
        moved = False
        for index, angle in enumerate(pose):
            step = pose[:index] + [READY[index]] + pose[index + 1:]
            if abs(angle - READY[index]) > 0.01 and not path_issue(clearance, tuple(pose), tuple(step), lower):
                pose = step
                frames.append(Keyframe(tuple(pose), 0.35))
                moved = True
    return tuple(frames) if max(abs(a - b) for a, b in zip(pose, READY)) <= 0.01 else ()


def plan(request: PlanRequest, clearance: Clearance | None) -> tuple[Segment, ...]:
    """Try a direct route, then via Ready, then joint by joint, and check the full path."""
    ready = Keyframe(READY, 0.35)
    tail = request.frames + (Keyframe(request.end, 0.35),)

    def routes():
        yield tail
        yield (ready,) + tail
        if clearance is not None:
            yield unwind(clearance, request.commanded, request.feedback.lower_position) + (ready,) + tail

    reason = None
    for frames in routes():
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
    raise UnsafePath(f"No clear route for {request.name}. {reason}")
