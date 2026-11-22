"""Single-owner motion policy. Call its methods from one backend event loop."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import copysign, isfinite, sqrt
from time import monotonic
from typing import Callable

from g1_control.joints import JOINT_LIMITS, JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED, PARKED, TARGET_MARGIN
from g1_control.protocol import COMMAND_PERIOD, COMMAND_TIMEOUT
from g1_control.trajectory import (
    MARGIN_METRES, READY, Clearance, Escape, Keyframe, PlanRequest, Segment, UnsafePath, path_issue, plan,
)
from g1_control.transport import Angles, ArmTransport, Feedback

OPERATOR_TIMEOUT = 1.0
VELOCITY_TOLERANCE = 0.10
ZERO = (0.0,) * len(JOINT_NAMES)


class Motion(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    PLANNING = "planning"
    GESTURE = "gesture"
    STOPPING = "stopping"


class Rejected(ValueError):
    """A command the operator may correct and retry."""


@dataclass(frozen=True)
class Status:
    """The latest measured state stays visible after a disconnect."""

    connection: str
    owner: str | None
    enabled: bool
    stop_latched: bool
    motion: Motion
    measured: Feedback | None
    target: Angles | None
    fault: str | None
    gesture: str | None


def checked_target(joint: str, angle: float) -> float:
    """Reject non-finite targets and keep clearance from the mechanical limit."""
    if joint not in JOINT_LIMITS:
        raise Rejected(f"Unknown arm joint: {joint}")
    minimum, maximum = JOINT_LIMITS[joint]
    if type(angle) not in (int, float) or not minimum + TARGET_MARGIN <= angle <= maximum - TARGET_MARGIN:
        raise Rejected(f"Target outside allowed range for {joint}")
    return float(angle)


class MotionController:
    """Own the motion state and write at most one setpoint per tick."""

    def __init__(self, transport: ArmTransport, clock: Callable[[], float] = monotonic, *,
                 clearance: Clearance | None = None) -> None:
        self._transport = transport
        self._clock = clock
        self._last_tick = clock()
        self._feedback: Feedback | None = None
        self._owner: str | None = None
        self._owner_deadline = 0.0
        self._enabled = False
        self._stop_latched = False
        self._motion = Motion.IDLE
        self._target: Angles | None = None
        self._position = ZERO
        self._velocity = ZERO
        self._clearance = clearance
        self._escapes = self._monitors()
        self._plan: PlanRequest | None = None
        self._segments: deque[Segment] = deque()
        self._segment_elapsed = 0.0
        self._gesture: str | None = None
        self._release = False
        self._fault: str | None = None

    @property
    def status(self) -> Status:
        """Return a snapshot without reading from the transport."""
        connection = "waiting"
        if self._feedback is not None:
            connection = "connected" if self._fresh(self._clock()) else "stale"
        return Status(
            connection, self._owner, self._enabled, self._stop_latched,
            self._motion, self._feedback, self._target, self._fault, self._gesture,
        )

    def take_control(self, session: str) -> None:
        """Reserve control for one session. Enabling motion is a separate action."""
        if not session or self._owner is not None or self._motion is Motion.STOPPING:
            raise Rejected("Control is unavailable")
        if not self._fresh(self._clock()):
            raise Rejected("Robot feedback is unavailable")
        self._owner = session
        self._owner_deadline = self._clock() + OPERATOR_TIMEOUT

    def heartbeat(self, session: str) -> None:
        """Renew the controlling session's lease."""
        self._require_owner(session)
        self._owner_deadline = self._clock() + OPERATOR_TIMEOUT

    def disconnect(self, session: str) -> None:
        """Revoke a departing operator and cancel their motion."""
        if self._owner == session:
            self._owner = None
            self._cancel_motion()

    def release(self, session: str) -> PlanRequest | None:
        """Park the arms before giving up control, or release at once when motion is disabled."""
        self._require_owner(session)
        if not self._enabled:
            self.disconnect(session)
            return None
        return self.prepare_gesture(session, "Parking arms", (), PARKED, release=True)

    def enable(self, session: str) -> None:
        """Seed a new motion session from a fresh, stationary measured pose."""
        self._require_owner(session)
        if self._enabled or self._stop_latched or self._motion is Motion.STOPPING:
            raise Rejected("Motion cannot be enabled in the current state")
        if not self._fresh(self._clock()):
            raise Rejected("Robot feedback is unavailable")
        feedback = self._feedback
        if any(abs(v) > VELOCITY_TOLERANCE for v in feedback.velocity):
            raise Rejected("Wait until the arms settle before enabling")
        if any(not low <= q <= high for q, (low, high) in zip(feedback.position, JOINT_LIMITS.values())):
            raise Rejected("Measured pose is outside the joint limits")
        self._escapes = self._monitors()
        self._position = self._target = feedback.position
        self._velocity = ZERO
        self._fault = None
        self._enabled = True

    def set_joint(self, session: str, joint: str, angle: float) -> None:
        """Replace the latest manual target for one joint."""
        self._require_enabled(session)
        if self._motion in (Motion.PLANNING, Motion.GESTURE):
            raise Rejected("A gesture is already in progress")
        target = list(self._target)
        value = checked_target(joint, angle)
        target[JOINT_NAMES.index(joint)] = value
        goal = tuple(target)
        if self._clearance is not None:
            for start in (self._feedback.position, self._position):
                if issue := path_issue(self._clearance, start, goal, self._feedback.lower_position):
                    raise Rejected(issue)
        self._target = goal
        self._motion = Motion.MANUAL

    def start_gesture(self, session: str, waypoints: tuple[Angles, ...]) -> None:
        """Plan and start a sequence synchronously for non-web callers."""
        request = self.prepare_gesture(session, "Gesture", tuple(Keyframe(pose, 0.35) for pose in waypoints))
        try:
            segments = self.plan_gesture(request)
        except UnsafePath as error:
            self.finish_plan(session, request, ())
            raise Rejected(str(error)) from error
        self.finish_plan(session, request, segments)

    def capture_pose(self, session: str) -> Angles:
        """Capture the measured arms once they are stationary."""
        self._require_enabled(session)
        if self._motion is not Motion.IDLE or any(abs(v) > VELOCITY_TOLERANCE for v in self._feedback.velocity):
            raise Rejected("Wait for the arms to settle before capturing a pose")
        return tuple(checked_target(joint, angle) for joint, angle in zip(JOINT_NAMES, self._feedback.position))

    def prepare_gesture(self, session: str, name: str, frames: tuple[Keyframe, ...],
                        end: Angles = READY, release: bool = False) -> PlanRequest:
        """Reserve motion and capture a stationary pose before planning a route that finishes at end."""
        self._require_enabled(session)
        if self._motion is not Motion.IDLE:
            raise Rejected("Wait for the current motion to settle")
        if any(len(frame.pose) != len(JOINT_NAMES)
               or not isfinite(frame.seconds) or frame.seconds <= 0 for frame in frames):
            raise Rejected("A gesture needs timed, complete arm poses")
        if any(abs(v) > VELOCITY_TOLERANCE for v in self._feedback.velocity):
            raise Rejected("Wait for the current motion to settle")
        checked = tuple(
            Keyframe(tuple(checked_target(joint, q) for joint, q in zip(JOINT_NAMES, frame.pose)), frame.seconds)
            for frame in frames
        )
        self._plan = PlanRequest(name, self._feedback, self._position, checked, end, release)
        self._gesture = name
        self._motion = Motion.PLANNING
        return self._plan

    def plan_gesture(self, request: PlanRequest) -> tuple[Segment, ...]:
        """Plan against an immutable snapshot. Safe to run off the control loop."""
        return plan(request, self._clearance)

    def finish_plan(self, session: str, request: PlanRequest, segments: tuple[Segment, ...]) -> None:
        """Accept a plan only if the same operator and pose are still current."""
        if self._plan is not request:
            raise Rejected("Gesture planning was cancelled")
        self._plan = None
        self._motion = Motion.IDLE
        self._gesture = None
        self._require_enabled(session)
        if not segments:
            if request.release:
                self.disconnect(session)
            return
        before = request.feedback.lower_position + request.feedback.position
        now = self._feedback.lower_position + self._feedback.position
        if max(abs(a - b) for a, b in zip(before, now)) > 0.035:
            raise Rejected("The measured pose changed during planning. Try again after it settles.")
        self._segments = deque(segments)
        self._segment_elapsed = 0.0
        self._target = segments[0].end
        self._gesture = request.name
        self._release = request.release
        self._motion = Motion.GESTURE

    def stop(self, session: str) -> None:
        """Cancel any trajectory and latch Stop before controlled deceleration."""
        self._require_owner(session)
        self._stop_latched = True
        self._cancel_motion()

    def reset_stop(self, session: str) -> None:
        """Clear the Stop latch after deceleration, leaving motion disabled."""
        self._require_owner(session)
        if not self._stop_latched or self._motion is Motion.STOPPING or not self._fresh(self._clock()):
            raise Rejected("Stop can only be reset with fresh feedback after deceleration")
        self._stop_latched = False

    def tick(self) -> None:
        """Refresh telemetry, enforce ownership, and advance output by one interval."""
        sample = self._transport.read()
        now = self._clock()
        interval = now - self._last_tick
        self._last_tick = now
        feedback_interrupted = False
        if sample is not None:
            if self._feedback is not None:
                feedback_interrupted = sample.received_at - self._feedback.received_at >= COMMAND_TIMEOUT
            self._feedback = sample

        if feedback_interrupted or not self._fresh(now):
            if self._owner is not None or self._motion is Motion.STOPPING:
                self._disarm_fault("Robot feedback is stale")
            return
        if self._owner is not None and now >= self._owner_deadline:
            self.disconnect(self._owner)
        if interval <= 0 or not (self._enabled or self._motion is Motion.STOPPING):
            return
        if interval >= COMMAND_TIMEOUT:
            # The simulator may have entered its own hold while the loop was stalled.
            self._disarm_fault("Control loop missed the command timeout")
            return

        elapsed = min(COMMAND_PERIOD, interval)
        if self._segments:
            self._play(elapsed)
        else:
            self._advance(elapsed)
        if any(not low <= q <= high for q, (low, high) in zip(self._position, JOINT_LIMITS.values())):
            self._disarm_fault("Generated setpoint exceeded the joint limits")
            return
        if self._clearance is not None and self._motion in (Motion.MANUAL, Motion.GESTURE, Motion.STOPPING):
            for pose, escape in zip((self._feedback.position, self._position), self._escapes):
                issue = self._clearance(pose, self._feedback.lower_position)
                if escape.blocks(issue):
                    try:
                        self._transport.write(self._feedback.position, ZERO, enabled=False)
                    except OSError:
                        pass  # The simulator watchdog remains the fallback.
                    self._stop_latched = True
                    self._disarm_fault(issue)
                    return
        releasing = self._motion is Motion.STOPPING and self._velocity == ZERO
        try:
            self._transport.write(self._position, self._velocity, enabled=not releasing)
        except OSError as error:
            self._disarm_fault(str(error))
            return
        if releasing:
            self._motion = Motion.IDLE
        elif self._motion in (Motion.MANUAL, Motion.GESTURE) and not self._segments and self._settled():
            self._motion = Motion.IDLE
            self._gesture = None
            if self._release:
                self.disconnect(self._owner)

    @staticmethod
    def _monitors() -> tuple[Escape, Escape]:
        """Stop on measured contact, and on a commanded pose that drifts back inside the margin.

        The margin absorbs tracking error, so the measured pose may use all of it.
        """
        return Escape(MARGIN_METRES + 0.002), Escape(0.0015)

    def _fresh(self, now: float) -> bool:
        return self._feedback is not None and now - self._feedback.received_at < COMMAND_TIMEOUT

    def _require_owner(self, session: str) -> None:
        if self._owner != session or self._clock() >= self._owner_deadline:
            raise Rejected("This session does not own control")

    def _require_enabled(self, session: str) -> None:
        self._require_owner(session)
        if not self._enabled or not self._fresh(self._clock()):
            raise Rejected("Motion is disabled or robot feedback is stale")

    def _cancel_motion(self) -> None:
        if self._enabled:
            self._motion = Motion.STOPPING
        self._enabled = False
        self._target = None
        self._plan = None
        self._segments.clear()
        self._gesture = None
        self._release = False

    def _disarm_fault(self, reason: str) -> None:
        self._cancel_motion()
        self._owner = None
        self._motion = Motion.IDLE
        self._fault = reason

    def _advance(self, elapsed: float) -> None:
        """Slew-limited velocities produce continuous position setpoints."""
        goals = self._target if self._target is not None else self._position
        speed_change = MAX_ACCELERATION * elapsed
        positions = []
        velocities = []
        for position, speed, target in zip(self._position, self._velocity, goals):
            error = target - position
            braking_speed = sqrt(2 * MAX_ACCELERATION * abs(error))
            desired = copysign(min(MAX_SPEED, braking_speed, abs(error) / elapsed), error)
            speed += max(-speed_change, min(speed_change, desired - speed))
            positions.append(position + speed * elapsed)
            velocities.append(speed)
        self._position = tuple(positions)
        self._velocity = tuple(velocities)

    def _play(self, elapsed: float) -> None:
        segment = self._segments[0]
        self._target = segment.end
        self._segment_elapsed += elapsed
        self._position, self._velocity = segment.sample(self._segment_elapsed)
        if self._segment_elapsed >= segment.seconds:
            self._segments.popleft()
            self._segment_elapsed = 0.0

    def _settled(self) -> bool:
        """The command has arrived and the arm has stopped. Gravity may hold it short of the target."""
        return (
            max(abs(a - b) for a, b in zip(self._position, self._target)) < 0.001
            and max(map(abs, self._velocity)) < 0.01
            and max(map(abs, self._feedback.velocity)) < VELOCITY_TOLERANCE
        )
