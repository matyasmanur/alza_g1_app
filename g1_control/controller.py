"""Single-owner motion policy. Call its methods from one backend event loop."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import copysign, sqrt
from time import monotonic
from typing import Callable

from g1_control.joints import JOINT_LIMITS, JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED, TARGET_MARGIN
from g1_control.protocol import COMMAND_PERIOD, COMMAND_TIMEOUT
from g1_control.transport import Angles, ArmTransport, Feedback

OPERATOR_TIMEOUT = 1.0
POSITION_TOLERANCE = 0.08
VELOCITY_TOLERANCE = 0.10
ZERO = (0.0,) * len(JOINT_NAMES)


class Motion(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
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

    def __init__(self, transport: ArmTransport, clock: Callable[[], float] = monotonic) -> None:
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
        self._waypoints: deque[Angles] = deque()
        self._fault: str | None = None

    @property
    def status(self) -> Status:
        """Return a snapshot without reading from the transport."""
        connection = "waiting"
        if self._feedback is not None:
            connection = "connected" if self._fresh(self._clock()) else "stale"
        return Status(
            connection, self._owner, self._enabled, self._stop_latched,
            self._motion, self._feedback, self._target, self._fault,
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
        self._position = self._target = feedback.position
        self._velocity = ZERO
        self._fault = None
        self._enabled = True

    def set_joint(self, session: str, joint: str, angle: float) -> None:
        """Replace the latest manual target for one joint."""
        self._require_enabled(session)
        if self._motion is Motion.GESTURE:
            raise Rejected("A gesture is already running")
        target = list(self._target)
        value = checked_target(joint, angle)
        target[JOINT_NAMES.index(joint)] = value
        self._target = tuple(target)
        self._motion = Motion.MANUAL

    def start_gesture(self, session: str, waypoints: tuple[Angles, ...]) -> None:
        """Reserve motion for a sequence of complete arm poses."""
        self._require_enabled(session)
        if self._motion is not Motion.IDLE:
            raise Rejected("Wait for the current motion to settle")
        if not waypoints or any(len(pose) != len(JOINT_NAMES) for pose in waypoints):
            raise Rejected("A gesture needs one or more complete arm poses")
        checked = deque(
            tuple(checked_target(joint, q) for joint, q in zip(JOINT_NAMES, pose))
            for pose in waypoints
        )
        self._target = checked.popleft()
        self._waypoints = checked
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

        self._advance(min(COMMAND_PERIOD, interval))
        if any(not low <= q <= high for q, (low, high) in zip(self._position, JOINT_LIMITS.values())):
            self._disarm_fault("Generated setpoint exceeded the joint limits")
            return
        releasing = self._motion is Motion.STOPPING and self._velocity == ZERO
        try:
            self._transport.write(self._position, self._velocity, enabled=not releasing)
        except OSError as error:
            self._disarm_fault(str(error))
            return
        if releasing:
            self._motion = Motion.IDLE
        elif self._target is not None and self._settled():
            if self._waypoints:
                self._target = self._waypoints.popleft()
            else:
                self._motion = Motion.IDLE

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
        self._waypoints.clear()

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

    def _settled(self) -> bool:
        feedback = self._feedback
        return (
            max(abs(a - b) for a, b in zip(self._position, self._target)) < 0.001
            and max(map(abs, self._velocity)) < 0.01
            and max(abs(a - b) for a, b in zip(feedback.position, self._target)) < POSITION_TOLERANCE
            and max(map(abs, feedback.velocity)) < VELOCITY_TOLERANCE
        )
