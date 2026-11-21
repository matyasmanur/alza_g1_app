"""SDK-free arm transport with measured motion and controllable feedback loss."""

from time import monotonic
from typing import Callable

from g1_control.joints import JOINT_NAMES
from g1_control.protocol import ARM_START, COMMAND_TIMEOUT
from g1_control.transport import Angles, Feedback


class MockTransport:
    """Follow bounded targets with simple lag, using the caller's clock."""

    def __init__(self, clock: Callable[[], float] = monotonic) -> None:
        self.clock = clock
        self.feedback_enabled = True
        self.authority = False
        self.position = self.target = (0.0,) * len(JOINT_NAMES)
        self.updated_at = clock()
        self.last_write = self.updated_at

    def read(self) -> Feedback | None:
        """Advance the mock plant and optionally expose its measurements."""
        now = self.clock()
        elapsed = now - self.updated_at
        if self.authority and now - self.last_write >= COMMAND_TIMEOUT:
            elapsed = max(0.0, self.last_write + COMMAND_TIMEOUT - self.updated_at)
        displacement = tuple(
            (goal - angle) * min(1.0, elapsed / 0.08)
            for angle, goal in zip(self.position, self.target)
        )
        self.position = tuple(q + delta for q, delta in zip(self.position, displacement))
        self.updated_at = now
        if self.authority and now - self.last_write >= COMMAND_TIMEOUT:
            self.authority = False
            self.target = self.position
        if not self.feedback_enabled:
            return None
        velocity = tuple(delta / elapsed if elapsed else 0.0 for delta in displacement)
        return Feedback(now, self.position, velocity, lower_position=(0.0,) * ARM_START)

    def write(self, position: Angles, velocity: Angles, *, enabled: bool) -> None:
        """Apply the next setpoint or capture a local hold on release."""
        self.authority = enabled
        self.target = position if enabled else self.position
        self.last_write = self.clock()
