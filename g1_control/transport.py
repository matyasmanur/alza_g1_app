"""App-side arm feedback and output contract shared by SDK and mock."""

from dataclasses import dataclass
from math import hypot
from typing import Protocol

from g1_control.protocol import SimMode

Angles = tuple[float, ...]
Velocity = tuple[float, float, float]


@dataclass(frozen=True)
class Feedback:
    """Measured arm angles and velocities, plus leg and waist angles in DDS order."""

    received_at: float
    position: Angles
    velocity: Angles
    lower_position: Angles


@dataclass(frozen=True)
class SimFeedback:
    received_at: float
    mode: SimMode
    position: tuple[float, ...]
    quaternion: tuple[float, ...]
    velocity: Velocity
    yaw_rate: float

    @property
    def stationary(self) -> bool:
        return hypot(*self.velocity[:2]) < 0.08 and abs(self.yaw_rate) < 0.18


class LocomotionTransport(Protocol):
    def read(self) -> SimFeedback | None: ...

    def set_mode(self, mode: str) -> None: ...

    def set_velocity(self, velocity: Velocity) -> None: ...

    def take_error(self) -> tuple[str, str] | None: ...


class ArmTransport(Protocol):
    """Publish setpoints and expose the most recent measured sample."""

    def read(self) -> Feedback | None:
        """Return the latest measurement with its monotonic timestamp."""
        ...

    def write(self, position: Angles, velocity: Angles, *, enabled: bool) -> None:
        """Request arm targets or release authority. Raise OSError on failure."""
        ...
