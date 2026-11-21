"""App-side arm feedback and output contract shared by SDK and mock."""

from dataclasses import dataclass
from typing import Protocol

Angles = tuple[float, ...]


@dataclass(frozen=True)
class Feedback:
    """Measured arm angles and velocities, plus leg and waist angles in DDS order."""

    received_at: float
    position: Angles
    velocity: Angles
    lower_position: Angles


class ArmTransport(Protocol):
    """Publish setpoints and expose the most recent measured sample."""

    def read(self) -> Feedback | None:
        """Return the latest measurement with its monotonic timestamp."""
        ...

    def write(self, position: Angles, velocity: Angles, *, enabled: bool) -> None:
        """Request arm targets or release authority. Raise OSError on failure."""
        ...
