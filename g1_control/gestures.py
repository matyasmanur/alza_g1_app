"""App-authored G1 gesture renditions and two original arm sequences."""

from dataclasses import dataclass
from math import radians

from g1_control.trajectory import Keyframe

REST = (0, 0, 0, 0, 0, 0, 0)


def pose(left=REST, *, right=REST, seconds=0.35) -> Keyframe:
    """Author both sides in left-arm degrees and reflect the right arm's axes."""
    pitch, roll, yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw = right
    mirrored = (pitch, -roll, -yaw, elbow, -wrist_roll, wrist_pitch, -wrist_yaw)
    return Keyframe(tuple(map(radians, left + mirrored)), seconds)


@dataclass(frozen=True)
class Gesture:
    """A display name, SDK reference ID when inspired by Unitree, and keyframes."""

    name: str
    sdk_id: int | None
    frames: tuple[Keyframe, ...]


FACE_WAVE = (-32, 30, 0, -24, 0, 0, 0)
HIGH_WAVE = (-72, 31, -5, -20, 0, 0, 0)
HIGH_FIVE = (-48, 27, -2, -16, 0, 10, 0)
SHAKE = (-22, 16, -5, 12, 9, 0, 0)
KISS_IN = (-47, 26, -8, -19, 0, -14, 0)
KISS_OUT = (-39, 29, -8, 3, 12, 3, 0)
OPEN = (-42, 34, -10, 4, 0, 0, 0)
CLAP = (-51, 24, -38, 31, 0, 0, 0)
HEART = (-45, 34, -29, 0, 0, -12, 0)
SALUTE = (-50, 25, -2, -24, 8, 0, 0)

GESTURES = {
    "wave": Gesture("Face wave", 25, (
        pose(FACE_WAVE), pose((-32, 30, 0, -24, -23, 0, 0)), pose((-32, 30, 0, -24, 24, 0, 0)),
        pose((-32, 30, 0, -24, -20, 0, 0)), pose(FACE_WAVE),
    )),
    "high-five": Gesture("High five", 18, (
        pose(right=HIGH_FIVE), pose(right=HIGH_FIVE, seconds=0.8),
    )),
    "handshake": Gesture("Handshake", 27, (
        pose(right=SHAKE), pose(right=(-22, 16, -5, 3, 9, 0, 0)),
        pose(right=(-22, 16, -5, 19, 9, 0, 0)), pose(right=SHAKE),
    )),
    "two-hand-kiss": Gesture("Two-hand kiss", 11, (
        pose(KISS_IN, right=KISS_IN), pose(KISS_OUT, right=KISS_OUT),
    )),
    "left-kiss": Gesture("Left kiss", 12, (pose(KISS_IN), pose(KISS_OUT))),
    "right-kiss": Gesture("Right kiss", 13, (pose(right=KISS_IN), pose(right=KISS_OUT))),
    "hands-up": Gesture("Hands up", 15, (
        pose(HIGH_WAVE, right=HIGH_WAVE), pose(HIGH_WAVE, right=HIGH_WAVE, seconds=0.8),
    )),
    "clap": Gesture("Clap", 17, (
        pose(OPEN, right=OPEN), pose(CLAP, right=CLAP), pose(OPEN, right=OPEN), pose(CLAP, right=CLAP),
    )),
    "hug": Gesture("Hug", 19, (
        pose((-45, 48, 4, 12, 0, 0, 0), right=(-45, 48, 4, 12, 0, 0, 0)),
        pose((-42, 26, -24, 24, 0, 0, 0), right=(-42, 26, -24, 24, 0, 0, 0), seconds=0.85),
    )),
    "heart": Gesture("Heart", 20, (pose(HEART, right=HEART), pose(HEART, right=HEART, seconds=0.8))),
    "right-heart": Gesture("Right heart", 21, (pose(right=HEART), pose(right=HEART, seconds=0.8))),
    "reject": Gesture("Reject", 22, (
        pose((-36, 33, -7, -4, -8, 16, 0)), pose((-28, 42, 5, -4, 16, 16, 0)),
    )),
    "right-hand-up": Gesture("Right hand up", 23, (
        pose(right=SALUTE), pose(right=SALUTE, seconds=0.8),
    )),
    "x-ray": Gesture("X-ray", 24, (
        pose((-28, 23, -38, 12, 0, 0, 0), right=(-47, 32, -28, -3, 0, 0, 0)),
        pose((-28, 23, -38, 12, 0, 0, 0), right=(-47, 32, -28, -3, 0, 0, 0), seconds=0.7),
    )),
    "high-wave": Gesture("High wave", 26, (
        pose(HIGH_WAVE), pose((-72, 31, -5, -20, -24, 0, 0)),
        pose((-72, 31, -5, -20, 23, 0, 0)), pose(HIGH_WAVE),
    )),
    "conductor": Gesture("Conductor", None, (
        pose(OPEN, right=SHAKE), pose((-34, 39, -7, -5, 0, 0, 0), right=OPEN),
        pose(OPEN, right=(-36, 39, -7, -5, 0, 0, 0)), pose(SHAKE, right=OPEN),
    )),
    "alternating-salute": Gesture("Alternating salute", None, (
        pose(SALUTE), pose(SALUTE, seconds=0.5), pose(), pose(right=SALUTE), pose(right=SALUTE, seconds=0.5),
    )),
}
