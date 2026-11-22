"""Built-in and recorded G1 arm gestures."""

import json
import logging
import os
import re
from dataclasses import dataclass
from math import degrees, radians
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4

from g1_control.controller import checked_target
from g1_control.joints import JOINT_NAMES, PARKED
from g1_control.trajectory import Keyframe

REST = (0, 0, 0, 0, 0, 0, 0)
RECORDINGS_DIR = Path(os.environ.get("G1_GESTURE_DIR", Path(__file__).resolve().parents[1] / "recorded_gestures"))
MAX_RECORDED_POSES = 100
logger = logging.getLogger(__name__)


def pose(left=REST, *, right=REST, seconds=0.35) -> Keyframe:
    """Author both sides in left-arm degrees and reflect the right arm's axes."""
    pitch, roll, yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw = right
    mirrored = (pitch, -roll, -yaw, elbow, -wrist_roll, wrist_pitch, -wrist_yaw)
    return Keyframe(tuple(map(radians, left + mirrored)), seconds)


@dataclass(frozen=True)
class Gesture:
    """A display name and arm keyframes."""

    name: str
    frames: tuple[Keyframe, ...]


# Stroke endpoints fitted to the G1 hand meshes, with the backs angled forward.
# From the audience's view the robot's right hand starts on the left.
ALZA_PARK = tuple(map(degrees, PARKED[:7]))
ALZA_A_FOOT = (-9.1, 9.7, -14.9, 55.9, 31.6, -48.0, 75.3)
ALZA_A = (
    # Raise the fingertips to the roof, then trace both sides down and out.
    pose((-31.4, 9.7, -14.9, 10.3, 50.9, -41.2, 55.7),
         right=(-31.7, 9.7, -15.2, 10.6, 51.1, -41.3, 56.8)),
    pose(ALZA_A_FOOT, right=(-9.4, 9.7, -15.6, 56.0, 31.3, -47.9, 76.4)),
    # Hold the left hand at its foot while the right hand draws the bar.
    pose(ALZA_A_FOOT, right=(-33.2, 49.4, -33.0, 33.8, 81.0, -51.1, 78.3)),
    pose(ALZA_A_FOOT, right=(-57.4, 9.7, -47.8, 47.0, 84.7, -40.4, 56.0)),
)
ALZA_L = (
    # The left fingertips turn from the vertical stroke into the base.
    pose((-105.0, 44.8, -51.3, 23.2, 87.3, -87.7, 31.9), right=ALZA_PARK),
    pose((-83.8, 13.2, -86.1, 27.5, 107.9, -75.0, 19.1), right=ALZA_PARK),
    pose((-83.7, 25.3, -125.8, 16.9, 108.2, -76.0, 1.4), right=ALZA_PARK),
)
ALZA_Z = (
    # The right hand traces the top, the descending diagonal, and the base.
    pose(ALZA_PARK, right=(-36.6, 41.8, 39.1, 7.4, 108.2, -87.7, 51.6)),
    pose(ALZA_PARK, right=(-43.5, 9.7, -26.5, 17.9, 74.9, -29.5, 59.2)),
    pose(ALZA_PARK, right=(-9.8, 22.6, -39.9, 47.8, 36.8, -8.8, 87.7)),
    pose(ALZA_PARK, right=(-31.7, 9.7, -64.7, 55.5, 87.0, 12.7, 66.3)),
)

GESTURES = {
    "alza": Gesture("ALZA", ALZA_A + ALZA_L + ALZA_Z + ALZA_A),
}


def _recorded_gesture(data: object) -> Gesture:
    if not isinstance(data, dict):
        raise ValueError("A gesture recording must be a JSON object")
    name, poses = data.get("name"), data.get("poses")
    if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80 or not name.strip().isprintable():
        raise ValueError("Enter a gesture name of 1 to 80 printable characters")
    if not isinstance(poses, list) or not 1 <= len(poses) <= MAX_RECORDED_POSES:
        raise ValueError(f"Record between 1 and {MAX_RECORDED_POSES} poses")
    frames = []
    for angles in poses:
        if not isinstance(angles, (list, tuple)) or len(angles) != len(JOINT_NAMES):
            raise ValueError(f"Each pose needs {len(JOINT_NAMES)} joint angles in radians")
        frames.append(Keyframe(tuple(checked_target(joint, angle) for joint, angle in zip(JOINT_NAMES, angles)), 0.35))
    return Gesture(name.strip(), tuple(frames))


def load_gestures() -> dict[str, Gesture]:
    """Load recordings first, followed by the built-in gestures."""
    gestures = {}
    for path in sorted(RECORDINGS_DIR.glob("*.json")):
        try:
            gestures[f"recorded-{path.stem}"] = _recorded_gesture(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError, RecursionError) as error:
            logger.warning("Skipping gesture recording %s: %s", path.name, error)
    return gestures | GESTURES


def save_recording(name: object, poses: list[tuple[float, ...]]) -> tuple[str, Gesture]:
    """Publish one complete JSON file without replacing an existing recording."""
    gesture = _recorded_gesture({"name": name, "poses": poses})
    slug = re.sub(r"[^a-z0-9]+", "-", gesture.name.lower()).strip("-") or "gesture"
    filename = f"{slug}-{uuid4().hex}.json"
    RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=RECORDINGS_DIR) as directory:
        pending = Path(directory) / filename
        data = {"name": gesture.name, "poses": [frame.pose for frame in gesture.frames]}
        pending.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        pending.replace(RECORDINGS_DIR / filename)
    return f"recorded-{Path(filename).stem}", gesture
