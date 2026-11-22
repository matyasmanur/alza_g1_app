"""G1 29-DoF arm order and limits from the pinned MuJoCo model, in radians."""

from math import radians

JOINT_LIMITS = {
    "left_shoulder_pitch": (-3.0892, 2.6704),
    "left_shoulder_roll": (-1.5882, 2.2515),
    "left_shoulder_yaw": (-2.618, 2.618),
    "left_elbow": (-1.0472, 2.0944),
    "left_wrist_roll": (-1.97222, 1.97222),
    "left_wrist_pitch": (-1.61443, 1.61443),
    "left_wrist_yaw": (-1.61443, 1.61443),
    "right_shoulder_pitch": (-3.0892, 2.6704),
    "right_shoulder_roll": (-2.2515, 1.5882),
    "right_shoulder_yaw": (-2.618, 2.618),
    "right_elbow": (-1.0472, 2.0944),
    "right_wrist_roll": (-1.97222, 1.97222),
    "right_wrist_pitch": (-1.61443, 1.61443),
    "right_wrist_yaw": (-1.61443, 1.61443),
}

JOINT_NAMES = tuple(JOINT_LIMITS)
# Arms hanging beside the body, rolled out just enough to clear the hips.
PARKED = tuple(map(radians, (0, 12, 0, 90, 0, 0, 0, 0, -12, 0, 90, 0, 0, 0)))
TARGET_MARGIN = 0.05
MAX_SPEED = 0.45
MAX_ACCELERATION = 2.5
