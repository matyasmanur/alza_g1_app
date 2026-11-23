"""DDS settings for the isolated 29-DoF G1 simulation."""

from enum import IntEnum

DOMAIN_ID = 1
INTERFACE = "lo"
ARM_TOPIC = "rt/arm_sdk"
STATE_TOPIC = "rt/lowstate"
LOW_COMMAND_TOPIC = "rt/lowcmd"
SPORT_STATE_TOPIC = "rt/sportmodestate"
SIM_SERVICE = "g1_sim"
SIM_SET_MODE = 1001
SIM_API_VERSION = "1.0.0.0"
MODE_MACHINE = 5
ARM_START = 15
ARM_END = 29
ARM_WEIGHT = 29
LEFT_ELBOW = 18
COMMAND_PERIOD = 0.02
COMMAND_TIMEOUT = 0.25
ZERO_VELOCITY = (0.0, 0.0, 0.0)


class SimMode(IntEnum):
    ARMS = 1
    PREPARING = 2
    LOCOMOTION = 3
    SETTLING = 4
    SUPPORTING = 5
    FALLEN = 6

    @property
    def changing(self) -> bool:
        return self in (self.PREPARING, self.SETTLING, self.SUPPORTING)
