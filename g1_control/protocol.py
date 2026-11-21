"""DDS settings for the isolated 29-DoF G1 simulation."""

DOMAIN_ID = 1
INTERFACE = "lo"
ARM_TOPIC = "rt/arm_sdk"
STATE_TOPIC = "rt/lowstate"
MODE_MACHINE = 5
ARM_START = 15
ARM_END = 29
ARM_WEIGHT = 29
LEFT_ELBOW = 18
COMMAND_PERIOD = 0.02
COMMAND_TIMEOUT = 0.25
