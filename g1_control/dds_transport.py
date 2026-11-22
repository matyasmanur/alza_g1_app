"""Unitree SDK telemetry and simulated arm commands."""

from math import isfinite
from time import monotonic

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from g1_control.joints import JOINT_NAMES
from g1_control.latency import LatencyTrace
from g1_control.protocol import (
    ARM_END, ARM_START, ARM_TOPIC, ARM_WEIGHT, DOMAIN_ID, INTERFACE, MODE_MACHINE, STATE_TOPIC,
)
from g1_control.transport import Angles, Feedback

ARM_GAINS = tuple((8.0, 1.0) if "wrist" in joint else (45.0, 4.0) for joint in JOINT_NAMES)


class DdsObservation:
    """Read G1 telemetry without creating a DDS command writer."""

    def __init__(self, domain_id: int, interface: str, *, mode_machine: int | None = None,
                 trace: LatencyTrace | None = None) -> None:
        ChannelFactoryInitialize(domain_id, interface)
        self._crc = CRC()
        self._latest: Feedback | None = None
        self._last_tick: int | None = None
        self._trace = trace
        self._mode_machine = mode_machine
        self._subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
        self._subscriber.Init(self._receive, 1)

    def _receive(self, message: LowState_) -> None:
        if message.tick == self._last_tick:
            return
        if self._mode_machine is not None and message.mode_machine != self._mode_machine:
            return
        if self._crc.Crc(message) != message.crc:
            return
        motors = message.motor_state[ARM_START:ARM_END]
        position = tuple(motor.q for motor in motors)
        velocity = tuple(motor.dq for motor in motors)
        lower_position = tuple(motor.q for motor in message.motor_state[:ARM_START])
        if all(map(isfinite, lower_position + position + velocity)):
            self._last_tick = message.tick
            self._latest = Feedback(monotonic(), position, velocity, lower_position)
            if self._trace is not None:
                self._trace.observe(self._latest)

    def read(self) -> Feedback | None:
        """Return the most recent valid, advancing G1 arm sample."""
        return self._latest

    def write(self, position: Angles, velocity: Angles, *, enabled: bool) -> None:
        raise OSError("Hardware observation is read-only")


class DdsTransport(DdsObservation):
    """Add a command writer on the simulator's isolated DDS interface."""

    def __init__(self, trace: LatencyTrace | None = None) -> None:
        super().__init__(DOMAIN_ID, INTERFACE, mode_machine=MODE_MACHINE, trace=trace)
        self._command = unitree_hg_msg_dds__LowCmd_()
        self._command.mode_machine = MODE_MACHINE
        for index, gains in enumerate(ARM_GAINS, start=ARM_START):
            motor = self._command.motor_cmd[index]
            motor.kp, motor.kd = gains
        self._publisher = ChannelPublisher(ARM_TOPIC, LowCmd_)
        self._publisher.Init()

    def write(self, position: Angles, velocity: Angles, *, enabled: bool) -> None:
        """Send one G1 arm setpoint or release arm authority on the simulator."""
        command = self._command
        command.motor_cmd[ARM_WEIGHT].q = float(enabled)
        for index, (angle, speed) in enumerate(zip(position, velocity), start=ARM_START):
            command.motor_cmd[index].q = angle
            command.motor_cmd[index].dq = speed
        command.crc = self._crc.Crc(command)
        if not self._publisher.Write(command):
            raise OSError("Unitree arm command write failed")
        if self._trace is not None:
            self._trace.written(position, enabled)
