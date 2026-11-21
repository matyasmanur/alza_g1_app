"""Verify official arm DDS commands and measured feedback in one Docker run."""

import argparse
import subprocess
import sys
from tempfile import TemporaryFile
from time import monotonic, sleep

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from g1_control.protocol import (
    ARM_END,
    ARM_START,
    ARM_TOPIC,
    ARM_WEIGHT,
    COMMAND_PERIOD,
    COMMAND_TIMEOUT,
    DOMAIN_ID,
    INTERFACE,
    LEFT_ELBOW,
    MODE_MACHINE,
    STATE_TOPIC,
)


def check_arm_motion() -> None:
    """Ramp one elbow, then let the simulator time out while feedback continues."""
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    crc = CRC()
    latest = None

    def receive_state(message: LowState_) -> None:
        nonlocal latest
        if message.crc == crc.Crc(message):
            latest = (monotonic(), message)

    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init(receive_state, 1)
    publisher = ChannelPublisher(ARM_TOPIC, LowCmd_)
    publisher.Init()

    deadline = monotonic() + 10
    while latest is None and monotonic() < deadline:
        sleep(COMMAND_PERIOD)

    def fresh_state() -> LowState_:
        sample = latest
        if sample is None or monotonic() - sample[0] >= COMMAND_TIMEOUT:
            raise RuntimeError("No fresh CRC-valid rt/lowstate feedback")
        return sample[1]

    initial = fresh_state()
    if initial.mode_machine != MODE_MACHINE:
        raise RuntimeError(f"Expected G1 mode {MODE_MACHINE}, got {initial.mode_machine}")

    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_machine = initial.mode_machine
    command.motor_cmd[ARM_WEIGHT].q = 1.0
    for index in range(ARM_START, ARM_END):
        motor = command.motor_cmd[index]
        motor.q = initial.motor_state[index].q
        motor.kp = 20.0
        motor.kd = 2.0

    baseline = initial.motor_state[LEFT_ELBOW].q
    elbow = command.motor_cmd[LEFT_ELBOW]
    elbow.kp = 45.0
    elbow.kd = 4.0
    increment = 0.25
    started = monotonic()
    while monotonic() - started < 1.8:
        fraction = min(1.0, monotonic() - started)
        elbow.q = baseline + increment * fraction**2 * (3 - 2 * fraction)
        command.crc = crc.Crc(command)
        publisher.Write(command)
        sleep(COMMAND_PERIOD)

    reached = fresh_state()
    change = reached.motor_state[LEFT_ELBOW].q - baseline
    if abs(change - increment) > 0.10:
        raise RuntimeError(f"Expected positive elbow motion of about {increment:.2f} rad, got {change:.3f} rad")

    # Leave arm authority enabled and stop publishing to exercise the watchdog.
    sleep(COMMAND_TIMEOUT + 0.6)
    held = fresh_state()
    if held.tick <= reached.tick or abs(held.motor_state[LEFT_ELBOW].dq) > 0.10:
        raise RuntimeError("Expected advancing feedback and a settled elbow after command timeout")
    print(f"Measured left elbow movement: {change:+.3f} rad for a {increment:+.2f} rad target")


def check_controller_motion() -> None:
    """Verify controller-owned motion and Stop via the DDS transport."""
    from g1_control.controller import Motion, MotionController
    from g1_control.dds_transport import DdsTransport
    from g1_control.joints import JOINT_NAMES

    controller = MotionController(DdsTransport())
    operator = "smoke-operator"
    elbow = JOINT_NAMES.index("left_elbow")
    deadline = monotonic() + 10
    while controller.status.connection != "connected" and monotonic() < deadline:
        controller.tick()
        sleep(COMMAND_PERIOD)
    if controller.status.connection != "connected":
        raise RuntimeError("No fresh G1 feedback for the controller")

    baseline = controller.status.measured.position[elbow]
    controller.take_control(operator)
    controller.enable(operator)
    controller.set_joint(operator, "left_elbow", baseline + 0.80)
    deadline = monotonic() + 0.9
    while monotonic() < deadline:
        controller.heartbeat(operator)
        controller.tick()
        sleep(COMMAND_PERIOD)
    moving = controller.status.measured.position[elbow] - baseline
    if controller.status.motion is not Motion.MANUAL or not 0.12 < moving < 0.65:
        raise RuntimeError(f"Expected elbow in motion before Stop, measured {moving:+.3f} rad")

    controller.stop(operator)
    deadline = monotonic() + 0.6
    while monotonic() < deadline:
        controller.heartbeat(operator)
        controller.tick()
        sleep(COMMAND_PERIOD)
    stopped = controller.status
    change = stopped.measured.position[elbow] - baseline
    if (stopped.connection != "connected" or stopped.motion is not Motion.IDLE
            or not stopped.stop_latched or stopped.enabled or stopped.fault
            or not 0.12 < change < 0.70 or abs(stopped.measured.velocity[elbow]) > 0.10):
        raise RuntimeError(f"Expected a settled, latched Stop, measured {change:+.3f} rad, {stopped}")
    print(f"Elbow displacement at Stop request: {moving:+.3f} rad")
    print(f"Elbow displacement after Stop: {change:+.3f} rad for a +0.80 rad target")


def main() -> None:
    """Run an SDK smoke check alongside a separate simulator process."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", action="store_true", help="Exercise the controller and DDS transport")
    use_controller = parser.parse_args().controller
    with TemporaryFile(mode="w+") as log:
        simulator = subprocess.Popen(
            [sys.executable, "-m", "g1_control.simulator"],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            if use_controller:
                check_controller_motion()
            else:
                check_arm_motion()
        finally:
            simulator.terminate()
            simulator.wait(timeout=5)
            log.seek(0)
            output = log.read()
            print(output, end="")
        if use_controller:
            if "Arm authority released" not in output or "Arm command timeout" in output:
                raise RuntimeError("Expected explicit controller handoff to the simulator hold")
            print("PASS: controller motion, measured feedback, and Stop handoff")
        else:
            if "Arm command timeout" not in output:
                raise RuntimeError("The simulator did not enter its timeout hold")
            print("PASS: rt/arm_sdk motion, rt/lowstate feedback, and simulator command timeout")


if __name__ == "__main__":
    main()
