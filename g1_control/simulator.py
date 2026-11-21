"""Run supported-base G1 physics with a minimal Unitree arm DDS endpoint.

The physics loop owns MuJoCo data and has a local hold target. The DDS
callback only replaces the latest valid command. Arm weight blends user
torque with the local hold, which approximates ownership for simulation.
"""

import argparse
from pathlib import Path
from threading import Lock
from time import monotonic, sleep
from xml.etree import ElementTree

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from g1_control.joints import JOINT_LIMITS, JOINT_NAMES
from g1_control.protocol import (
    ARM_END,
    ARM_START,
    ARM_TOPIC,
    ARM_WEIGHT,
    COMMAND_PERIOD,
    COMMAND_TIMEOUT,
    DOMAIN_ID,
    INTERFACE,
    MODE_MACHINE,
    STATE_TOPIC,
)

DEFAULT_MODEL = Path("/opt/unitree_mujoco/unitree_robots/g1/g1_29dof.xml")
ARMS = slice(ARM_START, ARM_END)


def load_supported_model(path: Path) -> mujoco.MjModel:
    """Add a pelvis-to-world weld while keeping the stock actuator order."""
    root = ElementTree.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str(path.resolve().parent / compiler.attrib["meshdir"]))
    constraints = root.find("equality")
    if constraints is None:
        constraints = ElementTree.SubElement(root, "equality")
    ElementTree.SubElement(constraints, "weld", name="support_stand", body1="pelvis")
    model = mujoco.MjModel.from_xml_string(ElementTree.tostring(root, encoding="unicode"))
    if model.nu != ARM_END:
        raise ValueError("The scene does not match the 29-DoF G1 DDS actuator order")
    arm_names = tuple(model.actuator(index).name for index in range(ARM_START, ARM_END))
    arm_joint_ids = model.actuator_trnid[ARMS, 0]
    if arm_names != JOINT_NAMES or not np.array_equal(model.jnt_range[arm_joint_ids], list(JOINT_LIMITS.values())):
        raise ValueError("The scene does not match the configured G1 arm names and joint limits")
    return model


class ArmPlant:
    """Advance the same supported G1 physics with live or deterministic inputs."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        joint_ids = model.actuator_trnid[:, 0]
        self.position_indices = model.jnt_qposadr[joint_ids]
        self.velocity_indices = model.jnt_dofadr[joint_ids]
        self.arm_limits = model.jnt_range[joint_ids[ARMS]]
        self.hold = self.data.qpos[self.position_indices].copy()
        self.active = False

    def step(self, now: float, command) -> None:
        """Apply a fresh arm target or capture a hold when the writer goes silent."""
        positions = self.data.qpos[self.position_indices]
        velocities = self.data.qvel[self.velocity_indices]

        fresh = command is not None and now - command[0] < COMMAND_TIMEOUT
        active = fresh and command[1] > 0
        if self.active and not active:
            self.hold[ARMS] = positions[ARMS]
            print("Arm authority released" if fresh else "Arm command timeout. Holding measured pose.")
        self.active = active

        # The supported waist and legs keep their initial posture independently.
        torque = 30 * (self.hold - positions) - 3 * velocities
        torque += self.data.qfrc_bias[self.velocity_indices]
        if active:
            _, weight, values = command
            target, velocity, kp, kd, feedforward = values.T
            target = np.clip(target, self.arm_limits[:, 0], self.arm_limits[:, 1])
            user_torque = feedforward + kp * (target - positions[ARMS])
            user_torque += kd * (velocity - velocities[ARMS])
            torque[ARMS] = (1 - weight) * torque[ARMS] + weight * user_torque

        self.data.ctrl[:] = np.clip(
            torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1]
        )
        mujoco.mj_step(self.model, self.data)


class ArmSimulator:
    """Exchange arm commands and measured joint state through Unitree DDS."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.plant = ArmPlant(model)
        self.lock = Lock()
        self.latest = None
        self.state = unitree_hg_msg_dds__LowState_()
        self.state.mode_machine = MODE_MACHINE
        self.crc = CRC()
        self.publisher = ChannelPublisher(STATE_TOPIC, LowState_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber(ARM_TOPIC, LowCmd_)
        self.subscriber.Init(self.on_command, 1)

    def on_command(self, message: LowCmd_) -> None:
        """Accept a CRC-checked arm command without touching the physics state."""
        received_at = monotonic()
        if message.crc != self.crc.Crc(message):
            return

        weight = message.motor_cmd[ARM_WEIGHT].q
        motors = message.motor_cmd[ARMS]
        values = np.array([(m.q, m.dq, m.kp, m.kd, m.tau) for m in motors])
        if not 0 <= weight <= 1 or not np.isfinite(values).all():
            return
        if (values[:, 2:4] < 0).any():
            return

        with self.lock:
            self.latest = (received_at, weight, values)

    def publish_state(self) -> None:
        """Publish MuJoCo measurements in Unitree motor order."""
        plant = self.plant
        for index, (q_index, dq_index) in enumerate(zip(plant.position_indices, plant.velocity_indices)):
            state = self.state.motor_state[index]
            state.q = float(plant.data.qpos[q_index])
            state.dq = float(plant.data.qvel[dq_index])
        self.state.tick = round(plant.data.time * 1000) % (2**32)
        self.state.crc = self.crc.Crc(self.state)
        self.publisher.Write(self.state)

    def run(self) -> None:
        """Advance physics at the model timestep and publish at 50 Hz."""
        deadline = next_state = monotonic()
        step_period = self.plant.model.opt.timestep
        print(f"Supported G1 ready on {INTERFACE}, DDS domain {DOMAIN_ID}")
        while True:
            now = monotonic()
            with self.lock:
                command = self.latest
            self.plant.step(now, command)
            if now >= next_state:
                self.publish_state()
                next_state = now + COMMAND_PERIOD
            deadline = max(deadline + step_period, now)
            sleep(max(0, deadline - monotonic()))


def main() -> None:
    """Start the simulator on a Docker-local loopback interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    model = load_supported_model(parser.parse_args().model)
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    ArmSimulator(model).run()


if __name__ == "__main__":
    main()
