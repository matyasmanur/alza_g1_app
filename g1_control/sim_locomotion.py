"""Simulated SDK locomotion service and exclusive whole-body motor authority."""

import json
from math import atan2, cos, sin
from threading import Lock
from time import monotonic
from typing import TYPE_CHECKING

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_api import (
    LOCO_API_VERSION, LOCO_SERVICE_NAME, ROBOT_API_ID_LOCO_SET_VELOCITY,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.rpc.internal import RPC_ERR_SERVER_INTERNAL
from unitree_sdk2py.rpc.server import Server
from unitree_sdk2py.utils.crc import CRC

from g1_control.joints import PARKED
from g1_control.policy import VelocityPolicy
from g1_control.protocol import (
    ARM_END, ARM_START, COMMAND_TIMEOUT, LOW_COMMAND_TOPIC, MODE_MACHINE, SIM_API_VERSION,
    SIM_SERVICE, SIM_SET_MODE, SPORT_STATE_TOPIC, STATE_TOPIC, ZERO_VELOCITY, SimMode,
)

if TYPE_CHECKING:
    from g1_control.simulator import ArmPlant

PREPARE_SECONDS = 1.6
SUPPORT_SECONDS = 0.8
SETTLE_SECONDS = 0.35


def blend(elapsed: float, duration: float) -> float:
    t = min(elapsed / duration, 1.0)
    return t * t * (3 - 2 * t)


class SimLocomotion:
    def __init__(self, plant: "ArmPlant") -> None:
        self.plant = plant
        self.data = data = plant.data
        model = plant.model
        self.policy = VelocityPolicy(tuple(model.actuator(i).name for i in range(model.nu)))
        self.support = model.equality("support_stand").id
        self.anchor = model.body("support_anchor").mocapid.item()
        self.neutral = data.qpos[plant.position_indices].copy()
        self.neutral[:ARM_START] = model.qpos0[plant.position_indices[:ARM_START]]
        self.limits = model.jnt_range[model.actuator_trnid[:, 0]].T
        self.mode = SimMode.ARMS
        self.started = data.time
        self.quiet_since = None
        self.footing_since = None
        self.lock = Lock()
        self.pending = None
        self.velocity = (0.0, ZERO_VELOCITY)
        self.low_command = None
        self.low_state = None
        self.imu_quat = data.sensor("imu_quat").data
        self.imu_gyro = data.sensor("imu_gyro").data
        self.imu_acc = data.sensor("imu_acc").data
        self.crc_state, self.crc_motor, self.crc_write = CRC(), CRC(), CRC()

        self.state = unitree_go_msg_dds__SportModeState_()
        self.state_pub = ChannelPublisher(SPORT_STATE_TOPIC, SportModeState_)
        self.state_pub.Init()
        self.motor = unitree_hg_msg_dds__LowCmd_()
        self.motor.mode_machine = MODE_MACHINE
        for i, kp, kd in zip(self.policy.order, self.policy.kp, self.policy.kd):
            motor = self.motor.motor_cmd[i]
            motor.mode, motor.kp, motor.kd = 1, float(kp), float(kd)
        self.motor_pub = ChannelPublisher(LOW_COMMAND_TOPIC, LowCmd_)
        self.motor_pub.Init()
        self.state_sub = ChannelSubscriber(STATE_TOPIC, LowState_)
        self.state_sub.Init(self.receive_state, 1)
        self.motor_sub = ChannelSubscriber(LOW_COMMAND_TOPIC, LowCmd_)
        self.motor_sub.Init(self.receive_motor, 1)

        self.sport = Server(LOCO_SERVICE_NAME)
        self.sport._SetApiVersion(LOCO_API_VERSION)
        self.sport._RegistHandler(ROBOT_API_ID_LOCO_SET_VELOCITY, self.set_velocity, False)
        self.sport.Start()
        self.mode_server = Server(SIM_SERVICE)
        self.mode_server._SetApiVersion(SIM_API_VERSION)
        self.mode_server._RegistHandler(SIM_SET_MODE, self.set_mode, False)
        self.mode_server.Start()

    def set_velocity(self, parameter: str) -> tuple[int, str]:
        request = json.loads(parameter)
        velocity, duration = request["velocity"], request["duration"]
        if (not isinstance(velocity, list) or len(velocity) != 3
                or any(type(v) not in (float, int) or not low <= v <= high
                       for v, (low, high) in zip(velocity, self.policy.bounds))
                or type(duration) not in (float, int) or not 0 < duration <= 1.0):
            return RPC_ERR_SERVER_INTERNAL, ""
        with self.lock:
            if any(velocity) and self.mode is not SimMode.LOCOMOTION:
                return RPC_ERR_SERVER_INTERNAL, ""
            self.velocity = monotonic() + duration, tuple(velocity)
        return 0, ""

    def set_mode(self, parameter: str) -> tuple[int, str]:
        request = json.loads(parameter)["mode"]
        with self.lock:
            allowed = {
                "locomotion": self.mode is SimMode.ARMS,
                "arms": self.mode is not SimMode.FALLEN,
                "reset": self.mode is SimMode.FALLEN,
            }
            if (not isinstance(request, str) or not allowed.get(request)
                    or self.pending is not None and request != "arms"):
                return RPC_ERR_SERVER_INTERNAL, ""
            self.pending = request
            self.velocity = 0.0, ZERO_VELOCITY
        return 0, ""

    def receive_state(self, message: LowState_) -> None:
        if message.mode_machine == MODE_MACHINE and self.crc_state.Crc(message) == message.crc:
            self.low_state = monotonic(), message

    def receive_motor(self, message: LowCmd_) -> None:
        if message.mode_machine != MODE_MACHINE or self.crc_motor.Crc(message) != message.crc:
            return
        values = np.array([(m.q, m.dq, m.kp, m.kd, m.tau) for m in message.motor_cmd[:ARM_END]])
        if np.isfinite(values).all() and (values[:, 2:4] >= 0).all():
            self.low_command = monotonic(), values

    def fill_imu(self, imu) -> None:
        imu.quaternion = self.imu_quat.tolist()
        imu.gyroscope = self.imu_gyro.tolist()
        imu.accelerometer = self.imu_acc.tolist()

    def publish_state(self, tick: int) -> None:
        self.state.stamp.sec, millis = divmod(tick, 1000)
        self.state.stamp.nanosec = millis * 1_000_000
        self.state.mode = int(self.mode)
        self.state.position = self.data.qpos[:3].tolist()
        self.state.velocity = self.data.qvel[:3].tolist()
        self.state.yaw_speed = float(self.imu_gyro[2])
        self.fill_imu(self.state.imu_state)
        self.state_pub.Write(self.state)

        ready = self.mode is SimMode.PREPARING and self.data.time - self.started >= PREPARE_SECONDS
        if ready or self.mode in (SimMode.LOCOMOTION, SimMode.SETTLING):
            self.publish_targets()

    def publish_targets(self) -> None:
        now = monotonic()
        sample = self.low_state
        if sample is None or now - sample[0] >= COMMAND_TIMEOUT:
            return
        state = sample[1]
        with self.lock:
            expires, velocity = self.velocity
        if now >= expires or self.mode is not SimMode.LOCOMOTION:
            velocity = ZERO_VELOCITY
        try:
            motors = state.motor_state[:ARM_END]
            targets = self.policy.targets(
                np.array([m.q for m in motors]), np.array([m.dq for m in motors]),
                state.imu_state.quaternion, state.imu_state.gyroscope, velocity,
            )
        except Exception as error:
            self.fall(str(error))
            return
        for index, target in zip(self.policy.order, targets):
            self.motor.motor_cmd[index].q = float(target)
        self.motor.crc = self.crc_write.Crc(self.motor)
        self.motor_pub.Write(self.motor)

    def change_mode(self, requested: str) -> None:
        data = self.data
        if requested == "locomotion":
            if self.mode is not SimMode.ARMS:
                return
            self.mode = SimMode.PREPARING
            self.from_pose = data.qpos[self.plant.position_indices].copy()
            self.plant.active = False
            self.footing_since = None
            self.policy.reset()
            self.low_command = None
        elif requested == "arms":
            if self.mode in (SimMode.ARMS, SimMode.SUPPORTING, SimMode.FALLEN):
                return
            if data.eq_active[self.support]:
                self.start_support()
            else:
                self.mode = SimMode.SETTLING
                self.quiet_since = None
        else:
            # Keep telemetry ticks advancing when the operator resets the scene.
            clock = data.time
            mujoco.mj_resetData(self.plant.model, data)
            data.time = clock
            data.qpos[self.plant.position_indices[ARM_START:]] = PARKED
            self.plant.hold[:] = self.neutral
            self.plant.active = False
            mujoco.mj_forward(self.plant.model, data)
            self.mode = SimMode.ARMS
        self.started = data.time

    def start_support(self) -> None:
        data = self.data
        self.from_pose = data.qpos[self.plant.position_indices].copy()
        self.from_base = data.qpos[:7].copy()
        w, x, y, z = self.from_base[3:]
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.heading = np.array([cos(yaw / 2), 0.0, 0.0, sin(yaw / 2)])
        if np.dot(self.heading, self.from_base[3:]) < 0:
            self.heading *= -1
        data.mocap_pos[self.anchor] = self.from_base[:3]
        data.mocap_quat[self.anchor] = self.from_base[3:]
        data.eq_active[self.support] = True
        self.mode = SimMode.SUPPORTING
        self.started = data.time

    def fall(self, reason: str) -> None:
        print(f"Locomotion stopped: {reason}")
        self.data.eq_active[self.support] = False
        self.mode = SimMode.FALLEN
        with self.lock:
            self.velocity = 0.0, ZERO_VELOCITY

    def step(self, now: float) -> bool:
        with self.lock:
            request, self.pending = self.pending, None
        if request is not None:
            self.change_mode(request)
        if self.mode is SimMode.ARMS:
            return False

        data, plant = self.data, self.plant
        q, dq = data.qpos[plant.position_indices], data.qvel[plant.velocity_indices]
        elapsed = data.time - self.started
        if self.mode is SimMode.SUPPORTING:
            weight = blend(elapsed, SUPPORT_SECONDS)
            data.mocap_pos[self.anchor, 2] = (1 - weight) * self.from_base[2] + weight * plant.model.qpos0[2]
            rotation = (1 - weight) * self.from_base[3:] + weight * self.heading
            data.mocap_quat[self.anchor] = rotation / np.linalg.norm(rotation)
            plant.hold[ARM_START:] = self.from_pose[ARM_START:]
            plant.hold[:ARM_START] = (1 - weight) * self.from_pose[:ARM_START] + weight * self.neutral[:ARM_START]
            plant.step(now, None)
            if elapsed >= SUPPORT_SECONDS + SETTLE_SECONDS:
                self.mode = SimMode.ARMS
            return True

        motor = self.low_command
        fresh = motor is not None and now - motor[0] < COMMAND_TIMEOUT
        if self.mode is SimMode.PREPARING and data.eq_active[self.support]:
            weight = blend(elapsed, PREPARE_SECONDS)
            plant.hold[:] = (1 - weight) * self.from_pose
            plant.hold[self.policy.order] += weight * self.policy.default
            if elapsed < PREPARE_SECONDS or not fresh:
                if elapsed > PREPARE_SECONDS + 1:
                    self.start_support()
                plant.step(now, None)
                return True
            data.eq_active[self.support] = False
            self.footing_since = data.time

        upright = 1 - 2 * (data.qpos[4] ** 2 + data.qpos[5] ** 2)
        if self.mode is not SimMode.FALLEN and (not fresh or data.qpos[2] < 0.55 or upright < 0.60):
            self.fall("motor commands expired" if not fresh else "G1 lost balance")
        if self.mode is SimMode.FALLEN:
            plant.apply_torque(-2 * dq)
            return True

        target, speed, kp, kd, feedforward = motor[1].T
        plant.apply_torque(feedforward + kp * (np.clip(target, *self.limits) - q) + kd * (speed - dq))
        if self.mode is SimMode.PREPARING and data.time - self.footing_since >= 0.7:
            self.mode = SimMode.LOCOMOTION
        elif self.mode is SimMode.SETTLING:
            quiet = np.linalg.norm(data.qvel[:3]) < 0.08 and np.linalg.norm(data.qvel[3:6]) < 0.25 and upright > 0.97
            if not quiet:
                self.quiet_since = None
            elif self.quiet_since is None:
                self.quiet_since = data.time
            elif data.time - self.quiet_since >= SETTLE_SECONDS:
                self.start_support()
            if self.mode is SimMode.SETTLING and elapsed >= 5.0:
                self.mode = SimMode.LOCOMOTION
        return True
