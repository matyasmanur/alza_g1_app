"""Nonblocking app transport for Unitree SDK's synchronous G1 RPC calls."""

import json
from math import isfinite
from threading import Condition, Thread
from time import monotonic

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.rpc.client import Client

from g1_control.protocol import (
    COMMAND_TIMEOUT, SIM_SERVICE, SIM_SET_MODE, SPORT_STATE_TOPIC, ZERO_VELOCITY, SimMode,
)
from g1_control.transport import SimFeedback, Velocity


class DdsLocomotion:
    """Keep one pending mode request and one current velocity off the web loop."""

    def __init__(self) -> None:
        self._condition = Condition()
        self._mode = self._velocity = self._error = None
        self._latest = self._stamp = None
        self._closed = False
        self._subscriber = ChannelSubscriber(SPORT_STATE_TOPIC, SportModeState_)
        self._subscriber.Init(self._receive, 1)
        self._sport = LocoClient()
        self._sport.Init()
        self._sport.SetTimeout(0.18)
        self._sim = Client(SIM_SERVICE, False)
        self._sim._RegistApi(SIM_SET_MODE, 0)
        self._sim.SetTimeout(0.18)
        self._worker = Thread(target=self._run, name="g1-locomotion-rpc", daemon=True)
        self._worker.start()

    def _receive(self, message: SportModeState_) -> None:
        stamp = (message.stamp.sec, message.stamp.nanosec)
        if stamp == self._stamp:
            return
        try:
            mode = SimMode(message.mode)
        except ValueError:
            return
        position = tuple(message.position)
        quaternion = tuple(message.imu_state.quaternion)
        velocity = tuple(message.velocity)
        yaw_rate = message.yaw_speed
        if (all(map(isfinite, position + quaternion + velocity + (yaw_rate,)))
                and 0.99 < sum(q * q for q in quaternion) < 1.01):
            self._stamp = stamp
            self._latest = SimFeedback(monotonic(), mode, position, quaternion, velocity, yaw_rate)

    def read(self) -> SimFeedback | None:
        return self._latest

    def set_mode(self, mode: str) -> None:
        with self._condition:
            self._mode = mode
            self._velocity = None
            self._condition.notify()

    def set_velocity(self, velocity: Velocity) -> None:
        with self._condition:
            self._velocity = monotonic(), velocity
            self._condition.notify()

    def take_error(self) -> tuple[str, str] | None:
        with self._condition:
            result, self._error = self._error, None
        return result

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._mode is not None or self._velocity is not None)
                if self._closed:
                    break
                mode, self._mode = self._mode, None
                velocity = None
                if mode is None:
                    velocity, self._velocity = self._velocity, None
            try:
                if mode is not None:
                    code, _ = self._sim._Call(SIM_SET_MODE, json.dumps({"mode": mode}))
                else:
                    sent, command = velocity
                    remaining = COMMAND_TIMEOUT - (monotonic() - sent)
                    if remaining <= 0 and any(command):
                        continue
                    code = self._sport.SetVelocity(*command, max(0.02, remaining))
            except Exception as error:
                code = str(error)
            if code != 0:
                kind = "mode" if mode is not None else "velocity"
                with self._condition:
                    self._error = kind, f"Unitree {kind} request failed ({code})"
        self._sport.SetVelocity(*ZERO_VELOCITY)

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify()
        self._worker.join(timeout=0.5)
        self._subscriber.Close()
