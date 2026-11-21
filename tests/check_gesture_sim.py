"""Replay the catalog in actual MuJoCo physics at a deterministic simulated cadence."""

from math import radians

import numpy as np

from g1_control.clearance import MujocoClearance
from g1_control.controller import Motion, MotionController, Rejected
from g1_control.dds_transport import ARM_GAINS
from g1_control.gestures import GESTURES, pose
from g1_control.protocol import ARM_START, COMMAND_PERIOD
from g1_control.simulator import ArmPlant, DEFAULT_MODEL, load_supported_model
from g1_control.trajectory import READY
from g1_control.transport import Feedback

OPERATOR = "gesture-check"


class SimulationRun:
    """Use the live plant and arm gains while replacing DDS timing with a fixed clock."""

    def __init__(self, model, clearance):
        self.plant = ArmPlant(model)
        self.now = 0.0
        self.command = None
        self.dt = model.opt.timestep
        self.controller = MotionController(self, lambda: self.now, clearance=clearance)
        self.controller.tick()
        self.controller.take_control(OPERATOR)
        self.controller.enable(OPERATOR)

    def read(self):
        a = self.plant
        position = tuple(a.data.qpos[a.position_indices])
        velocity = tuple(a.data.qvel[a.velocity_indices[ARM_START:]])
        return Feedback(self.now, position[ARM_START:], velocity, position[:ARM_START])

    def write(self, position, velocity, *, enabled):
        values = np.array([(q, dq, kp, kd, 0.0) for q, dq, (kp, kd) in zip(position, velocity, ARM_GAINS)])
        self.command = self.now, float(enabled), values

    def step(self):
        self.controller.heartbeat(OPERATOR)
        for _ in range(round(COMMAND_PERIOD / self.dt)):
            self.now += self.dt
            self.plant.step(self.now, self.command)
        self.controller.tick()
        if self.controller.status.fault:
            raise AssertionError(self.controller.status.fault)

    def settle(self, max_ticks=1800):
        """Require feedback to settle and return the largest observed angle."""
        maximum = 0.0
        for _ in range(max_ticks):
            self.step()
            maximum = max(maximum, max(map(abs, self.read().position)))
            if self.controller.status.motion is Motion.IDLE:
                return maximum
        raise AssertionError(f"Did not settle: {self.controller.status}")

    def start(self, gesture):
        request = self.controller.prepare_gesture(OPERATOR, gesture.name, gesture.frames)
        segments = self.controller.plan_gesture(request)
        self.controller.finish_plan(OPERATOR, request, segments)


def check_catalog(model, clearance):
    for gesture in GESTURES.values():
        try:
            for off_ready in (False, True):
                run = SimulationRun(model, clearance)
                if off_ready:
                    run.controller.set_joint(OPERATOR, "left_elbow", 0.24)
                    run.controller.set_joint(OPERATOR, "right_wrist_roll", -0.20)
                    run.settle()
                    assert max(map(abs, run.read().position)) > 0.15
                run.start(gesture)
                assert run.settle() > 0.3
                assert max(map(abs, run.read().position)) < 0.08

            run = SimulationRun(model, clearance)
            run.start(gesture)
            for _ in range(125):
                run.step()
                if max(map(abs, run.read().velocity)) > 0.12 and max(map(abs, run.read().position)) > 0.12:
                    break
            else:
                raise AssertionError("Gesture was not observably moving before Stop")
            run.controller.stop(OPERATOR)
            run.settle()
            for _ in range(30):
                run.step()
            assert run.controller.status.stop_latched and not run.plant.active
            assert max(map(abs, run.read().position)) > 0.09, "Stop returned to Ready"
        except (AssertionError, ValueError) as error:
            raise AssertionError(f"{gesture.name}: {error}") from error
        print(f"PASS {gesture.name}: Ready, off-Ready, return, Stop")


def check_rejection(model, clearance):
    run = SimulationRun(model, clearance)
    for request in (
        lambda: run.controller.set_joint(OPERATOR, "left_shoulder_roll", radians(-50)),
        lambda: run.controller.start_gesture(OPERATOR, (pose((0, -50, 0, 0, 0, 0, 0)).pose,)),
    ):
        try:
            request()
        except Rejected as error:
            assert "clearance" in str(error).lower(), error
        else:
            raise AssertionError("Overlapping arm target was accepted")
    assert run.controller.status.target == READY and run.command is None
    print("PASS modeled interference: manual target and gesture rejected before movement")


if __name__ == "__main__":
    plant_model = load_supported_model(DEFAULT_MODEL)
    guard = MujocoClearance()
    check_rejection(plant_model, guard)
    check_catalog(plant_model, guard)
