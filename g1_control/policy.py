"""CPU inference for the pinned Unitree MJLab G1-29 velocity export."""

from math import cos, pi, sin
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml

from g1_control.protocol import ARM_END, COMMAND_PERIOD

POLICY_DIR = Path("/opt/g1-policy")


class VelocityPolicy:
    def __init__(self, joint_names: tuple[str, ...], directory: Path = POLICY_DIR) -> None:
        config = yaml.safe_load((directory / "deploy.yaml").read_text())
        self.order = np.asarray(config["joint_ids_map"])
        self.default = np.asarray(config["default_joint_pos"], dtype=np.float32)
        self.kp = np.asarray(config["stiffness"])
        self.kd = np.asarray(config["damping"])
        action = config["actions"]["JointPositionAction"]
        self.scale = np.asarray(action["scale"], dtype=np.float32)
        self.offset = np.asarray(action["offset"], dtype=np.float32)
        self.dt = config["step_dt"]
        self.period = config["observations"]["gait_phase"]["params"]["period"]
        ranges = config["commands"]["base_velocity"]["ranges"]
        self.bounds = tuple(ranges[name] for name in ("lin_vel_x", "lin_vel_y", "ang_vel_z"))

        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            directory / "policy.onnx", sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        metadata = self.session.get_modelmeta().custom_metadata_map
        names = tuple(name.strip().removesuffix("_joint") for name in metadata["joint_names"].split(","))
        expected_obs = (
            "base_ang_vel", "projected_gravity", "velocity_commands", "gait_phase",
            "joint_pos_rel", "joint_vel_rel", "last_action",
        )
        if (self.dt != COMMAND_PERIOD or sorted(self.order) != list(range(ARM_END))
                or names != tuple(joint_names[index] for index in self.order)
                or tuple(config["observations"]) != expected_obs
                or len(inputs) != 1 or inputs[0].shape[-1] != 98
                or len(outputs) != 1 or outputs[0].shape[-1] != ARM_END
                or any(values.shape != (ARM_END,) or not np.isfinite(values).all()
                       for values in (self.default, self.kp, self.kd, self.scale, self.offset))):
            raise ValueError("Locomotion policy does not match the Unitree 29-DoF deployment config")
        self.input = inputs[0].name
        self.reset()
        self.targets(self.default[np.argsort(self.order)], np.zeros(ARM_END), (1, 0, 0, 0),
                     (0, 0, 0), (0, 0, 0))
        self.reset()

    def reset(self) -> None:
        self.phase = 0.0
        self.previous = np.zeros(ARM_END, dtype=np.float32)

    def targets(self, q, dq, quaternion, gyro, velocity) -> np.ndarray:
        w, x, y, z = quaternion
        gravity = (2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1)
        self.phase = (self.phase + self.dt / self.period) % 1.0
        phase = (sin(2 * pi * self.phase), cos(2 * pi * self.phase)) if np.linalg.norm(velocity) >= 0.1 else (0, 0)
        obs = np.concatenate((gyro, gravity, velocity, phase, q[self.order] - self.default,
                              dq[self.order], self.previous), dtype=np.float32)
        action = self.session.run(None, {self.input: obs[None, :]})[0][0]
        if not np.isfinite(action).all():
            raise ValueError("Locomotion policy returned an invalid action")
        self.previous = action
        return self.offset + self.scale * action
