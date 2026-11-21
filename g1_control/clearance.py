"""Sample simulated arm clearance in an independent MuJoCo configuration."""

from threading import local

import mujoco

from g1_control.protocol import ARM_START
from g1_control.simulator import DEFAULT_MODEL, load_supported_model
from g1_control.transport import Angles

CLEARANCE_METRES = 0.012


class MujocoClearance:
    """Use stock collision geoms and add the visual hands to the shadow model."""

    def __init__(self) -> None:
        self.model = load_supported_model(DEFAULT_MODEL)
        model = self.model
        self.indices = model.jnt_qposadr[model.actuator_trnid[:, 0]]
        self.arm_geoms = set()
        self._thread = local()
        arm_bodies = set(model.jnt_bodyid[model.actuator_trnid[ARM_START:, 0]])
        for geom in range(model.ngeom):
            mesh_id = model.geom_dataid[geom]
            if model.geom_bodyid[geom] not in arm_bodies:
                continue
            if mesh_id >= 0 and model.mesh(mesh_id).name.endswith("rubber_hand"):
                model.geom_contype[geom] = model.geom_conaffinity[geom] = 1
                body = model.geom_bodyid[geom]
                model.body_contype[body] = model.body_conaffinity[body] = 1
            if model.geom_contype[geom] or model.geom_conaffinity[geom]:
                self.arm_geoms.add(geom)
                model.geom_margin[geom] = CLEARANCE_METRES

        # Factory joint housings have small intentional gaps at the reference pose.
        neutral = mujoco.MjData(model)
        mujoco.mj_fwdPosition(model, neutral)
        self.close_at_ready = {
            tuple(sorted(contact.geom)) for contact in neutral.contact if contact.dist > 0
        }

    def __call__(self, pose: Angles, lower: Angles) -> str | None:
        """Return the first arm clearance violation reported by the shadow model."""
        if not hasattr(self._thread, "data"):
            self._thread.data = mujoco.MjData(self.model)
        data = self._thread.data
        data.qpos[self.indices] = lower + pose
        mujoco.mj_fwdPosition(self.model, data)
        for contact in data.contact:
            a, b = contact.geom
            limit = 0.001 if tuple(sorted((a, b))) in self.close_at_ready else CLEARANCE_METRES
            if contact.dist >= limit or (a not in self.arm_geoms and b not in self.arm_geoms):
                continue
            first = self.model.body(self.model.geom_bodyid[a]).name.removesuffix("_link")
            second = self.model.body(self.model.geom_bodyid[b]).name.removesuffix("_link")
            return f"Insufficient clearance between {first.replace('_', ' ')} and {second.replace('_', ' ')}."
        return None
