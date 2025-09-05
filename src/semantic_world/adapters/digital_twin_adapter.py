"""
Digital-Twin adaptor: bridges semantic_world views to an actual simulator (MuJoCo).
Use case: open / close a cabinet door by name, using the joint mapped from the semantic view.

Adjust the imports below to match your repo's layout.
"""

from dataclasses import dataclass
from typing import Optional

import mujoco as mj

# ⬇️ Adjust these imports if your module paths differ
from semantic_world.world import Body
from semantic_world.prefixed_name import PrefixedName
from semantic_world.views.views import Cabinet, Door, Handle  # your dataclasses

# ---------- Build a semantic Cabinet view from a MuJoCo model ----------

def _mk_body(model: mj.MjModel, data: mj.MjData, body_name: str) -> Body:
    """
    Build a lightweight Body compatible with your repo's dataclass.
    We avoid passing model/data/id because your Body(...) doesn't accept them.
    """
    # Optional sanity: ensure the body exists in the MuJoCo model
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"Body '{body_name}' not found in MuJoCo model")

    # Prefer PrefixedName if your Body expects it; fall back to plain str if needed.
    try:
        return Body(name=PrefixedName(body_name, "Body"))
    except TypeError:
        # If Body(name=...) expects a plain string, use that:
        return Body(name=body_name)


def build_cabinet_view(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    cabinet_body: str,
    door_body: str,
    handle_body: str,
    door_joint: Optional[str] = None
) -> Cabinet:
    """Create a Cabinet→Door→Handle semantic view from MuJoCo names."""
    cab_body = _mk_body(model, data, cabinet_body)
    d_body   = _mk_body(model, data, door_body)
    h_body   = _mk_body(model, data, handle_body)

    handle = Handle(body=h_body)
    door   = Door(body=d_body, handle=handle)

    # Derive joint type if provided
    joint_type = None
    if door_joint:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, door_joint)
        if jid >= 0:
            jtype = model.jnt_type[jid]
            if jtype == mj.mjtJoint.mjJNT_HINGE:
                joint_type = "hinge"
            elif jtype == mj.mjtJoint.mjJNT_SLIDE:
                joint_type = "prismatic"
            else:
                joint_type = "other"

    return Cabinet(
        body=cab_body,
        doors=[door],
        drawers=[],
        joint_type=joint_type,
        joint_sim_name=door_joint,
        description="Semantic view built from MuJoCo model",
    )

# ---------- Digital-Twin Controller bound to the semantic view ----------

@dataclass
class DigitalTwinCabinetAgent:
    """Tiny agent that uses the semantic Cabinet view to actuate the correct joint."""
    model: mj.MjModel
    data: mj.MjData
    cab: Cabinet

    @property
    def _door_jid(self) -> int:
        if not self.cab.joint_sim_name:
            raise RuntimeError("Cabinet view has no joint_sim_name set.")
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, self.cab.joint_sim_name)
        if jid < 0:
            raise RuntimeError(f"Joint '{self.cab.joint_sim_name}' not found in model.")
        return jid

    def set_fraction_open(self, frac: float) -> None:
        """
        Set door openness in [0,1].
        - hinge: maps to angle between [closed, open], here we assume [0, +90°]
        - prismatic: maps to linear slide, assume [0, +0.25 m]
        """
        jid = self._door_jid
        jtype = self.model.jnt_type[jid]

        frac = max(0.0, min(1.0, frac))

        if jtype == mj.mjtJoint.mjJNT_HINGE:
            closed = 0.0
            open_  = 1.57079632679 / 2.0  # 90° = 1.5708 rad; or choose your range; here 45°
            target = closed + frac * (open_ - closed)
        elif jtype == mj.mjtJoint.mjJNT_SLIDE:
            closed = 0.0
            open_  = 0.25  # 25 cm slide
            target = closed + frac * (open_ - closed)
        else:
            raise RuntimeError("Unsupported joint type for demo.")

        qposadr = self.model.jnt_qposadr[jid]
        self.data.qpos[qposadr] = target
        mj.mj_forward(self.model, self.data)

    def open(self, percent: float = 60.0) -> None:
        """Open the cabinet door to N percent (0–100)."""
        self.set_fraction_open(percent / 100.0)

    def close(self) -> None:
        """Close the cabinet door."""
        self.set_fraction_open(0.0)

    def is_open(self, thresh_percent: float = 10.0) -> bool:
        """Simple check based on joint value vs. a threshold."""
        jid = self._door_jid
        qposadr = self.model.jnt_qposadr[jid]
        val = float(self.data.qpos[qposadr])

        if self.model.jnt_type[jid] == mj.mjtJoint.mjJNT_HINGE:
            return val > (1.57079632679 / 2.0) * (thresh_percent / 100.0)
        elif self.model.jnt_type[jid] == mj.mjtJoint.mjJNT_SLIDE:
            return val > 0.25 * (thresh_percent / 100.0)
        else:
            return False

