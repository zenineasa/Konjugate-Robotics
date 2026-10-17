# Copyright © 2026 Zenin Easa Panthakkalakath

"""Describes a MuJoCo model as the robotics toolbox builds a mechanism from it: its joints (name, kind, range and
starting position), the actuators that drive them (a position servo, a motor, or another kind, with its gain and
force limit) and its bodies. MuJoCo compiles the model first, so defaults, includes and mesh directories are resolved
as they will be in a run, which reading the XML by hand would not do.

    python describeMechanism.py <model.xml>

Prints JSON.
"""

import json
import sys

import mujoco

_JOINT_KINDS = {
    int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
    int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
}


def _name(model, kind, index):
    return mujoco.mj_id2name(model, kind, index) or ""


# An actuator's kind as MuJoCo's shortcuts make it: a position servo has a gain of kp on its setpoint and a bias of
# -kp on the joint's position; a motor has a unit gain and no bias.
def _actuator_kind(model, index):
    gain, bias = model.actuator_gainprm[index], model.actuator_biasprm[index]
    if int(model.actuator_gaintype[index]) == int(mujoco.mjtGain.mjGAIN_FIXED) and int(model.actuator_biastype[index]) == int(mujoco.mjtBias.mjBIAS_AFFINE):
        if gain[0] > 0 and bias[1] == -gain[0]:
            return "position"
    if int(model.actuator_biastype[index]) == int(mujoco.mjtBias.mjBIAS_NONE) and gain[0] == 1:
        return "motor"
    return "other"


def describe(path):
    model = mujoco.MjModel.from_xml_path(path)
    joints = []
    for index in range(model.njnt):
        limited = bool(model.jnt_limited[index])
        joints.append({
            "name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, index),
            "kind": _JOINT_KINDS[int(model.jnt_type[index])],
            "range": [float(value) for value in model.jnt_range[index]] if limited else None,
            "initialPosition": float(model.qpos0[model.jnt_qposadr[index]]),
        })
    actuators = []
    for index in range(model.nu):
        target = int(model.actuator_trnid[index][0])
        transmission = int(model.actuator_trntype[index])
        actuators.append({
            "name": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index),
            "joint": _name(model, mujoco.mjtObj.mjOBJ_JOINT, target) if transmission == int(mujoco.mjtTrn.mjTRN_JOINT) else None,
            "kind": _actuator_kind(model, index),
            "gain": float(model.actuator_gainprm[index][0]),
            "forceRange": [float(value) for value in model.actuator_forcerange[index]] if model.actuator_forcelimited[index] else None,
        })
    bodies = [_name(model, mujoco.mjtObj.mjOBJ_BODY, index) for index in range(1, model.nbody)]
    return {"timestep": float(model.opt.timestep), "joints": joints, "actuators": actuators, "bodies": bodies}


if __name__ == "__main__":
    json.dump(describe(sys.argv[1]), sys.stdout, indent=2)
