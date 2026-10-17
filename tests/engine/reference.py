# Copyright © 2026 Zenin Easa Panthakkalakath

"""MuJoCo on its own, as a reference for the mechanism provider run through Konjugate's engine.

Reads a JSON request {mjcf, joints: [{name, initialPosition, initialSpeed}], forces: {joint: value}, times: [...]}
and prints {joint: [position at each time]}: the model stepped at its own timestep, with each joint's force held
constant throughout.
"""

import json
import sys

import mujoco


def main(path):
    with open(path, encoding="utf-8") as handle:
        request = json.load(handle)
    model = mujoco.MjModel.from_xml_string(request["mjcf"])
    data = mujoco.MjData(model)
    joints = {}
    for joint in request["joints"]:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
        joints[joint["name"]] = (int(model.jnt_qposadr[index]), int(model.jnt_dofadr[index]))
        data.qpos[joints[joint["name"]][0]] = joint.get("initialPosition", 0.0)
        data.qvel[joints[joint["name"]][1]] = joint.get("initialSpeed", 0.0)
    for name, value in request.get("forces", {}).items():
        data.qfrc_applied[joints[name][1]] = value
    mujoco.mj_forward(model, data)
    positions = {name: [] for name in joints}
    timestep = model.opt.timestep
    for time in request["times"]:
        while data.time < time - timestep / 2:
            mujoco.mj_step(model, data)
        for name, (qpos, _) in joints.items():
            positions[name].append(float(data.qpos[qpos]))
    json.dump(positions, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1])
