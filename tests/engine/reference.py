# Copyright © 2026 Zenin Easa Panthakkalakath

"""MuJoCo on its own, as a reference for a drawn robot built into Konjugate and run by Konjugate's engine.

Reads a JSON request {mjcf, joints: [{name, initialPosition, initialSpeed}], forces: {joint: value},
controls: {actuator: value}, bodies: [name], times: [...]} and prints {joint: [position at each time]} and, for each body
named, {body: [[x, y, z] at each time]}: the model stepped at its own timestep, with each joint's force and each
actuator's control held constant throughout.
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
    for name, value in request.get("controls", {}).items():
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = value
    bodies = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in request.get("bodies", [])}
    mujoco.mj_forward(model, data)
    positions = {name: [] for name in joints}
    places = {name: [] for name in bodies}
    timestep = model.opt.timestep
    for time in request["times"]:
        while data.time < time - timestep / 2:
            mujoco.mj_step(model, data)
        for name, (qpos, _) in joints.items():
            positions[name].append(float(data.qpos[qpos]))
        for name, index in bodies.items():
            places[name].append([float(value) for value in data.xpos[index]])
    json.dump({**positions, **places}, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1])
