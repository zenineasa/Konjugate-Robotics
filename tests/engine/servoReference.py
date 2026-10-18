# Copyright © 2026 Zenin Easa Panthakkalakath

"""MuJoCo on its own, as a reference for a mechanism of servo joints run through Konjugate's engine: the same model
file, each servo joint's actuator given its setpoint (held throughout), every other joint's actuator switched off, the
model stepped at its own timestep.

A setpoint may instead follow a schedule ({joint: [[time, value], ...]}, linear between samples, held before the first
and after the last), set at the start of each engine step of `step` seconds as the engine reads it; a servo may be given
a torque limit of its own ({joint: N·m}).

Reads a JSON request {modelPath, joints: [{name, initialPosition}], setpoints: {joint: value}, schedules, torqueLimits,
step, times, bodies} and
prints, for each joint, its position at each time and, for a servo joint, the torque its servo applies; and for each
body named, its place in the world at each time (position and quaternion, as `bodies`).
"""

import json
import math
import sys

import mujoco


def main(path):
    with open(path, encoding="utf-8") as handle:
        request = json.load(handle)
    model = mujoco.MjModel.from_xml_path(request["modelPath"])
    data = mujoco.MjData(model)
    schedules = request.get("schedules", {})
    setpoints = {**request.get("setpoints", {}), **{name: samples[0][1] for name, samples in schedules.items()}}
    joints = {}
    for joint in request["joints"]:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
        actuators = [actuator for actuator in range(model.nu)
                     if int(model.actuator_trntype[actuator]) == int(mujoco.mjtTrn.mjTRN_JOINT) and int(model.actuator_trnid[actuator][0]) == index]
        joints[joint["name"]] = (int(model.jnt_qposadr[index]), actuators)
        data.qpos[joints[joint["name"]][0]] = joint.get("initialPosition", 0.0)
        if joint["name"] in setpoints:
            data.ctrl[actuators[0]] = setpoints[joint["name"]]
        else:
            for actuator in actuators:
                model.actuator_gainprm[actuator][:] = 0.0
                model.actuator_biasprm[actuator][:] = 0.0
    for name, limit in request.get("torqueLimits", {}).items():
        actuator = joints[name][1][0]
        model.actuator_forcelimited[actuator] = 1
        model.actuator_forcerange[actuator] = [-limit, limit]
    mujoco.mj_forward(model, data)
    record = {name: {"position": [], "servoTorque": []} for name in joints}
    bodies = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in request.get("bodies", [])}
    record["bodies"] = {name: [] for name in bodies}
    timestep = model.opt.timestep
    step = request.get("step")

    def scheduled(samples, time):
        if time <= samples[0][0]:
            return samples[0][1]
        for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
            if time <= t1:
                return v0 + (v1 - v0) * (time - t0) / (t1 - t0)
        return samples[-1][1]

    reached = 0.0
    for time in request["times"]:
        if step is None:
            while data.time < time - timestep / 2:
                mujoco.mj_step(model, data)
        else:
            # Engine steps, each with the setpoints read at its start and MuJoCo's steps shortened evenly to land on its end.
            while reached < time - step / 2:
                for name, samples in schedules.items():
                    data.ctrl[joints[name][1][0]] = scheduled(samples, reached)
                count = max(1, math.ceil(step / timestep - 1e-9))
                model.opt.timestep = step / count
                for _ in range(count):
                    mujoco.mj_step(model, data)
                model.opt.timestep = timestep
                reached += step
        for name, (qpos, actuators) in joints.items():
            record[name]["position"].append(float(data.qpos[qpos]))
            if name in setpoints:
                record[name]["servoTorque"].append(float(data.actuator_force[actuators[0]]))
        for name, index in bodies.items():
            record["bodies"][name].append([float(value) for value in (*data.xpos[index], *data.xquat[index])])
    json.dump(record, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1])
