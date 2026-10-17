# Copyright © 2026 Zenin Easa Panthakkalakath

"""MuJoCo on its own, as a reference for a mechanism of servo joints run through Konjugate's engine: the same model
file, each servo joint's actuator given its setpoint (held throughout), every other joint's actuator switched off, the
model stepped at its own timestep.

Reads a JSON request {modelPath, joints: [{name, initialPosition}], setpoints: {joint: value}, times} and prints, for
each joint, its position at each time and, for a servo joint, the torque its servo applies.
"""

import json
import sys

import mujoco


def main(path):
    with open(path, encoding="utf-8") as handle:
        request = json.load(handle)
    model = mujoco.MjModel.from_xml_path(request["modelPath"])
    data = mujoco.MjData(model)
    setpoints = request.get("setpoints", {})
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
    mujoco.mj_forward(model, data)
    record = {name: {"position": [], "servoTorque": []} for name in joints}
    timestep = model.opt.timestep
    for time in request["times"]:
        while data.time < time - timestep / 2:
            mujoco.mj_step(model, data)
        for name, (qpos, actuators) in joints.items():
            record[name]["position"].append(float(data.qpos[qpos]))
            if name in setpoints:
                record[name]["servoTorque"].append(float(data.actuator_force[actuators[0]]))
    json.dump(record, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1])
