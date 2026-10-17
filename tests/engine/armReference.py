# Copyright © 2026 Zenin Easa Panthakkalakath

"""MuJoCo on its own, with joint motors integrated beside it, as a reference for a mechanism driven by Konjugate's
joint motors (packages/engine/components/jointMotor.json and jointDrive.json).

It follows the engine's step: at each step's start t every joint's torque is the gear ratio times the torque constant
times the motor's current at t; MuJoCo advances over [t, t + h] with that torque held (in whole steps of its own
timestep, shortened evenly to land on t + h, as the mechanism provider does); then each motor's current and winding
temperature take explicit Euler steps over [t, t + h], in the motor's substeps, with the joint's speed held at its
value at t (the motor reads it from the step's snapshot); and the battery's charge one step, from each motor's current
at t (the battery, a node of one substep, reads it from the snapshot too).

Reads a JSON request {mjcf, joints: [{name, initialPosition}], voltage, capacity, motors: [{joint, duty, resistance,
inductance, torqueConstant, gearRatio, thermalResistance, thermalCapacitance, ambientTemperature, substeps}], step,
steps} and prints, for each joint and at each of the steps + 1 instants from 0, its position and its motor's current
and winding temperature, and the battery's state of charge.
"""

import json
import math
import sys

import mujoco


def main(path):
    with open(path, encoding="utf-8") as handle:
        request = json.load(handle)
    model = mujoco.MjModel.from_xml_string(request["mjcf"])
    data = mujoco.MjData(model)
    addresses = {}
    for joint in request["joints"]:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
        addresses[joint["name"]] = (int(model.jnt_qposadr[index]), int(model.jnt_dofadr[index]))
        data.qpos[addresses[joint["name"]][0]] = joint.get("initialPosition", 0.0)
    mujoco.mj_forward(model, data)
    motors = request["motors"]
    currents = [0.0] * len(motors)
    temperatures = [motor["ambientTemperature"] for motor in motors]
    charge = 100.0
    step = request["step"]
    timestep = model.opt.timestep
    count = max(1, math.ceil(step / timestep - 1e-9))

    record = {"charge": [charge]}
    for motor in motors:
        record[motor["joint"]] = {"position": [float(data.qpos[addresses[motor["joint"]][0]])], "current": [0.0], "temperature": [temperatures[0]]}

    for _ in range(request["steps"]):
        speeds = [float(data.qvel[addresses[motor["joint"]][1]]) for motor in motors]
        data.qfrc_applied[:] = 0.0
        for motor, current in zip(motors, currents):
            data.qfrc_applied[addresses[motor["joint"]][1]] = motor["gearRatio"] * motor["torqueConstant"] * current
        model.opt.timestep = step / count
        for _ in range(count):
            mujoco.mj_step(model, data)
        model.opt.timestep = timestep

        drain = sum(-(100 * motor["duty"] * current / request["capacity"]) for motor, current in zip(motors, currents))
        for index, (motor, speed) in enumerate(zip(motors, speeds)):
            substeps = motor.get("substeps", 1)
            substep = step / substeps
            current, temperature = currents[index], temperatures[index]
            for _ in range(substeps):
                supply = (motor["duty"] * request["voltage"] - motor["resistance"] * current) / motor["inductance"]
                back_emf = -(motor["torqueConstant"] * motor["gearRatio"] * speed / motor["inductance"])
                heating = (motor["resistance"] * current ** 2 - (temperature - motor["ambientTemperature"]) / motor["thermalResistance"]) / motor["thermalCapacitance"]
                current, temperature = current + substep * (supply + back_emf), temperature + substep * heating
            currents[index], temperatures[index] = current, temperature
        charge += step * drain

        record["charge"].append(charge)
        for motor, current, temperature in zip(motors, currents, temperatures):
            entry = record[motor["joint"]]
            entry["position"].append(float(data.qpos[addresses[motor["joint"]][0]]))
            entry["current"].append(current)
            entry["temperature"].append(temperature)
    json.dump(record, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1])
