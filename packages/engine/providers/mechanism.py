# Copyright © 2026 Zenin Easa Panthakkalakath

"""One mechanism, solved by MuJoCo, as a Konjugate computational-node provider.

The mechanism node holds, for each joint, its position and speed, which this provider sets (as algebraic values).
Each joint is in one of three modes:
- torque: it reads a torque (or, for a slide, a force) from a state, one of the mechanism node's own or another node's,
  such as a motor's, which the engine reads from the step's synchronization snapshot;
- servo: it reads a setpoint the same way, and the model's own position actuator on the joint (a servo's inner loop,
  run by MuJoCo at every one of its steps) drives it there; the torque the servo applies is a state too;
- free: nothing drives it.
An actuator the model has on a joint in torque or free mode is switched off, so nothing drives a joint that the
mechanism does not say drives it.

The mechanism may also read shared parameters, each setting one quantity of MuJoCo's model: a body's mass (its inertia
scaled with it, as for a body of the same shape and density), or a joint's damping, friction loss or armature. Each is
applied before a step whenever its value has changed, so a live slider, a stored schedule, a fork's intervention and
fitting all reach the physics; MuJoCo's derived quantities are recomputed after a change of mass or armature.

Each call is a co-simulation step, as an imported FMU's is: called at a substep's start time t with its step h, the
provider advances MuJoCo over [t, t + h] with each joint's torque held at the value read at t, and reports the joints
at t + h. The engine writes those values after the substep's other contributions have read the ones from t, and
records them beside the node's other states at t + h. MuJoCo keeps the mechanism's true state between calls, in its
own timestep, shortened evenly within a step so as to land on t + h.

The model builder (packages/toolbox/lib/mechanismModel.mjs) writes the configuration below.
"""

import json
import math
import struct

import mujoco
import numpy

from konjugate import NodeProvider, NodeProviderDescription, ScalarPort

CONFIGURATION = json.loads(r'''{}''')

# Times closer than this, relative to their size, are the same instant (the engine's times are sums of step sizes).
_SAME_TIME = 1e-9

# MuJoCo's own warnings that it found the simulation unstable. On one it resets its state to the model's initial pose
# and carries on, which would leave a run looking calm when it is not; the provider stops the run instead.
_INSTABILITY_WARNINGS = {
    int(mujoco.mjtWarning.mjWARN_BADQACC): "accelerations",
    int(mujoco.mjtWarning.mjWARN_BADQVEL): "speeds",
    int(mujoco.mjtWarning.mjWARN_BADQPOS): "positions",
}

# MuJoCo's warning messages, kept here for the error that stops the run, rather than printed and written to a
# MUJOCO_LOG.TXT in whatever directory the engine runs in.
_warnings = []
mujoco.set_mju_user_warning(_warnings.append)


class MechanismNode(NodeProvider):
    def __init__(self):
        self._configuration = CONFIGURATION
        if "modelPath" in self._configuration:
            # A model file on this computer, its meshes beside it (one fetched from Menagerie, say).
            self._model = mujoco.MjModel.from_xml_path(self._configuration["modelPath"])
        else:
            self._model = mujoco.MjModel.from_xml_string(self._configuration["mjcf"])
        self._data = mujoco.MjData(self._model)
        self._timestep = self._model.opt.timestep
        self._joints = []
        for joint in self._configuration["joints"]:
            index = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
            if index < 0:
                raise ValueError(f"The mechanism has no joint named {joint['name']}.")
            if int(self._model.jnt_type[index]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise ValueError(f"Joint {joint['name']} is neither a hinge nor a slide; only those are supported yet.")
            mode = joint.get("mode", "torque" if joint.get("driven") else "free")
            actuators = [actuator for actuator in range(self._model.nu)
                         if int(self._model.actuator_trntype[actuator]) == int(mujoco.mjtTrn.mjTRN_JOINT) and int(self._model.actuator_trnid[actuator][0]) == index]
            if mode == "servo":
                if len(actuators) != 1:
                    raise ValueError(f"Joint {joint['name']} in servo mode needs one actuator of the model on it; it has {len(actuators)}.")
            else:
                for actuator in actuators:
                    self._model.actuator_gainprm[actuator][:] = 0.0
                    self._model.actuator_biasprm[actuator][:] = 0.0
            self._joints.append({
                "key": joint["key"],
                "mode": mode,
                "actuator": actuators[0] if mode == "servo" else None,
                "qpos": int(self._model.jnt_qposadr[index]),
                "dof": int(self._model.jnt_dofadr[index]),
            })
            self._data.qpos[self._joints[-1]["qpos"]] = float(joint.get("initialPosition", 0.0))
            self._data.qvel[self._joints[-1]["dof"]] = float(joint.get("initialSpeed", 0.0))
        self._parameters = [self._parameter(parameter) for parameter in self._configuration.get("parameters", [])]
        mujoco.mj_forward(self._model, self._data)
        # The time MuJoCo has reached; None until the first call.
        self._time = None

    # A parameter's place in MuJoCo's model: the array it sets and the index in it.
    def _parameter(self, parameter):
        kind, target = parameter["kind"], parameter["target"]
        if kind == "bodyMass":
            index = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, target)
            if index < 0:
                raise ValueError(f"The mechanism has no body named {target}.")
            if not self._model.body_mass[index] > 0:
                raise ValueError(f"Body {target} has no mass to scale its inertia from; give it one in the MJCF.")
            return {"key": parameter["key"], "kind": kind, "index": index}
        if kind in ("jointDamping", "jointFrictionloss", "jointArmature"):
            index = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, target)
            if index < 0:
                raise ValueError(f"The mechanism has no joint named {target}.")
            return {"key": parameter["key"], "kind": kind, "index": int(self._model.jnt_dofadr[index])}
        raise ValueError(f"Parameter {parameter['key']} sets {kind}, which the mechanism does not know.")

    # Sets each parameter's quantity in MuJoCo's model to its value, where it differs; True if a mass or armature changed.
    def _apply_parameters(self, inputs, start):
        changed_constants = False
        for parameter in self._parameters:
            value = inputs[parameter["key"]]
            if not math.isfinite(value) or (parameter["kind"] == "bodyMass" and not value > 0):
                raise RuntimeError(f"Parameter {parameter['key']} was given {value} at {start} s.")
            index = parameter["index"]
            if parameter["kind"] == "bodyMass":
                if self._model.body_mass[index] != value:
                    self._model.body_inertia[index] *= value / self._model.body_mass[index]
                    self._model.body_mass[index] = value
                    changed_constants = True
            else:
                array = {"jointDamping": self._model.dof_damping, "jointFrictionloss": self._model.dof_frictionloss, "jointArmature": self._model.dof_armature}[parameter["kind"]]
                if array[index] != value:
                    array[index] = value
                    changed_constants = changed_constants or parameter["kind"] == "jointArmature"
        if changed_constants:
            # mj_setConst computes at the model's initial pose, using the data as scratch: keep the state around it.
            spec = mujoco.mjtState.mjSTATE_INTEGRATION
            state = numpy.zeros(mujoco.mj_stateSize(self._model, spec))
            mujoco.mj_getState(self._model, self._data, state, spec)
            mujoco.mj_setConst(self._model, self._data)
            mujoco.mj_setState(self._model, self._data, state, spec)
            mujoco.mj_forward(self._model, self._data)

    def describe(self):
        return NodeProviderDescription(
            self._configuration.get("providerId", "konjugate.robotics.mechanism"),
            self._configuration.get("name", "Mechanism"),
            [ScalarPort(f"{joint['key']}Effort", "Torque or force applied", "") for joint in self._joints if joint["mode"] == "torque"]
            + [ScalarPort(f"{joint['key']}Setpoint", "Servo setpoint", "") for joint in self._joints if joint["mode"] == "servo"]
            + [ScalarPort(parameter["key"], parameter["kind"], "") for parameter in self._parameters],
            [port for joint in self._joints for port in (
                ScalarPort(f"{joint['key']}Position", "Position", ""),
                ScalarPort(f"{joint['key']}Speed", "Speed", ""),
            ) + ((ScalarPort(f"{joint['key']}ServoTorque", "Torque the servo applies", ""),) if joint["mode"] == "servo" else ())],
        )

    def evaluate(self, context, inputs, outputs):
        start, step = context.simulation_time, context.step_size
        if self._time is None:
            self._time = start
            self._data.time = start
        elif abs(start - self._time) > _SAME_TIME * max(1.0, abs(start)):
            raise RuntimeError(f"Asked to advance from {start} with the mechanism at {self._time}; each step must start where the last ended.")
        self._apply_parameters(inputs, start)
        self._data.qfrc_applied[:] = 0.0
        for joint in self._joints:
            if joint["mode"] == "torque":
                effort = inputs[f"{joint['key']}Effort"]
                if not math.isfinite(effort):
                    raise RuntimeError(f"Joint {joint['key']} was given a torque of {effort} at {start} s.")
                self._data.qfrc_applied[joint["dof"]] = effort
            elif joint["mode"] == "servo":
                setpoint = inputs[f"{joint['key']}Setpoint"]
                if not math.isfinite(setpoint):
                    raise RuntimeError(f"Joint {joint['key']} was given a setpoint of {setpoint} at {start} s.")
                self._data.ctrl[joint["actuator"]] = setpoint
        warnings = {index: self._data.warning[index].number for index in _INSTABILITY_WARNINGS}
        # Whole MuJoCo steps no longer than its own timestep, shortened evenly to land on the step's end.
        count = max(1, math.ceil(step / self._timestep - 1e-9))
        self._model.opt.timestep = step / count
        try:
            for _ in range(count):
                mujoco.mj_step(self._model, self._data)
                for index, quantity in _INSTABILITY_WARNINGS.items():
                    if self._data.warning[index].number != warnings[index]:
                        said = f" MuJoCo: {_warnings[-1].strip()}" if _warnings else ""
                        raise RuntimeError(
                            f"MuJoCo found the mechanism's {quantity} not finite or too large between {start} s and {start + step} s: "
                            "the simulation is unstable. A torque may be too large or change too fast for the step, as a motor's "
                            f"current does when its step is longer than its electrical time constant (inductance over resistance).{said}")
        finally:
            self._model.opt.timestep = self._timestep
        self._time = start + step
        self._data.time = self._time
        for joint in self._joints:
            outputs.add_gradient(f"{joint['key']}Position", float(self._data.qpos[joint["qpos"]]))
            outputs.add_gradient(f"{joint['key']}Speed", float(self._data.qvel[joint["dof"]]))
            if joint["mode"] == "servo":
                outputs.add_gradient(f"{joint['key']}ServoTorque", float(self._data.actuator_force[joint["actuator"]]))

    # A checkpoint holds the time reached and MuJoCo's whole integration state.
    def checkpoint(self):
        if self._time is None:
            return b""
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        array = numpy.zeros(mujoco.mj_stateSize(self._model, spec))
        mujoco.mj_getState(self._model, self._data, array, spec)
        return struct.pack(f"!{1 + len(array)}d", self._time, *array.tolist())

    def restore(self, payload):
        if not payload:
            self._time = None
            return
        values = struct.unpack(f"!{len(payload) // 8}d", payload)
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        array = numpy.array(values[1:])
        if len(array) != mujoco.mj_stateSize(self._model, spec):
            raise ValueError("This checkpoint is of another MuJoCo model.")
        mujoco.mj_setState(self._model, self._data, array, spec)
        mujoco.mj_forward(self._model, self._data)
        self._time = values[0]
