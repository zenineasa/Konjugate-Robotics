# Copyright © 2026 Zenin Easa Panthakkalakath

"""One mechanism, solved by MuJoCo, as a Konjugate computational-node provider.

The mechanism node holds, for each joint, its position and speed, which this provider sets (as algebraic values).
Each joint may read a torque (or, for a slide, a force) from a state: one of the mechanism node's own, or another
node's, such as a motor's, which the engine reads from the step's synchronization snapshot.

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


class MechanismNode(NodeProvider):
    def __init__(self):
        self._configuration = CONFIGURATION
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
            self._joints.append({
                "key": joint["key"],
                "driven": bool(joint.get("driven")),
                "qpos": int(self._model.jnt_qposadr[index]),
                "dof": int(self._model.jnt_dofadr[index]),
            })
            self._data.qpos[self._joints[-1]["qpos"]] = float(joint.get("initialPosition", 0.0))
            self._data.qvel[self._joints[-1]["dof"]] = float(joint.get("initialSpeed", 0.0))
        mujoco.mj_forward(self._model, self._data)
        # The time MuJoCo has reached; None until the first call.
        self._time = None

    def describe(self):
        return NodeProviderDescription(
            self._configuration.get("providerId", "konjugate.robotics.mechanism"),
            self._configuration.get("name", "Mechanism"),
            [ScalarPort(f"{joint['key']}Effort", "Torque or force applied", "") for joint in self._joints if joint["driven"]],
            [port for joint in self._joints for port in (
                ScalarPort(f"{joint['key']}Position", "Position", ""),
                ScalarPort(f"{joint['key']}Speed", "Speed", ""),
            )],
        )

    def evaluate(self, context, inputs, outputs):
        start, step = context.simulation_time, context.step_size
        if self._time is None:
            self._time = start
            self._data.time = start
        elif abs(start - self._time) > _SAME_TIME * max(1.0, abs(start)):
            raise RuntimeError(f"Asked to advance from {start} with the mechanism at {self._time}; each step must start where the last ended.")
        self._data.qfrc_applied[:] = 0.0
        for joint in self._joints:
            if joint["driven"]:
                self._data.qfrc_applied[joint["dof"]] = inputs[f"{joint['key']}Effort"]
        # Whole MuJoCo steps no longer than its own timestep, shortened evenly to land on the step's end.
        count = max(1, math.ceil(step / self._timestep - 1e-9))
        self._model.opt.timestep = step / count
        try:
            for _ in range(count):
                mujoco.mj_step(self._model, self._data)
        finally:
            self._model.opt.timestep = self._timestep
        self._time = start + step
        self._data.time = self._time
        for joint in self._joints:
            outputs.add_gradient(f"{joint['key']}Position", float(self._data.qpos[joint["qpos"]]))
            outputs.add_gradient(f"{joint['key']}Speed", float(self._data.qvel[joint["dof"]]))

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
