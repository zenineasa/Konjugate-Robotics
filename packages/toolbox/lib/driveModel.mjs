/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Joint motors and their drives (packages/engine/components/jointMotor.json and jointDrive.json), placed with a
// DocumentBuilder. A motor is placed before its mechanism, which reads the motor's torque; its drive after, since it
// reads the joint's speed.
//
// The engine integrates a motor's current with explicit Euler, which is stable only for a step shorter than twice the
// winding's electrical time constant (inductance over resistance), and accurate only for one well inside it. A small
// motor's is under a millisecond. So a motor takes substeps of its own, a tenth of its time constant or less, while the
// mechanism and the rest of the model keep their step: core runs each node at its own rate.

// Substeps per global step of `step` seconds for a winding of `inductance` H and `resistance` Ω.
export function motorSubsteps({ inductance, resistance }, step, perTimeConstant = 10) {
    if (!(inductance > 0) || !(resistance > 0)) throw new Error('A winding needs a positive inductance and resistance.');
    const substeps = Math.max(1, Math.ceil(step * perTimeConstant * resistance / inductance - 1e-9));
    if (substeps > 10000) throw new Error(`A winding of ${inductance} H and ${resistance} Ω needs ${substeps} substeps of a ${step} s step; core allows 10000. Shorten the step.`);
    return substeps;
}

// The template's value of a shared parameter, unless `values` gives one.
function valueOf(builder, key, values) {
    if (values[key] !== undefined) return values[key];
    for (const id of ['jointMotor', 'jointDrive']) {
        const declared = builder.template(id, id === 'jointMotor' ? 'node' : 'bundle').sharedParameters.find((parameter) => parameter.key === key);
        if (declared) return declared.value;
    }
    throw new Error(`No joint motor parameter "${key}".`);
}

// A joint's motor, with `values` overriding the motor's and drive's parameters by key (resistance, inductance,
// torqueConstant, gearRatio, duty). Returns { node, torque, scope }: the mechanism reads `torque`, and the drive
// placed later shares `scope`'s parameters.
export function placeJointMotor(builder, { name, values = {}, step }) {
    const scope = new Map();
    const substeps = motorSubsteps({ inductance: valueOf(builder, 'inductance', values), resistance: valueOf(builder, 'resistance', values) }, step);
    const node = builder.placeNode('jointMotor', { name, values, scope, substeps });
    return { node, torque: node.states.find(({ symbol }) => symbol === 'torque'), scope };
}

// The drive between a battery, a motor placed by placeJointMotor and its joint's speed state on `mechanism`.
export function connectJointDrive(builder, { battery, motor, mechanism, speed, name, values = {} }) {
    return builder.applyBundle('jointDrive', { battery, motor: motor.node, joint: { node: mechanism, ports: { speed: speed.id } } }, { scope: motor.scope, name, values });
}

// A servo's winding and its supply (packages/engine/components/servoWinding.json and servoSupply.json): the winding
// heats with the current a servo joint reports (`current`, its state on `mechanism`), and the battery loses it.
// `values` override the winding's and the supply's parameters by key (resistance, thermalCapacitance,
// thermalResistance, ambientTemperature, capacity).
export function placeServoSupply(builder, { battery, mechanism, current, name, values = {} }) {
    const scope = new Map();
    const winding = builder.placeNode('servoWinding', { name: `${name} winding`, values, scope });
    builder.applyBundle('servoSupply', { battery, winding, joint: { node: mechanism, ports: { current: current.id } } }, { scope, name, values });
    return { winding };
}
