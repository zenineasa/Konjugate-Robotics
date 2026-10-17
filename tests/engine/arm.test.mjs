/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A two-link arm solved by MuJoCo inside Konjugate's engine, each joint driven by a joint motor (current, torque,
// winding temperature) from one battery: the motors read their joints' speeds, the mechanism reads the motors'
// torques, each through the step's snapshot. It runs as MuJoCo alone runs it with the same motor equations
// integrated beside it (armReference.py): joint angles, currents, winding temperatures and the battery's charge.
// A small motor, whose electrical time constant is shorter than the step, runs in substeps of its own; forced into
// one, the run stops with MuJoCo's report of instability rather than carrying on reset.

import assert from 'node:assert/strict';
import { mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { DocumentBuilder } from '../../packages/toolbox/lib/documentBuilder.mjs';
import { connectJointDrive, motorSubsteps, placeJointMotor } from '../../packages/toolbox/lib/driveModel.mjs';
import { addRunConfiguration, emptyDocument, placeMechanism } from '../../packages/toolbox/lib/mechanismModel.mjs';
import { helpers, loadTemplates, providerTemplate, runDocument, runReference } from './harness.mjs';

// Rotor inertia, reflected through the gearbox into MuJoCo's joint armature (the motor's mechanical side lives there).
const rotorInertia = 1e-6;
const gearRatio = 50;
const armature = rotorInertia * gearRatio ** 2;

const mjcf = `<mujoco>
  <option timestep="0.001" integrator="RK4" gravity="0 0 -9.81"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="upper">
      <joint name="shoulder" type="hinge" axis="0 1 0" armature="${armature}"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
      <body name="lower" pos="0.3 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0" armature="${armature}"/>
        <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.015" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>`;

const voltage = 24;
const capacity = 7200;
const thermal = { thermalResistance: 8, thermalCapacitance: 20, ambientTemperature: 293.15 };
// A winding of 42 ms, and a small motor's of 0.42 ms, shorter than the 1 ms step.
const slowWinding = { resistance: 1.2, inductance: 0.05, torqueConstant: 0.05, gearRatio };
const fastWinding = { ...slowWinding, inductance: 0.0005 };
// A positive turn about y carries +x towards -z, so lifting the arm against gravity takes a negative torque.
const duties = { shoulder: -0.25, elbow: 0.15 };
const step = 0.001;
const targetTime = 1;

// The arm, its two motors and their battery, placed as the app would place them. `substeps` forces the motors'.
async function armDocument(winding, { substeps } = {}) {
    const templates = await loadTemplates(['battery']);
    const document = emptyDocument();
    const builder = new DocumentBuilder(document, templates, helpers);
    const battery = builder.placeNode('battery', { name: 'Battery', initialValues: { voltage } });
    const motors = Object.fromEntries(Object.entries(duties).map(([joint, duty]) => {
        const values = { ...winding, duty };
        return [joint, { values, ...placeJointMotor(builder, { name: `${joint} motor`, values, step }) }];
    }));
    if (substeps) for (const { node } of Object.values(motors)) node.numerics.substepsPerGlobalStep = substeps;
    const { node: arm, joints } = placeMechanism(document, () => builder.id(), {
        name: 'Arm', mjcf, joints: Object.keys(duties).map((joint) => ({ name: joint, torqueFrom: { nodeId: motors[joint].node.id, stateId: motors[joint].torque.id } }))
    }, { template: providerTemplate, helpers });
    for (const [joint, motor] of Object.entries(motors)) {
        connectJointDrive(builder, { battery, motor, mechanism: arm, speed: joints.get(joint).speed, name: joint, values: motor.values });
    }
    // The reference is given the template's battery capacity; check it is what the drive has.
    for (const edge of document.edges.filter(({ name }) => name.includes('Drain'))) assert.equal(edge.parameters.find(({ symbol }) => symbol === 'capacity').value, capacity);
    addRunConfiguration(document, () => builder.id(), { globalTimeStep: step, targetTime });
    return { document, substeps: motors.shoulder.node.numerics.substepsPerGlobalStep };
}

async function compareWithReference(name, winding) {
    const { document, substeps } = await armDocument(winding);
    const run = await runDocument(directory, name, document);
    const reference = await runReference(directory, {
        mjcf, voltage, capacity, step, steps: Math.round(targetTime / step),
        joints: Object.keys(duties).map((joint) => ({ name: joint, initialPosition: 0 })),
        motors: Object.entries(duties).map(([joint, duty]) => ({ joint, duty, ...winding, ...thermal, substeps }))
    }, 'armReference.py');
    assert.equal(run.times.length, reference.charge.length);
    const largest = (actual, expected) => Math.max(...actual.map((value, index) => Math.abs(value - expected[index])));
    for (const joint of Object.keys(duties)) {
        const position = largest(run.series(`Arm.${joint}Position`), reference[joint].position);
        const current = largest(run.series(`${joint} motor.current`), reference[joint].current);
        const temperature = largest(run.series(`${joint} motor.windingTemperature`), reference[joint].temperature);
        assert.ok(position < 1e-9, `${joint}: angles differ by up to ${position} rad.`);
        assert.ok(current < 1e-9, `${joint}: currents differ by up to ${current} A.`);
        assert.ok(temperature < 1e-9, `${joint}: winding temperatures differ by up to ${temperature} K.`);
    }
    const charge = largest(run.series('Battery.stateOfCharge'), reference.charge);
    assert.ok(charge < 1e-9, `The battery's charge differs by up to ${charge} %.`);
    // And the arm did something: the shoulder lifted it against gravity, and the winding warmed.
    assert.ok(run.series('Arm.shoulderPosition').at(-1) < -1, 'The shoulder should have lifted the arm.');
    assert.ok(run.series('shoulder motor.windingTemperature').at(-1) > thermal.ambientTemperature + 0.05, 'The shoulder winding should have warmed.');
    return { run, substeps };
}

const directory = await mkdtemp(join(tmpdir(), 'robotics-arm-'));

test('a two-link arm driven by joint motors from a battery runs in Konjugate as MuJoCo alone runs it with the same motors', async () => {
    const { substeps } = await compareWithReference('arm', slowWinding);
    assert.equal(substeps, 1, 'A winding of 42 ms needs no substeps of a 1 ms step.');
});

test('a small motor, faster than the step, runs in substeps of its own and still matches', async () => {
    const { substeps } = await compareWithReference('fastArm', fastWinding);
    assert.equal(substeps, motorSubsteps(fastWinding, step));
    assert.equal(substeps, 24, 'A winding of 0.42 ms should take 24 substeps of a 1 ms step, a tenth of its time constant each.');
});

test('forced into one substep, a small motor\'s current diverges, and the run stops saying so rather than resetting', async () => {
    const { document } = await armDocument(fastWinding, { substeps: 1 });
    const { errors } = await runDocument(directory, 'unstableArm', document, { expectFailure: true });
    assert.match(errors, /the simulation is unstable/);
    assert.match(errors, /inductance over resistance/);
    // MuJoCo's own words, which the provider keeps rather than leaving them in a log file wherever the engine runs.
    assert.match(errors, /MuJoCo: Nan, Inf or huge value in QACC/);
});
