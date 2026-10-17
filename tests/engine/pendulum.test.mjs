/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A pendulum solved by MuJoCo inside Konjugate's engine, through the mechanism provider: it swings as MuJoCo alone
// swings it, with the period the exact (large-swing) solution gives, and a torque from a shared parameter holds it level.

import assert from 'node:assert/strict';
import { mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { buildMechanismDocument } from '../../packages/toolbox/lib/mechanismModel.mjs';
import { helpers, providerTemplate, runDocument, runReference } from './harness.mjs';

const gravity = 9.81;
const length = 1;
const mass = 1;
const radius = 0.01;
// The bob is a solid sphere on a massless rod: its inertia about the hinge.
const inertia = mass * length ** 2 + 0.4 * mass * radius ** 2;

const mjcf = `<mujoco>
  <option timestep="0.001" integrator="RK4" gravity="0 0 -${gravity}"/>
  <worldbody>
    <body name="bob">
      <joint name="swing" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="${radius}" pos="0 0 -${length}" mass="${mass}" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>`;

// The complete elliptic integral of the first kind, by the arithmetic-geometric mean.
function ellipticK(k) {
    let [a, b] = [1, Math.sqrt(1 - k * k)];
    while (Math.abs(a - b) > 1e-15) [a, b] = [(a + b) / 2, Math.sqrt(a * b)];
    return Math.PI / (2 * a);
}

// Times the series falls through zero, linearly interpolated between samples.
function downwardCrossings(times, values) {
    const crossings = [];
    for (let index = 1; index < values.length; index += 1) {
        if (values[index - 1] > 0 && values[index] <= 0) {
            const fraction = values[index - 1] / (values[index - 1] - values[index]);
            crossings.push(times[index - 1] + fraction * (times[index] - times[index - 1]));
        }
    }
    return crossings;
}

const directory = await mkdtemp(join(tmpdir(), 'robotics-pendulum-'));
const build = (joint, targetTime = 10, parameters = []) => buildMechanismDocument(
    { name: 'Pendulum', mjcf, joints: [{ name: 'swing', ...joint }], parameters },
    { template: providerTemplate, helpers, globalTimeStep: 0.01, targetTime }
);

test('a pendulum swings in Konjugate exactly as MuJoCo alone swings it', async () => {
    const amplitude = 0.8;
    const run = await runDocument(directory, 'swing', build({ initialPosition: amplitude }));
    const positions = run.series('Pendulum.swingPosition');
    assert.equal(positions[0], amplitude);
    const reference = await runReference(directory, { mjcf, joints: [{ name: 'swing', initialPosition: amplitude }], times: run.times });
    const largest = Math.max(...positions.map((value, index) => Math.abs(value - reference.swing[index])));
    assert.ok(largest < 1e-9, `Konjugate and MuJoCo alone differ by up to ${largest} rad.`);
});

test('the pendulum swings with the exact large-swing period', async () => {
    const amplitude = 0.8;
    const run = await runDocument(directory, 'period', build({ initialPosition: amplitude }));
    const crossings = downwardCrossings(run.times, run.series('Pendulum.swingPosition'));
    assert.ok(crossings.length >= 4, `Only ${crossings.length} swings in the run.`);
    const period = (crossings.at(-1) - crossings[0]) / (crossings.length - 1);
    const exact = 4 * Math.sqrt(inertia / (mass * gravity * length)) * ellipticK(Math.sin(amplitude / 2));
    assert.ok(Math.abs(period - exact) / exact < 1e-4, `Period ${period} s, exact ${exact} s.`);
});

test('a torque from a shared parameter reaches the joint: m g L holds the pendulum level', async () => {
    const level = Math.PI / 2;
    const run = await runDocument(directory, 'held', build({ initialPosition: level, torque: { value: mass * gravity * length } }, 2));
    const positions = run.series('Pendulum.swingPosition');
    const largest = Math.max(...positions.map((value) => Math.abs(value - level)));
    assert.ok(largest < 1e-9, `It moved ${largest} rad from level.`);
    // Without it, the same pendulum falls.
    const fallen = (await runDocument(directory, 'fallen', build({ initialPosition: level }, 2))).series('Pendulum.swingPosition');
    assert.ok(Math.min(...fallen) < 0, 'Without the torque it should swing through the bottom.');
});

// A torque read from another node's state, as a motor's would be: through the step's snapshot, at once.
test('a torque from another node\'s state reaches the joint: a drive node holding m g L keeps it level', async () => {
    const level = Math.PI / 2;
    const [driveId, torqueId] = [1, 2];
    const document = buildMechanismDocument(
        { name: 'Pendulum', mjcf, joints: [{ name: 'swing', initialPosition: level, torqueFrom: { nodeId: driveId, stateId: torqueId } }] },
        { template: providerTemplate, helpers, globalTimeStep: 0.01, targetTime: 2, firstId: 3 }
    );
    document.nodes.unshift({
        id: driveId, name: 'Drive', type: 'drive', position: [0, 0, 0], sourceTerms: [], numerics: { substepsPerGlobalStep: 1 }, appearance: {},
        states: [{ id: torqueId, name: 'Torque', symbol: 'torque', initialValue: mass * gravity * length, unit: 'N·m' }]
    });
    const positions = (await runDocument(directory, 'drive', document)).series('Pendulum.swingPosition');
    const largest = Math.max(...positions.map((value) => Math.abs(value - level)));
    assert.ok(largest < 1e-9, `It moved ${largest} rad from level.`);
});

// Forks and restarts resume from a checkpoint: MuJoCo's whole state goes with it, so the run carries on exactly.
test('a run resumed from a checkpoint carries on exactly as the run that never stopped', async () => {
    const whole = await runDocument(directory, 'whole', build({ initialPosition: 0.8 }, 2));
    const checkpoint = whole.result.checkpoints.find((candidate) => Math.abs(candidate.time - 1) < 1e-9);
    assert.ok(checkpoint, 'The run kept no checkpoint at 1 s.');
    const resumed = await runDocument(directory, 'resumed', build({ initialPosition: 0.8 }, 2), { startCheckpoint: checkpoint });
    const original = new Map(whole.times.map((time, index) => [time.toFixed(6), whole.series('Pendulum.swingPosition')[index]]));
    const positions = resumed.series('Pendulum.swingPosition');
    assert.ok(resumed.times.at(-1) > 1.99 && resumed.times[0] >= 1 - 1e-9, `The resumed run covers ${resumed.times[0]} to ${resumed.times.at(-1)} s.`);
    resumed.times.forEach((time, index) => assert.equal(positions[index], original.get(time.toFixed(6)), `At ${time} s.`));
});

// A node has no parameters of its own; the mechanism reads shared ones, each setting a quantity of MuJoCo's model.
test('a damping parameter reaches the joint: the pendulum swings as one with that damping in its MJCF', async () => {
    const damping = 0.3;
    const run = await runDocument(directory, 'damped', build({ initialPosition: 0.8 }, 3, [{ name: 'swing damping', kind: 'jointDamping', target: 'swing', value: damping }]));
    const dampedMjcf = mjcf.replace('<joint name="swing" type="hinge" axis="0 1 0"/>', `<joint name="swing" type="hinge" axis="0 1 0" damping="${damping}"/>`);
    assert.notEqual(dampedMjcf, mjcf);
    const reference = await runReference(directory, { mjcf: dampedMjcf, joints: [{ name: 'swing', initialPosition: 0.8 }], times: run.times });
    const positions = run.series('Pendulum.swingPosition');
    const largest = Math.max(...positions.map((value, index) => Math.abs(value - reference.swing[index])));
    assert.ok(largest < 1e-9, `Konjugate and MuJoCo alone differ by up to ${largest} rad.`);
    assert.ok(Math.abs(positions.at(-1)) < 0.5, 'The damped swing should have died down.');
});

// A stored schedule on the shared parameter changes the mass mid-run, as a fork's intervention or a slider would.
test('a payload mass scheduled to double reaches the body: held level by m g L, the pendulum then falls as a heavier one does', async () => {
    const level = Math.PI / 2;
    const document = build({ initialPosition: level, torque: { value: mass * gravity * length } }, 2, [
        { name: 'bob mass', kind: 'bodyMass', target: 'bob', value: mass, schedule: { interpolation: 'hold', samples: [[0, mass], [0.995, 2 * mass]] } }
    ]);
    const run = await runDocument(directory, 'payload', document);
    const positions = run.series('Pendulum.swingPosition');
    const change = run.times.findIndex((time) => time > 0.995);
    const before = Math.max(...positions.slice(0, change + 1).map((value) => Math.abs(value - level)));
    assert.ok(before < 1e-9, `Before the change it moved ${before} rad from level.`);
    // From the change on, as a pendulum of twice the mass under the same torque, from rest at level.
    const heavier = mjcf.replace(`mass="${mass}"`, `mass="${2 * mass}"`);
    assert.notEqual(heavier, mjcf);
    const after = run.times.slice(change).map((time) => time - run.times[change]);
    const reference = await runReference(directory, { mjcf: heavier, joints: [{ name: 'swing', initialPosition: level }], forces: { swing: mass * gravity * length }, times: after });
    const largest = Math.max(...positions.slice(change).map((value, index) => Math.abs(value - reference.swing[index])));
    assert.ok(largest < 1e-9, `After the change Konjugate and MuJoCo alone differ by up to ${largest} rad.`);
    assert.ok(positions.at(-1) < level - 0.5, 'The heavier pendulum should have fallen.');
});
