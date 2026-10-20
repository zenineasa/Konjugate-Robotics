/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Drawn robots, built as the window builds them (lib/modelBuilder.mjs) and run by Konjugate's engine alone: each solid
// a node, each joint a relationship carrying the articulated-body equations. They move as a second, independent
// solution of the same drawing moves (newtonEuler.mjs: the recursive Newton-Euler method, stepped the same way), to
// within rounding; their live parameters reach the motion; a joint stops at its limit; and a run resumed from a
// checkpoint carries on exactly. MuJoCo run on its own gives a third opinion, where a Python with MuJoCo is at hand.
//
// Needs Konjugate's engine build (see harness.mjs).

import assert from 'node:assert/strict';
import { mkdtemp } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { exampleDrawing, examples } from '../../packages/toolbox/lib/examples.mjs';
import { massOf, reverseJoint, updateJoint, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { konjugateModule } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { simulate } from './newtonEuler.mjs';

const directory = await mkdtemp(join(tmpdir(), 'robotics-drawn-'));
const build = (drawing, runTime = 3) => buildModel(drawing, { timeStep: 0.001, runTime }).document;
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));
const jointNamed = (drawing, name) => drawing.joints.find((joint) => joint.name === name);
const solidNamed = (drawing, name) => drawing.solids.find((solid) => solid.name === name);
const coordinate = (joint) => (joint.kind === 'slider' ? 'jointPosition' : 'jointAngle');
const withoutMujoco = await mujocoMissing();

// Runs `drawing` in the engine and in the Newton-Euler reference; every moving joint and every solid's centre agree.
async function matchesReference(name, drawing, carried, runTime = 3, document = build(drawing, runTime)) {
    const run = await runDocument(directory, name, document);
    const reference = simulate(drawing, run.times);
    for (const [jointName, solidName] of Object.entries(carried)) {
        const joint = jointNamed(drawing, jointName);
        const values = run.series(`${solidName}.${coordinate(joint)}`);
        assert.ok(largestDifference(values, reference[joint.id]) < 1e-9, `${name}: ${jointName} differs from the reference by ${largestDifference(values, reference[joint.id])}.`);
    }
    for (const solid of drawing.solids) {
        for (const [axis, symbol] of ['x', 'y', 'z'].entries()) {
            const values = run.series(`${solid.name}.${symbol}`);
            const expected = reference[solid.id].map((place) => place[axis]);
            assert.ok(largestDifference(values, expected) < 1e-9, `${name}: ${solid.name}'s ${symbol} differs from the reference by ${largestDifference(values, expected)} m.`);
        }
    }
    return run;
}

test('a drawn pendulum swings as the Newton-Euler reference swings it, and keeps its energy', async () => {
    let drawing = exampleDrawing('pendulum');
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0 });
    const run = await matchesReference('pendulum', drawing, { Pivot: 'Arm' }, 4);
    const heights = run.series('Arm.z');
    assert.ok(Math.min(...heights) < solidNamed(drawing, 'Arm').position[2] - 0.05, 'Let go level, the arm should swing down.');
    // Undamped, its energy stays within a tenth of a percent of where it started (the step's own error).
    const kinetic = run.series('Arm.kineticEnergy');
    const potential = run.series('Arm.potentialEnergy');
    const energy = kinetic.map((value, index) => value + potential[index]);
    const swing = Math.max(...kinetic);
    assert.ok(swing > 0.1, 'It should swing.');
    assert.ok(Math.max(...energy) - Math.min(...energy) < 0.01 * swing, `Its energy wandered by ${Math.max(...energy) - Math.min(...energy)} J.`);
});

test('a joint\'s torque, a live parameter of its edge, reaches the joint: m g L holds the arm level', async () => {
    let drawing = exampleDrawing('pendulum');
    const arm = solidNamed(drawing, 'Arm');
    // The arm's middle is 0.1 m out along x from the hinge, whose axis is y: gravity turns it about +y by m g 0.1.
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0, drive: { mode: 'torque', torque: -massOf(arm) * 9.81 * 0.1 } });
    const angles = (await runDocument(directory, 'held', build(drawing, 1))).series('Arm.jointAngle');
    assert.ok(Math.max(...angles.map(Math.abs)) < 1e-9, `Held by its torque, it moved ${Math.max(...angles.map(Math.abs))} rad.`);
});

test('servos hold a drawn two-link arm at its setpoints, moving as the reference moves it', async () => {
    const drawing = exampleDrawing('twoLinkArm');
    const run = await matchesReference('arm', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' });
    const elbow = run.series('Forearm.jointAngle').at(-1);
    assert.ok(Math.abs(elbow - 0.5) < 0.01, `The elbow should settle near its 0.5 rad setpoint, not ${elbow}.`);
});

test('a cart on a slider carries two pendulums, a branch of the tree each, as the reference does', async () => {
    const drawing = exampleDrawing('cartPendulums');
    const carried = { Track: 'Cart', 'Front swing': 'Front pendulum', 'Back swing': 'Back pendulum' };
    const run = await matchesReference('cart', drawing, carried);
    const back = run.series('Back pendulum.jointAngle');
    assert.ok(Math.max(...back.map(Math.abs)) > 0.1, 'The cart starting off should set the hanging pendulum swinging.');
    // The front swing joint read the other way round (its point and axis on the pendulum): the same robot, the same motion.
    const reversed = reverseJoint(drawing, jointNamed(drawing, 'Front swing').id);
    const again = await matchesReference('cartReversed', reversed, carried);
    assert.ok(largestDifference(again.series('Front pendulum.x'), run.series('Front pendulum.x')) < 1e-12);
});

test('an arm with a gripper turns at its waist and closes its fingers, held up against gravity, as the reference moves it', async () => {
    const drawing = exampleDrawing('gripperArm');
    const carried = {
        Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm', Wrist: 'Hand', 'Left finger slide': 'Left finger', 'Right finger slide': 'Right finger'
    };
    const run = await matchesReference('gripper', drawing, carried);
    const waist = run.series('Turret.jointAngle').at(-1);
    assert.ok(Math.abs(waist - 0.8) < 0.02, `The waist should turn to its 0.8 rad setpoint, not ${waist}.`);
    for (const [joint, solid] of [['Shoulder', 'Upper arm'], ['Elbow', 'Forearm'], ['Wrist', 'Hand']]) {
        const sag = Math.max(...run.series(`${solid}.jointAngle`).map(Math.abs));
        assert.ok(sag < 0.05, `The ${joint} servo should hold the arm where it is drawn against gravity, but it gave ${sag} rad.`);
    }
    for (const finger of ['Left finger', 'Right finger']) {
        const travel = run.series(`${finger}.jointPosition`).at(-1);
        assert.ok(Math.abs(travel + 0.015) < 0.002, `The ${finger.toLowerCase()} should close by 15 mm, not ${travel * 1000} mm.`);
    }
    const gap = (index) => Math.hypot(...['x', 'y', 'z'].map((axis) => run.series(`Left finger.${axis}`).at(index) - run.series(`Right finger.${axis}`).at(index)));
    assert.ok(gap(-1) < gap(0) - 0.025, `The fingers should close towards each other, from ${gap(0)} m apart to ${gap(-1)} m.`);
});

test('friction, armature, a torque drive and every shape reach the motion as drawn', async () => {
    let drawing = exampleDrawing('twoLinkArm');
    drawing = updateJoint(drawing, jointNamed(drawing, 'Waist').id, { friction: 0.01, armature: 0.002, drive: { mode: 'free' } });
    drawing = updateJoint(drawing, jointNamed(drawing, 'Shoulder').id, { friction: 0.02, armature: 0.001, limits: null, drive: { mode: 'torque', torque: 0.3 } });
    drawing = updateSolid(drawing, solidNamed(drawing, 'Forearm').id, { shape: 'sphere', size: { radius: 0.03 } });
    drawing = updateSolid(drawing, solidNamed(drawing, 'Turret').id, { shape: 'capsule', size: { radius: 0.03, length: 0.03 } });
    await matchesReference('mixed', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' }, 2);
});

test('a joint\'s damping and a solid\'s mass, parameters in the canvas, reach the motion as if drawn', async () => {
    const drawing = exampleDrawing('pendulum');
    const pivot = jointNamed(drawing, 'Pivot');
    const arm = solidNamed(drawing, 'Arm');
    const document = build(drawing, 2);
    // Change the edge's damping and the arm's shared mass in the model, as a slider or a fit would, not the drawing.
    document.edges.find((edge) => edge.name === 'Pivot').parameters.find(({ symbol }) => symbol === 'damping').value = 0.05;
    document.sharedParameters.find(({ name }) => name === 'Arm mass').value = 2 * massOf(arm);
    let drawn = updateJoint(drawing, pivot.id, { damping: 0.05 });
    drawn = updateSolid(drawn, arm.id, { mass: { by: 'mass', value: 2 * massOf(arm) } });
    await matchesReference('changed', drawn, { Pivot: 'Arm' }, 2, document);
});

test('a joint stops at its limit', async () => {
    let drawing = exampleDrawing('pendulum');
    // Let go level, the arm swings down to positive angles; its limit stops it at 0.4 rad.
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0.002, limits: { lower: -0.4, upper: 0.4 } });
    const angles = (await runDocument(directory, 'limited', build(drawing, 2))).series('Arm.jointAngle');
    assert.ok(Math.max(...angles) < 0.45, `It passed its limit, to ${Math.max(...angles)} rad.`);
    assert.ok(Math.abs(angles.at(-1) - 0.4) < 0.02, `It should rest against its limit, not at ${angles.at(-1)} rad.`);
});

test('a drawn robot resumed from a checkpoint carries on exactly as the run that never stopped', async () => {
    const drawing = exampleDrawing('cartPendulums');
    const whole = await runDocument(directory, 'whole', build(drawing, 2));
    const checkpoint = whole.result.checkpoints.find((candidate) => Math.abs(candidate.time - 1) < 1e-9);
    assert.ok(checkpoint, 'The run kept no checkpoint at 1 s.');
    const resumed = await runDocument(directory, 'resumed', build(drawing, 2), { startCheckpoint: checkpoint });
    for (const key of ['Cart.jointPosition', 'Front pendulum.jointAngle', 'Back pendulum.yaw']) {
        const original = new Map(whole.times.map((time, index) => [time.toFixed(6), whole.series(key)[index]]));
        resumed.times.forEach((time, index) => assert.equal(resumed.series(key)[index], original.get(time.toFixed(6)), `${key} at ${time} s.`));
    }
});

// MuJoCo steps the same way (its Euler integrator: speeds, then positions from the new speeds) but takes a joint's
// damping implicitly and models its servos as position actuators, so the two agree closely, not to rounding. The bound
// below is a finding, stated: Konjugate and MuJoCo agree to within it over these runs.
const mujocoBound = 1e-2;
test('MuJoCo on its own swings the same pendulum and holds the same arm, to within a stated bound', { skip: withoutMujoco ?? false }, async () => {
    for (const id of ['pendulum', 'twoLinkArm', 'cartPendulums']) {
        const drawing = exampleDrawing(id);
        const run = await runDocument(directory, `mujoco-${id}`, build(drawing, 3));
        const moving = drawing.joints.filter(({ kind }) => kind === 'hinge' || kind === 'slider');
        const controls = Object.fromEntries(moving.filter(({ drive }) => drive?.mode === 'servo').map(({ id: jointId, drive }) => [jointId, drive.setpoint]));
        const reference = await runReference(directory, { mjcf: writeMjcf(drawing), joints: moving.map(({ id: jointId }) => ({ name: jointId })), controls, times: run.times });
        for (const joint of moving) {
            const solidId = drawing.frames.find(({ id: frameId }) => frameId === joint.child).solid;
            const values = run.series(`${drawing.solids.find(({ id: candidate }) => candidate === solidId).name}.${coordinate(joint)}`);
            const difference = largestDifference(values, reference[joint.id]);
            console.log(`${id} ${joint.name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
            assert.ok(difference < mujocoBound, `${id}: ${joint.name} differs from MuJoCo alone by ${difference}.`);
        }
    }
});

// Every equation Build writes can be edited by a user in the canvas: its LaTeX reads back, in Konjugate's own parser, as
// an expression giving the same value as the MathJSON the engine runs.
test('every equation a build writes reads back in the canvas as the same expression', async () => {
    const { validateEquationLatex } = await import(pathToFileURL(konjugateModule('src/equationModel.mjs')));
    const evaluate = (expression, values) => {
        if (typeof expression === 'number') return expression;
        if (typeof expression === 'string') return expression === 'True' ? 1 : values.get(expression);
        const [operator, ...operands] = expression;
        if (operator === 'Which') {
            for (let index = 0; index < operands.length; index += 2) if (evaluate(operands[index], values)) return evaluate(operands[index + 1], values);
            return Number.NaN;
        }
        const v = operands.map((operand) => evaluate(operand, values));
        return {
            Add: () => v.reduce((a, b) => a + b, 0), Multiply: () => v.reduce((a, b) => a * b, 1), Negate: () => -v[0], Subtract: () => v[0] - v[1],
            Divide: () => v[0] / v[1], Power: () => v[0] ** v[1], Sqrt: () => Math.sqrt(v[0]), Sin: () => Math.sin(v[0]), Cos: () => Math.cos(v[0]),
            Tanh: () => Math.tanh(v[0]), Arcsin: () => Math.asin(v[0]), Arctan2: () => Math.atan2(v[0], v[1]), Min: () => Math.min(...v), Max: () => Math.max(...v),
            Less: () => Number(v[0] < v[1]), Greater: () => Number(v[0] > v[1]), LessEqual: () => Number(v[0] <= v[1]), GreaterEqual: () => Number(v[0] >= v[1])
        }[operator]();
    };
    let seed = 3;
    const random = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) * 1.6 - 0.8;
    for (const { id } of examples) {
        for (const edge of buildModel(exampleDrawing(id)).document.edges) {
            const { bindings, definitions, outputs } = edge.equationModel;
            const readable = bindings.map(({ symbol }) => ({ symbol }));
            const values = new Map(bindings.map(({ symbol }) => [symbol, random()]));
            for (const item of [...definitions, ...outputs]) {
                const validation = validateEquationLatex(item.latex, readable);
                assert.equal(validation.valid, true, `${id}, ${edge.name}: ${item.latex} ${validation.errors.join(' ')}`);
                const expected = evaluate(item.mathJson, values);
                const actual = evaluate(validation.mathJson, values);
                assert.ok(Math.abs(expected - actual) <= 1e-9 * Math.max(1, Math.abs(expected)), `${id}, ${edge.name}: ${item.latex} gives ${actual}, not ${expected}.`);
                if (item.symbol) { readable.push({ symbol: item.symbol }); values.set(item.symbol, expected); }
            }
        }
    }
});
