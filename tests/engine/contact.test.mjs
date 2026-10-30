/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Contact (lib/contact.mjs, lib/collision.mjs, and lib/modelBuilder.mjs's contact relationships), built into Konjugate
// and run by its engine: against the Newton-Euler reference, which finds the same rows its own way, and against MuJoCo
// running the drawing written back as MJCF (its contacts written as <pair>s with the parameters MuJoCo would mix).
// - A body free to move in a vertical plane (tests/fixtures/contact/planar.xml) dropped tilted onto the floor, as a box,
//   a capsule, a cylinder and a sphere; and as a capsule and a sphere onto a box fixed beside it.
// - A solid joined to nothing tumbling onto the floor: its rows as the reference's; MuJoCo turns a free joint by its
//   step's exact rotation, which this does not yet, so the two part by up to a few tenths of a millimetre over 2 s.
// - A two-finger gripper lifting a box off the floor by friction alone (tests/fixtures/contact/gripper.xml).

import assert from 'node:assert/strict';
import { mkdtemp, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { addSolid, checkDrawing, createDrawing, updateJoint, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { contactPairs, setTouching, updateContact } from '../../packages/toolbox/lib/contact.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { simulate } from './newtonEuler.mjs';

const directory = await mkdtemp(join(tmpdir(), 'robotics-contact-'));
const withoutMujoco = await mujocoMissing();
const fixture = async (name) => {
    const model = readModel({ [name]: await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', name), 'utf8') }, name);
    return importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name }).drawing;
};
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));
const solidNamed = (drawing, name) => drawing.solids.find((solid) => solid.name === name);
const moving = (drawing) => drawing.joints.filter(({ kind }) => kind === 'hinge' || kind === 'slider');
const seriesOf = (run, drawing, joint) => {
    const solid = drawing.solids.find(({ id }) => id === drawing.frames.find((frame) => frame.id === joint.child).solid);
    return run.series(`${solid.name}.${joint.kind === 'slider' ? 'jointPosition' : 'jointAngle'}`);
};

// Runs `drawing`; checks each joint against the reference to `bound`, and against MuJoCo running it written back
// (with `controls` for its servos) to `mujocoBound`.
async function check(key, drawing, runTime, { bound = 1e-9, mujocoBound = 1e-9, controls = {} } = {}) {
    assert.deepEqual(checkDrawing(drawing).errors, []);
    const run = await runDocument(directory, key, buildModel(drawing, { timeStep: 0.001, runTime }).document);
    const reference = simulate(drawing, run.times);
    for (const joint of moving(drawing)) {
        const difference = largestDifference(seriesOf(run, drawing, joint), reference[joint.id]);
        assert.ok(difference < bound, `${key}: ${joint.name} differs from the reference by ${difference}.`);
    }
    if (!withoutMujoco) {
        const mujoco = await runReference(directory, { mjcf: writeMjcf(drawing), joints: moving(drawing).map(({ id }) => ({ name: id })), controls, times: run.times });
        for (const joint of moving(drawing)) {
            const difference = largestDifference(seriesOf(run, drawing, joint), mujoco[joint.id]);
            console.log(`${key}, ${joint.name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
            assert.ok(difference < mujocoBound, `${key}: ${joint.name} differs from MuJoCo by ${difference}.`);
        }
    }
    return { run, reference };
}

const planar = await fixture('planar.xml');
const shapes = { box: { x: 0.2, y: 0.16, z: 0.1 }, capsule: { radius: 0.05, length: 0.2 }, cylinder: { radius: 0.05, length: 0.16 }, sphere: { radius: 0.05 } };
const dropping = (shape, { step = false, position = [0, 0, 0] } = {}) => {
    let drawing = updateSolid(planar, solidNamed(planar, 'Body').id, { colliders: [{ shape, size: shapes[shape], position }] });
    if (step) drawing = updateSolid(drawing, solidNamed(planar, 'Step').id, { colliders: [{ shape: 'box', size: { x: 0.1, y: 0.2, z: 0.08 } }] });
    return updateContact(drawing, {});
};

for (const shape of Object.keys(shapes)) {
    test(`a ${shape} dropped tilted onto the floor lands, tips and settles as the reference and MuJoCo have it`, async () => {
        const drawing = dropping(shape);
        assert.deepEqual(contactPairs(drawing).pairs.map(({ name }) => name), ['Floor and Body']);
        const { run } = await check(`drop-${shape}`, drawing, 2);
        // It comes down and stays on the floor, pressed on it.
        const up = seriesOf(run, drawing, moving(drawing).find(({ name }) => name === 'Up'));
        assert.ok(up.at(-1) < -0.1, 'It should fall.');
        assert.ok(run.series('Body.floorAndBodyForce1').some((force) => force > 1), 'The floor should push it.');
    });
}

for (const shape of ['capsule', 'sphere']) {
    test(`a ${shape} falling onto a box fixed beside it, and onto the floor, as the reference and MuJoCo have it`, async () => {
        // The sphere off the body's centre, so it comes down on the step.
        const drawing = dropping(shape, { step: true, position: shape === 'sphere' ? [0.1, 0, 0] : [0, 0, 0] });
        // The step, fixed to the Ground, does not touch the floor.
        assert.deepEqual(contactPairs(drawing).pairs.map(({ name }) => name), ['Floor and Body', `Body and Step`]);
        const { run } = await check(`step-${shape}`, drawing, 2);
        assert.ok(run.series('Step.bodyAndStepForce1').some((force) => force > 0.1), 'The step should push it.');
    });
}

test('a solid joined to nothing tumbles onto the floor as the reference and MuJoCo have it, its turn stepped by its exact rotation', async () => {
    for (const shape of ['box', 'capsule']) {
        let drawing = createDrawing('Tumble');
        const added = addSolid(drawing, shape);
        drawing = updateContact(updateSolid(added.drawing, added.id, { position: [0, 0, 0.3], rotation: [0.9, 0.3, 0.2, 0.1] }), {});
        const run = await runDocument(directory, `tumble-${shape}`, buildModel(drawing, { timeStep: 0.001, runTime: 2 }).document);
        const reference = simulate(drawing, run.times);
        const solid = drawing.solids[0];
        for (const [axis, symbol] of ['x', 'y', 'z'].entries()) {
            const difference = largestDifference(run.series(`${solid.name}.${symbol}`), reference[solid.id].map((place) => place[axis]));
            assert.ok(difference < 1e-9, `${shape}'s ${symbol} differs from the reference by ${difference}.`);
        }
        assert.ok(run.series(`${solid.name}.z`).at(-1) < 0.1, 'It should come to rest on the floor.');
        if (withoutMujoco) continue;
        const mujoco = await runReference(directory, { mjcf: writeMjcf(drawing), joints: [], controls: {}, bodies: [solid.id], times: run.times });
        const place = Math.max(...['x', 'y', 'z'].map((symbol, axis) => largestDifference(run.series(`${solid.name}.${symbol}`), mujoco[solid.id].map((at) => at[axis]))));
        console.log(`tumble-${shape}: Konjugate and MuJoCo place it within ${place.toExponential(2)} m.`);
        assert.ok(place < 1e-9, `${shape}: ${place} m from where MuJoCo has it.`);
    }
});

test('a two-finger gripper lifts a box off the floor by friction alone, as the reference and MuJoCo have it', async () => {
    let drawing = await fixture('gripper.xml');
    const id = (name) => solidNamed(drawing, name).id;
    for (const name of ['Left finger', 'Right finger']) drawing = updateSolid(drawing, id(name), { colliders: [{ shape: 'sphere', size: { radius: 0.012 } }] });
    drawing = updateContact(updateSolid(drawing, id('Part'), { colliders: [{ shape: 'box', size: { x: 0.06, y: 0.06, z: 0.06 } }] }), {});
    // Only what matters touches: the fingers neither each other nor the floor.
    drawing = setTouching(setTouching(setTouching(drawing, id('Left finger'), id('Right finger'), false), 'ground', id('Left finger'), false), 'ground', id('Right finger'), false);
    assert.deepEqual(contactPairs(drawing).pairs.map(({ name }) => name), ['Floor and Part', 'Left finger and Part', 'Right finger and Part']);
    const joint = (name) => drawing.joints.find((candidate) => candidate.name === name);
    drawing = updateJoint(drawing, joint('Lift').id, { drive: { setpoint: 0.1 } });
    for (const name of ['Left', 'Right']) drawing = updateJoint(drawing, joint(name).id, { drive: { setpoint: 0.04 } });
    const controls = Object.fromEntries(moving(drawing).map(({ id: jointId, drive }) => [jointId, drive.setpoint]));
    const { run } = await check('gripper', drawing, 1.2, { controls });
    // The fingers stop at the box's sides, and the box comes up with them.
    assert.ok(Math.abs(seriesOf(run, drawing, joint('Left')).at(-1) - 0.0285) < 0.001, 'The finger should stop at the box.');
    assert.ok(run.series('Part.z').at(-1) > 0.07, 'The box should come off the floor.');
});
