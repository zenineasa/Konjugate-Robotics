/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Bodies with more than one joint, imported from a MuJoCo model (tests/fixtures/gimbal: two hinges one after the other,
// a hinge and a slide along one axis, three hinges at one point; servos on a hinge and on the slide). Each becomes a
// chain of links with no mass, one after each joint but the last (lib/mjcfImport.mjs). Built into Konjugate and run by
// its engine, against the Newton-Euler reference; against MuJoCo running the model's own file; and against MuJoCo
// running the drawing written back as MJCF (lib/mjcfWriter.mjs), where each link's joint goes back into its body.

import assert from 'node:assert/strict';
import { mkdtemp, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { checkDrawing } from '../../packages/toolbox/lib/drawing.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { simulate } from './newtonEuler.mjs';

const fixture = join(roboticsRoot, 'tests', 'fixtures', 'gimbal', 'gimbal.xml');
const directory = await mkdtemp(join(tmpdir(), 'robotics-multijoint-'));
const withoutMujoco = await mujocoMissing();
const model = readModel({ 'gimbal.xml': await readFile(fixture, 'utf8') }, 'gimbal.xml');
const { drawing, looks } = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Gimbal' });
const runTime = 2;
const run = await runDocument(directory, 'gimbal', buildModel(drawing, { timeStep: 0.001, runTime, looks }).document);
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));
const jointNamed = (name) => drawing.joints.find((joint) => joint.name === name);
const carriedBy = (joint) => drawing.solids.find(({ id }) => id === drawing.frames.find((frame) => frame.id === joint.child).solid);
const moving = ['Yaw', 'Pitch', 'Twist', 'Reach', 'Turn', 'Nod', 'Roll'];
const coordinateOf = (name) => `${carriedBy(jointNamed(name)).name}.${jointNamed(name).kind === 'slider' ? 'jointPosition' : 'jointAngle'}`;
const references = Object.fromEntries(model.bodies.flatMap((body) => body.joints).map((joint) => [joint.name, joint.reference]));

test('a body\'s joints become a chain of links with no mass, each where the body is, the body\'s solid on the last', () => {
    assert.deepEqual(drawing.solids.map(({ name, mass }) => [name, mass.value]), [
        ['Post', 1], ['Yaw link', 0], ['Arm', 0.4], ['Twist link', 0], ['Sleeve', 0.15], ['Turn link', 0], ['Nod link', 0], ['Head', 0.1]
    ]);
    const chain = drawing.joints.map((joint) => [joint.name, joint.kind, carriedBy(joint).name]);
    assert.deepEqual(chain, [
        ['Post weld', 'fixed', 'Post'], ['Yaw', 'hinge', 'Yaw link'], ['Pitch', 'hinge', 'Arm'], ['Twist', 'hinge', 'Twist link'],
        ['Reach', 'slider', 'Sleeve'], ['Turn', 'hinge', 'Turn link'], ['Nod', 'hinge', 'Nod link'], ['Roll', 'hinge', 'Head']
    ]);
    assert.deepEqual(checkDrawing(drawing).errors, []);
    assert.equal(jointNamed('Yaw').drive.mode, 'servo');
    assert.equal(jointNamed('Reach').drive.mode, 'servo');
    // A link with no mass that carries nothing is refused.
    const bare = structuredClone(drawing);
    bare.joints = bare.joints.filter(({ name }) => name !== 'Roll');
    assert.match(checkDrawing(bare).errors.map(({ message }) => message).join(' '), /Nod link: its mass must be more than zero/);
});

test('an imported body with several joints runs as the Newton-Euler reference runs the drawing it became', () => {
    const reference = simulate(drawing, run.times);
    for (const name of moving) {
        const difference = largestDifference(run.series(coordinateOf(name)), reference[jointNamed(name).id]);
        assert.ok(difference < 1e-9, `${name} differs from the reference by ${difference}.`);
    }
    for (const solid of drawing.solids) {
        for (const [axis, symbol] of ['x', 'y', 'z'].entries()) {
            const difference = largestDifference(run.series(`${solid.name}.${symbol}`), reference[solid.id].map((place) => place[axis]));
            assert.ok(difference < 1e-9, `${solid.name}'s ${symbol} differs from the reference by ${difference} m.`);
        }
    }
    // Things move: the arm, free to pitch, swings down; the head nods.
    assert.ok(Math.abs(run.series(coordinateOf('Pitch')).at(-1)) > 0.1, 'The arm should swing.');
    assert.ok(Math.max(...run.series(coordinateOf('Nod')).map(Math.abs)) > 0.01, 'The head should nod.');
});

test('MuJoCo running the model\'s own file moves each of a body\'s joints as Konjugate moves them', { skip: withoutMujoco ?? false }, async () => {
    const reference = await runReference(directory, {
        mjcf: (await readFile(fixture, 'utf8')).replace('<compiler angle="degree"/>', '<compiler angle="degree"/><option timestep="0.001" integrator="implicitfast"><flag contact="disable"/></option>'),
        joints: moving.map((name) => ({ name, initialPosition: references[name] })), times: run.times
    });
    for (const name of moving) {
        const difference = largestDifference(run.series(coordinateOf(name)).map((value) => value + references[name]), reference[name]);
        console.log(`${name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
        assert.ok(difference < 1e-8, `${name} differs from MuJoCo by ${difference}.`);
    }
});

test('the drawing written back as MJCF puts each link\'s joint back in its body, and MuJoCo runs it as Konjugate does', { skip: withoutMujoco ?? false }, async () => {
    const mjcf = writeMjcf(drawing);
    for (const link of drawing.solids.filter(({ mass }) => mass.value === 0)) assert.ok(!mjcf.includes(`<body name="${link.id}"`), `${link.name} should not be a body.`);
    const reference = await runReference(directory, { mjcf, joints: moving.map((name) => ({ name: jointNamed(name).id, initialPosition: 0 })), times: run.times });
    for (const name of moving) {
        const difference = largestDifference(run.series(coordinateOf(name)), reference[jointNamed(name).id]);
        console.log(`${name}, written back: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
        assert.ok(difference < 1e-8, `${name} differs from MuJoCo running the drawing written back by ${difference}.`);
    }
});
