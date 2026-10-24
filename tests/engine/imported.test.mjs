/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A robot imported from a MuJoCo model, built into Konjugate and run by its engine: against the Newton-Euler reference
// (the imported solids' own inertias reach the motion as drawn), and against MuJoCo running the model's own file, the
// one the import read (so the import itself is checked: every body's mass, centre and inertia, every joint's anchor,
// axis, ref, range, damping and armature, every servo's gain and gear, as MuJoCo has them). The model is
// tests/fixtures/arm, run without contact and at a millisecond with MuJoCo's implicitfast integrator, as Konjugate
// steps; what the import leaves out of it (a floor, a tendon, an equality, a damper) does nothing there.

import assert from 'node:assert/strict';
import { cp, mkdtemp, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { simulate } from './newtonEuler.mjs';

const fixture = join(roboticsRoot, 'tests', 'fixtures', 'arm');
const directory = await mkdtemp(join(tmpdir(), 'robotics-imported-'));
const withoutMujoco = await mujocoMissing();
const files = {
    'scene.xml': await readFile(join(fixture, 'scene.xml'), 'utf8'),
    'arm.xml': await readFile(join(fixture, 'arm.xml'), 'utf8'),
    'meshes/turret.stl': new Uint8Array(await readFile(join(fixture, 'meshes', 'turret.stl')))
};
const model = readModel(files, 'scene.xml');
const { drawing, looks } = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Test arm' });
const runTime = 2;
const run = await runDocument(directory, 'imported', buildModel(drawing, { timeStep: 0.001, runTime, looks }).document);
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));
const jointNamed = (name) => drawing.joints.find((joint) => joint.name === name);
const solidOfJoint = (joint) => drawing.solids.find(({ id }) => id === drawing.frames.find((frame) => frame.id === joint.child).solid);
const hinges = ['Waist', 'Shoulder', 'Elbow', 'Extend'];
const coordinateOf = (name) => `${solidOfJoint(jointNamed(name)).name}.${jointNamed(name).kind === 'slider' ? 'jointPosition' : 'jointAngle'}`;

test('an imported robot runs as the Newton-Euler reference runs the drawing it became', () => {
    const reference = simulate(drawing, run.times);
    for (const name of hinges) {
        const difference = largestDifference(run.series(coordinateOf(name)), reference[jointNamed(name).id]);
        assert.ok(difference < 1e-9, `${name} differs from the reference by ${difference}.`);
    }
    for (const solid of drawing.solids) {
        for (const [axis, symbol] of ['x', 'y', 'z'].entries()) {
            const difference = largestDifference(run.series(`${solid.name}.${symbol}`), reference[solid.id].map((place) => place[axis]));
            assert.ok(difference < 1e-9, `${solid.name}'s ${symbol} differs from the reference by ${difference} m.`);
        }
    }
    // Something moves: the elbow, unpowered, swings down; the servos hold the shoulder up against it.
    assert.ok(Math.abs(run.series(coordinateOf('Elbow')).at(-1)) > 0.1, 'The unpowered elbow should swing.');
});

// MuJoCo running the model's own file. A tree of hinges and a slider steps alike in both, so they agree to rounding;
// a ball turns by its step's exact rotation in MuJoCo and component by component here, so the bob's centre agrees to
// within a stated bound, as for drawn robots (tests/engine/drawn.test.mjs).
const treeBound = 1e-8;
const ballBound = 1e-3;
test('MuJoCo running the model\'s own file moves the robot as Konjugate moves the robot imported from it', { skip: withoutMujoco ?? false }, async () => {
    const copy = join(directory, 'model');
    await cp(fixture, copy, { recursive: true });
    await writeFile(join(copy, 'run.xml'), '<mujoco><include file="scene.xml"/><option timestep="0.001" integrator="implicitfast"><flag contact="disable"/></option></mujoco>\n');
    const references = Object.fromEntries(model.bodies.flatMap((body) => body.joints).filter(({ kind }) => kind === 'hinge' || kind === 'slide').map((joint) => [joint.name, joint.reference]));
    const reference = await runReference(directory, {
        mjcfPath: join(copy, 'run.xml'),
        joints: hinges.map((name) => ({ name, initialPosition: references[name] })),
        centres: ['Bob'], times: run.times
    });
    for (const name of hinges) {
        const konjugate = run.series(coordinateOf(name)).map((value) => value + references[name]);
        const difference = largestDifference(konjugate, reference[name]);
        console.log(`${name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
        assert.ok(difference < treeBound, `${name} differs from MuJoCo by ${difference}.`);
    }
    const bob = drawing.solids.find(({ name }) => name === 'Bob');
    const centre = ['x', 'y', 'z'].map((symbol) => run.series(`${bob.name}.${symbol}`));
    const difference = Math.max(...run.times.map((_, index) => Math.hypot(...[0, 1, 2].map((axis) => centre[axis][index] - reference.Bob[index][axis]))));
    console.log(`Bob's centre: Konjugate and MuJoCo differ by up to ${(difference * 1000).toFixed(4)} mm.`);
    assert.ok(difference < ballBound, `Bob's centre differs from MuJoCo by ${difference} m.`);
    assert.ok(Math.max(...reference.Bob.map(([, y]) => Math.abs(y - reference.Bob[0][1]))) > 0.005, 'The bob, let go tilted, should swing.');
});
