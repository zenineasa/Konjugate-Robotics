/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Couplings and tendons (tests/fixtures/tendon): an arm with a fixed tendon over its shoulder and elbow (a spring,
// damping, a dry friction and a limit), and two fingers held mirrored by a joint equality and closed by a servo on a
// tendon over both, as the Panda's are. Imported, built into Konjugate and run by its engine; against the Newton-Euler
// reference, against MuJoCo running the model's own file, and against MuJoCo running the drawing written back. Then
// without any tendon damping (the second sweep takes the joints' damping), and with a tendon over the elbow and both
// fingers (its damping kept only in part, as MuJoCo keeps it) and a coupling of a polynomial.

import assert from 'node:assert/strict';
import { mkdtemp, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { addTendon, checkDrawing, updateCoupling, updateTendon } from '../../packages/toolbox/lib/drawing.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { simulate } from './newtonEuler.mjs';

const fixture = join(roboticsRoot, 'tests', 'fixtures', 'tendon', 'tendon.xml');
const directory = await mkdtemp(join(tmpdir(), 'robotics-tendons-'));
const withoutMujoco = await mujocoMissing();
const model = readModel({ 'tendon.xml': await readFile(fixture, 'utf8') }, 'tendon.xml');
const imported = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Tendons' }).drawing;
const names = ['Shoulder', 'Elbow', 'Left', 'Right'];
const jointNamed = (drawing, name) => drawing.joints.find((joint) => joint.name === name);
const solidOf = (drawing, joint) => drawing.solids.find(({ id }) => id === drawing.frames.find((frame) => frame.id === joint.child).solid);
const seriesOf = (run, drawing, name) => {
    const joint = jointNamed(drawing, name);
    return run.series(`${solidOf(drawing, joint).name}.${joint.kind === 'slider' ? 'jointPosition' : 'jointAngle'}`);
};
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));

// Runs `drawing` for `runTime`, checks it against the reference to rounding, and returns the run.
async function matchesReference(key, drawing, runTime) {
    assert.deepEqual(checkDrawing(drawing).errors, []);
    const run = await runDocument(directory, key, buildModel(drawing, { timeStep: 0.001, runTime }).document);
    const reference = simulate(drawing, run.times);
    for (const name of names) {
        const difference = largestDifference(seriesOf(run, drawing, name), reference[jointNamed(drawing, name).id]);
        assert.ok(difference < 1e-9, `${key}: ${name} differs from the reference by ${difference}.`);
    }
    return run;
}

async function matchesWrittenBack(key, drawing, run, bound = 1e-9) {
    const reference = await runReference(directory, { mjcf: writeMjcf(drawing), joints: names.map((name) => ({ name: jointNamed(drawing, name).id })), times: run.times });
    for (const name of names) {
        const difference = largestDifference(seriesOf(run, drawing, name), reference[jointNamed(drawing, name).id]);
        console.log(`${key}, ${name}, written back: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
        assert.ok(difference < bound, `${key}: ${name} differs from MuJoCo running the drawing written back by ${difference}.`);
    }
}

const asImported = await matchesReference('tendons', imported, 2);

test('the arm and its fingers, as imported, move as the reference moves them: the strap\'s limit and the grip\'s servo held', () => {
    assert.ok(Math.max(...seriesOf(asImported, imported, 'Shoulder')) > 1.2, 'The shoulder should swing down to its limit.');
    const strap = asImported.series('Strap.length');
    const shoulder = seriesOf(asImported, imported, 'Shoulder');
    const elbow = seriesOf(asImported, imported, 'Elbow');
    strap.forEach((length, index) => assert.ok(Math.abs(length - (shoulder[index] + 0.5 * elbow[index])) < 1e-12, 'The strap\'s length is the shoulder plus half the elbow.'));
    // The servo closes the grip to 3 cm, each finger alike.
    assert.ok(Math.abs(asImported.series('Grip.length').at(-1) - 0.03) < 1e-4);
    const [left, right] = ['Left', 'Right'].map((name) => seriesOf(asImported, imported, name));
    assert.ok(largestDifference(left, right) < 1e-6, 'The coupling holds the fingers mirrored.');
});

test('MuJoCo running the model\'s own file moves the arm and its fingers as Konjugate moves them', { skip: withoutMujoco ?? false }, async () => {
    const reference = await runReference(directory, { mjcfPath: fixture, joints: names.map((name) => ({ name })), times: asImported.times });
    for (const name of names) {
        const difference = largestDifference(seriesOf(asImported, imported, name), reference[name]);
        console.log(`${name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
        assert.ok(difference < 1e-9, `${name} differs from MuJoCo by ${difference}.`);
    }
});

test('the drawing written back as MJCF runs in MuJoCo as Konjugate runs it', { skip: withoutMujoco ?? false }, async () => {
    await matchesWrittenBack('tendons', imported, asImported);
});

test('without a tendon\'s damping the second sweep takes the joints\' damping, and the robot moves as the reference and MuJoCo move it', async () => {
    let drawing = imported;
    for (const tendon of drawing.tendons) drawing = updateTendon(drawing, tendon.id, { damping: 0, drive: { damping: 0 } });
    const run = await matchesReference('undamped', drawing, 2);
    if (!withoutMujoco) await matchesWrittenBack('undamped', drawing, run);
});

test('a tendon over the elbow and both fingers, its damping kept only where the mass matrix has entries, and a coupling of a polynomial', async () => {
    const [elbow, left, right] = ['Elbow', 'Left', 'Right'].map((name) => jointNamed(imported, name).id);
    const strung = addTendon(imported, [{ joint: elbow, coef: 0.2 }, { joint: left, coef: 1 }, { joint: right, coef: -0.5 }], { name: 'Mixed' });
    let drawing = updateTendon(strung.drawing, strung.id, { damping: 0.5, stiffness: 3, friction: 0.02 });
    drawing = updateCoupling(drawing, drawing.couplings[0].id, { polycoef: [0.001, 0.9, 2, 0, 0] });
    const run = await matchesReference('mixed', drawing, 2);
    // The right finger follows the left by the polynomial.
    const [l, r] = ['Left', 'Right'].map((name) => seriesOf(run, drawing, name));
    assert.ok(Math.abs(r.at(-1) - (0.001 + 0.9 * l.at(-1) + 2 * l.at(-1) ** 2)) < 1e-4);
    if (!withoutMujoco) await matchesWrittenBack('mixed', drawing, run);
});
