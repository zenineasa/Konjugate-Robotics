/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Limits and dry friction held as MuJoCo holds them (lib/modelBuilder.mjs): each joint's inverse inertia where drawn
// as MuJoCo's dof_invweight0, each row's B, K and impedance as MuJoCo works them out, and a model's own settings read,
// kept and written back. The MuJoCo figures below are MuJoCo 3.16's, printed from the compiled fixtures.

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import { checkDrawing, updateJoint } from '../../packages/toolbox/lib/drawing.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel, defaultSoftness, impedanceAt, inverseWeights, rowReference, softnessOf } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const close = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)), `${actual} is not ${expected}.`);
const imported = async (files, main) => {
    const model = readModel(files, main);
    return importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Robot' }).drawing;
};
const fixture = (...path) => join(roboticsRoot, 'tests', 'fixtures', ...path);

test('a joint\'s inverse inertia where drawn is MuJoCo\'s dof_invweight0', async () => {
    const gimbal = await imported({ 'gimbal.xml': await readFile(fixture('gimbal', 'gimbal.xml'), 'utf8') }, 'gimbal.xml');
    const weights = inverseWeights(gimbal);
    const mujoco = { Yaw: 45.983179889803885, Pitch: 55.973911367251716, Twist: 1528.6115765231389, Reach: 4.0436585396821618, Turn: 1699.8117514738321, Nod: 1649.9308172681756, Roll: 1795.553839287129 };
    for (const [name, value] of Object.entries(mujoco)) close(weights.get(gimbal.joints.find((joint) => joint.name === name).id), value);
    // With a ball among the joints, which moves the solids as any joint does.
    const arm = await imported({
        'scene.xml': await readFile(fixture('arm', 'scene.xml'), 'utf8'), 'arm.xml': await readFile(fixture('arm', 'arm.xml'), 'utf8'),
        'meshes/turret.stl': new Uint8Array(await readFile(fixture('arm', 'meshes', 'turret.stl')))
    }, 'scene.xml');
    const armWeights = inverseWeights(arm);
    for (const [name, value] of Object.entries({ Waist: 20.485898960188862, Shoulder: 60.455863573321416, Elbow: 438.76994144505943, Extend: 9.9009900990099009 })) {
        close(armWeights.get(arm.joints.find((joint) => joint.name === name).id), value);
    }
});

test('a row\'s B, K and impedance as MuJoCo works them out', () => {
    // MuJoCo's defaults, a millisecond's step: B = 2 / (dmax timeconst), K = 1 / (dmax² timeconst² ratio²).
    const { B, K } = rowReference(defaultSoftness, 0.001);
    close(B, 105.26315789473685);
    close(K, 2770.0831024930749);
    // A time constant under two steps is taken as two.
    close(rowReference({ solref: [0.001, 1], solimp: defaultSoftness.solimp }, 0.001).B, 2 / (0.95 * 0.002));
    // Given as negatives: a stiffness and a damping.
    const given = rowReference({ solref: [-400, -20], solimp: [0.9, 0.8, 0.001, 0.5, 2] }, 0.001);
    close(given.B, 20 / 0.8);
    close(given.K, 400 / 0.64);
    // The impedance: dmin at the limit, dmax a width past it, and between them MuJoCo's curve of its power.
    close(impedanceAt(defaultSoftness.solimp, 0), 0.9);
    close(impedanceAt(defaultSoftness.solimp, -0.3), 0.95);
    close(impedanceAt(defaultSoftness.solimp, -0.0005), 0.925);
    close(impedanceAt(defaultSoftness.solimp, -0.00025), 0.9 + 0.05 * 0.125);
    close(impedanceAt(defaultSoftness.solimp, -0.00075), 0.9 + 0.05 * 0.875);
});

test('a model\'s own softness for a limit and a dry friction is read, built, kept and written back', async () => {
    const xml = `<mujoco><compiler angle="radian"/><default><joint solreffriction="0.05 0.5"/></default><worldbody>
        <body name="Arm" pos="0 0 0.3"><joint name="Pivot" axis="0 1 0" range="-1 1" frictionloss="0.2" solreflimit="0.01 2" solimplimit="0.8 0.9" margin="0.01"/>
        <inertial pos="0.1 0 0" mass="1" diaginertia="0.001 0.001 0.001"/></body></worldbody></mujoco>`;
    const drawing = await imported({ 'arm.xml': xml }, 'arm.xml');
    const pivot = drawing.joints.find((joint) => joint.name === 'Pivot');
    assert.deepEqual(pivot.softness, { limit: { solref: [0.01, 2], solimp: [0.8, 0.9, 0.001, 0.5, 2] }, friction: { solref: [0.05, 0.5] }, margin: 0.01 });
    assert.deepEqual(softnessOf(pivot), {
        limit: { solref: [0.01, 2], solimp: [0.8, 0.9, 0.001, 0.5, 2] }, friction: { solref: [0.05, 0.5], solimp: defaultSoftness.solimp }, margin: 0.01
    });
    // Built: the row's settings on its relationship.
    const edge = buildModel(drawing, { timeStep: 0.001, runTime: 1 }).document.edges.find(({ name }) => name === 'Pivot');
    const value = (symbol) => edge.parameters.find((parameter) => parameter.symbol === symbol).value;
    close(value('frictionDamping'), 2 / (0.95 * 0.05));
    close(value('limitDamping'), 2 / (0.9 * 0.01));
    close(value('margin'), 0.01);
    close(value('inverseWeight'), 1 / 0.011);
    // Written back as MJCF, as it came.
    const mjcf = writeMjcf(drawing);
    assert.match(mjcf, /solreflimit="0\.01 2" solimplimit="0\.8 0\.9 0\.001 0\.5 2" solreffriction="0\.05 0\.5" margin="0\.01"/);
    // A joint of another kind holds MuJoCo's defaults again; a solref is two numbers.
    assert.equal(updateJoint(drawing, pivot.id, { kind: 'slider' }).joints.find(({ id }) => id === pivot.id).softness, undefined);
    const broken = structuredClone(drawing);
    broken.joints.find(({ id }) => id === pivot.id).softness.limit.solref = [1];
    assert.match(checkDrawing(broken).errors.map(({ message }) => message).join(' '), /Pivot: its limit's solref is two numbers/);
});
