/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Couplings and tendons (lib/drawing.mjs): drawn, changed and checked; their rows' inverse inertias as MuJoCo's; built
// as relationships of their own (lib/modelBuilder.mjs); and written back as MJCF (lib/mjcfWriter.mjs). The MuJoCo
// figures are MuJoCo 3.16's, printed from the compiled fixture (tests/fixtures/tendon).

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import { addCoupling, addTendon, checkDrawing, removeCouplings, removeJoints, removeTendons, updateCoupling, updateTendon } from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel, inverseWeights, symmetricEigen } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const close = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)), `${actual} is not ${expected}.`);
const model = readModel({ 'tendon.xml': await readFile(join(roboticsRoot, 'tests', 'fixtures', 'tendon', 'tendon.xml'), 'utf8') }, 'tendon.xml');
const { drawing } = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Tendons' });
const jointNamed = (from, name) => from.joints.find((joint) => joint.name === name);
const errors = (from) => checkDrawing(from).errors.map(({ message }) => message).join(' ');

test('a coupling and a tendon are drawn over a robot\'s joints, changed, checked and go with their joints', () => {
    const arm = exampleDrawing('twoLinkArm');
    const [shoulder, elbow] = ['Shoulder', 'Elbow'].map((name) => jointNamed(arm, name).id);
    const coupled = addCoupling(arm, elbow, shoulder);
    let next = coupled.drawing;
    assert.deepEqual(next.couplings[0], { id: coupled.id, name: 'Elbow with Shoulder', first: elbow, second: shoulder, polycoef: [0, 1, 0, 0, 0] });
    next = updateCoupling(next, coupled.id, { polycoef: [0.1, -2, 0, 0, 0] });
    assert.deepEqual(next.couplings[0].polycoef, [0.1, -2, 0, 0, 0]);
    const strung = addTendon(next, [{ joint: shoulder, coef: 1 }, { joint: elbow, coef: -0.5 }], { name: 'Elbow with Shoulder' });
    next = strung.drawing;
    assert.equal(next.tendons[0].name, 'Elbow with Shoulder 2', 'names are unique across solids, joints, couplings and tendons');
    next = updateTendon(next, strung.id, { stiffness: 3, limits: { lower: -1, upper: 1 }, drive: { mode: 'servo', setpoint: 0.2 } });
    assert.deepEqual(next.tendons[0].drive, { mode: 'servo', torque: 0, setpoint: 0.2, gain: 100, damping: 0, torqueLimit: 100 });
    assert.deepEqual(checkDrawing(next).errors, []);
    // Checks: a joint coupled to itself, a polynomial of four numbers, a tendon over a joint twice, a negative stiffness,
    // limits the wrong way round, a servo with no stiffness.
    assert.match(errors(updateCoupling(next, coupled.id, { second: elbow })), /couples a joint with itself/);
    assert.match(errors(updateCoupling(next, coupled.id, { polycoef: [0, 1, 0, 0] })), /its polynomial is five numbers/);
    assert.match(errors(updateTendon(next, strung.id, { parts: [{ joint: shoulder, coef: 1 }, { joint: shoulder, coef: 2 }] })), /runs over a joint twice/);
    assert.match(errors(updateTendon(next, strung.id, { parts: [{ joint: shoulder, coef: 0 }] })), /a number other than zero/);
    assert.match(errors(updateTendon(next, strung.id, { stiffness: -1 })), /its stiffness must be zero or more/);
    assert.match(errors(updateTendon(next, strung.id, { limits: { lower: 1, upper: -1 } })), /shortest length must be below its longest/);
    assert.match(errors(updateTendon(next, strung.id, { drive: { gain: 0 } })), /servo's stiffness and most force must be more than zero/);
    // A coupling on a fixed joint is refused.
    const fixed = arm.joints.find((joint) => joint.kind === 'fixed');
    if (fixed) assert.match(errors(addCoupling(next, fixed.id, shoulder).drawing), /couples hinges and sliders/);
    // With the elbow gone, the coupling goes, and the tendon keeps its shoulder.
    const without = removeJoints(next, [elbow]);
    assert.deepEqual(without.couplings, []);
    assert.deepEqual(without.tendons[0].parts, [{ joint: shoulder, coef: 1 }]);
    assert.deepEqual(removeTendons(removeCouplings(next, [coupled.id]), [strung.id]).tendons, []);
});

test('a tendon\'s inverse inertia where drawn is MuJoCo\'s tendon_invweight0, cᵀ M⁻¹ c', () => {
    const weights = inverseWeights(drawing);
    for (const [name, value] of Object.entries({ Shoulder: 25.817314438052431, Elbow: 226.93045227070715, Left: 20, Right: 20 })) close(weights.get(jointNamed(drawing, name).id), value);
    close(weights.get(drawing.tendons.find(({ name }) => name === 'Strap').id), 16.977690472849702);
    close(weights.get(drawing.tendons.find(({ name }) => name === 'Grip').id), 10);
});

test('a tendon is a node, each of its joints a relationship to it, and a coupling a relationship between its two joints\' solids', () => {
    const { document } = buildModel(drawing, { timeStep: 0.001, runTime: 1 });
    const strap = document.nodes.find(({ name }) => name === 'Strap');
    assert.equal(strap.type, 'robotTendon');
    assert.deepEqual(strap.states.map(({ symbol }) => symbol), ['length', 'speed', 'force', 'limitSide']);
    const solidId = (name) => document.nodes.find((node) => node.name === name).id;
    const over = document.edges.filter(({ target }) => target.nodeId === strap.id).map(({ name, source }) => [name, source.nodeId]);
    assert.deepEqual(over, [['Strap over Shoulder', solidId('Upper arm')], ['Strap over Elbow', solidId('Forearm')]]);
    const mirror = document.edges.find(({ name }) => name === 'Mirror');
    assert.deepEqual([mirror.source.nodeId, mirror.target.nodeId], [solidId('Left finger'), solidId('Right finger')]);
    // The servo on the grip has its own damping, so every damping is a row: no second sweep.
    const finger = document.nodes.find(({ name }) => name === 'Left finger');
    assert.ok(!finger.states.some(({ symbol }) => symbol === 'ib11'));
    assert.ok(finger.states.some(({ symbol }) => symbol === 'tendonForce'));
    // Without any tendon's damping, the second sweep takes the joints' damping.
    let undamped = drawing;
    for (const tendon of drawing.tendons) undamped = updateTendon(undamped, tendon.id, { damping: 0, drive: { damping: 0 } });
    assert.ok(buildModel(undamped, { timeStep: 0.001, runTime: 1 }).document.nodes.find(({ name }) => name === 'Left finger').states.some(({ symbol }) => symbol === 'ib11'));
});

test('a tendon\'s damping over two joints side by side is two rows, one on each, as MuJoCo\'s implicitfast keeps only the mass matrix\'s entries', () => {
    const { document } = buildModel(drawing, { timeStep: 0.001, runTime: 1 });
    // The grip's servo damping (c = [0.5, 0.5], c cᵀ kept only on its diagonal): two rows, each its h b times 0.25.
    const grip = document.edges.find(({ name }) => name === 'Grip over Left');
    const symbolOf = new Map(document.nodes.flatMap(({ states }) => states).map(({ id, symbol }) => [id, symbol]));
    const damping = grip.equationModel.outputs.filter(({ stateId }) => /^kc\d+$/.test(symbolOf.get(stateId)));
    assert.equal(damping.length, 2);
    for (const { mathJson } of damping) assert.equal(mathJson[1], 0.25);
    // The eigenvectors of c cᵀ kept where M has entries: for the two fingers, diagonal.
    const values = symmetricEigen([[0.25, 0], [0, 0.25]]).map(({ value }) => value);
    assert.deepEqual(values, [0.25, 0.25]);
    // Kept in part (an elbow carrying two fingers side by side), with a negative eigenvalue; the sum of λ v vᵀ is it.
    const G = [[0.04, 0.2, -0.1], [0.2, 1, 0], [-0.1, 0, 0.25]];
    const found = symmetricEigen(G);
    assert.ok(found.some(({ value }) => value < 0));
    G.forEach((row, i) => row.forEach((value, j) => close(found.reduce((sum, { value: lambda, vector }) => sum + lambda * vector[i] * vector[j], 0), value, 1e-12)));
});

test('couplings and tendons are written back as MJCF: a joint equality, fixed tendons counted from where drawn and a general actuator', () => {
    let shifted = updateTendon(drawing, drawing.tendons[0].id, { offset: 0.25, springLength: 0.35, limits: { lower: -0.35, upper: 0.85 } });
    shifted = updateTendon(shifted, drawing.tendons[1].id, { offset: 0.01, drive: { setpoint: 0.04 } });
    const mjcf = writeMjcf(shifted);
    const [strap, grip] = shifted.tendons;
    assert.match(mjcf, new RegExp(`<fixed name="${strap.id}" stiffness="2" springlength="0.1" damping="0.3" frictionloss="0.05" limited="true" range="-0.6 0.6">`));
    assert.match(mjcf, new RegExp(`<joint joint="${jointNamed(shifted, 'Elbow').id}" coef="0.5"/>`));
    const [mirror] = shifted.couplings;
    assert.match(mjcf, new RegExp(`<joint name="${mirror.id}" joint1="${mirror.first}" joint2="${mirror.second}" polycoef="0 1 0 0 0" solref="0.005 1" solimp="0.95 0.99 0.001 0.5 2"/>`));
    // The servo's target less the offset, times its stiffness, as its bias.
    assert.match(mjcf, new RegExp(`<general name="${grip.id}" tendon="${grip.id}" gainprm="0" biastype="affine" biasprm="3 -100 -10" forcelimited="true" forcerange="-20 20"/>`));
});
