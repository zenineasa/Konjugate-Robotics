/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Contact in the drawing (lib/contact.mjs): which pairs touch, as MuJoCo would pick them and as the user says; the
// checks; what goes with a solid; and the pairs written back as MJCF with MuJoCo's mixed parameters. Then what Build
// writes for it: equations that stay small, and a solve whose rows with no room are not worked out.

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import { addSolid, checkDrawing, createDrawing, removeSolids, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { collidersOf, contactPairs, setTouching, updateContact } from '../../packages/toolbox/lib/contact.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { bodyLook, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { buildModel, symmetricSolver } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const named = (drawing, name) => drawing.solids.find((solid) => solid.name === name).id;
const names = (drawing) => contactPairs(drawing).pairs.map(({ name }) => name);

test('without contact turned on nothing touches, and a drawing builds as it did', () => {
    const arm = exampleDrawing('twoLinkArm');
    assert.deepEqual(contactPairs(arm), { pairs: [], unsupported: [] });
    assert.ok(!buildModel(arm, { runTime: 1 }).document.edges.some(({ name }) => / and /.test(name)));
});

test('the pairs that touch: MuJoCo\'s rule (not joined, not welded together, not a weld and its parent, not two that cannot move), named pairs and exclusions', () => {
    let arm = updateContact(exampleDrawing('twoLinkArm'), {});
    // The base is fixed to the Ground, so it never touches the floor; the turret, the upper arm and the forearm do. Two
    // boxes, or a cylinder and a box, would touch but cannot yet: said so.
    assert.deepEqual(names(arm), ['Floor and Turret', 'Floor and Upper arm', 'Floor and Forearm']);
    assert.deepEqual(contactPairs(arm).unsupported.map(({ kinds }) => kinds.join(' and ')), ['cylinder and box', 'cylinder and box', 'box and box']);
    assert.match(checkDrawing(arm).warnings.map(({ message }) => message).join(' '), /Turret and Forearm: a box against a box does not touch yet; they pass through each other/);
    // Excluded, then named back, then as the rule has it again.
    const forearm = named(arm, 'Forearm');
    arm = setTouching(arm, 'ground', forearm, false);
    assert.deepEqual(names(arm), ['Floor and Turret', 'Floor and Upper arm']);
    arm = updateContact(arm, { floor: false });
    assert.deepEqual(names(arm), []);
    arm = setTouching(arm, 'ground', forearm, true);
    assert.deepEqual(names(arm), ['Floor and Forearm']);
    arm = setTouching(updateContact(arm, { floor: true }), 'ground', forearm, null);
    assert.equal(names(arm).length, 3);
    // A solid that does not touch has no colliders; one of several colliders is numbered.
    const turret = named(arm, 'Turret');
    assert.deepEqual(collidersOf(updateSolid(arm, turret, { touches: false }).solids.find(({ id }) => id === turret)), []);
    const two = updateSolid(arm, turret, { colliders: [{ shape: 'sphere', size: { radius: 0.02 } }, { shape: 'sphere', size: { radius: 0.02 }, position: [0, 0, 0.1] }] });
    assert.deepEqual(names(two).slice(0, 2), ['Floor and Turret 1', 'Floor and Turret 2']);
    // A solid gone takes its named pairs with it.
    assert.deepEqual(removeSolids(setTouching(arm, turret, forearm, false), [turret]).contact.exclude, []);
});

test('two solids joined to nothing touch each other and the floor, and their surfaces are checked', () => {
    let drawing = createDrawing('Two');
    const first = addSolid(drawing, 'sphere');
    const second = addSolid(first.drawing, 'capsule');
    drawing = updateContact(second.drawing, {});
    assert.deepEqual(names(drawing), ['Floor and Sphere', 'Floor and Capsule', 'Sphere and Capsule']);
    assert.match(checkDrawing(updateSolid(drawing, first.id, { surface: { friction: -1 } })).errors.map(({ message }) => message).join(' '), /Sphere: its friction must be zero or more/);
    assert.match(checkDrawing(updateContact(drawing, { floorSurface: { solref: [1] } })).errors.map(({ message }) => message).join(' '), /The floor: its contact's solref is two numbers/);
});

test('the pairs are written back as MJCF <pair>s, with the friction, softness and margin MuJoCo would mix', () => {
    let drawing = createDrawing('Two');
    const first = addSolid(drawing, 'sphere');
    drawing = updateContact(updateSolid(first.drawing, first.id, { surface: { friction: 0.6, solref: [0.04, 1], margin: 0.001 } }), { floorSurface: { friction: 0.3 } });
    const mjcf = writeMjcf(drawing);
    assert.match(mjcf, new RegExp(`<pair geom1="floor" geom2="${first.id}" condim="3" friction="0.6 0.6 0.005 0.0001 0.0001" solref="0.03 1" solimp="0.9 0.95 0.001 0.5 2" margin="0.001"/>`));
    assert.match(mjcf, /<geom name="floor" type="plane"[^>]*contype="0" conaffinity="0"/);
});

// Works out MathJSON as Konjugate's engine does: a definition when first read, and only a choice's branch taken.
// Returns { value(expression), operations() }, operations() the terms worked out so far.
function evaluator(definitions, values) {
    const byName = new Map(definitions.map(({ symbol, mathJson }) => [symbol, mathJson]));
    const known = new Map(Object.entries(values));
    let operations = 0;
    const read = (name) => {
        if (name === 'True') return 1;
        if (!known.has(name)) {
            if (!byName.has(name)) throw new Error(`Nothing is ${name}.`);
            known.set(name, value(byName.get(name)));
        }
        return known.get(name);
    };
    const value = (expression) => {
        operations += 1;
        if (typeof expression === 'number') return expression;
        if (typeof expression === 'string') return read(expression);
        const [operator, ...items] = expression;
        if (operator === 'Which') {
            for (let index = 0; index < items.length; index += 2) if (value(items[index]) !== 0) return value(items[index + 1]);
            throw new Error('No branch matched.');
        }
        const all = items.map(value);
        switch (operator) {
            case 'Add': return all.reduce((sum, item) => sum + item, 0);
            case 'Multiply': return all.reduce((product, item) => product * item, 1);
            case 'Negate': return -all[0];
            case 'Divide': return all[0] / all[1];
            case 'Greater': return Number(all[0] > all[1]);
            case 'Less': return Number(all[0] < all[1]);
            default: throw new Error(`No ${operator} here.`);
        }
    };
    return { value, operations: () => operations };
}

test('a solve given which rows are active works out only theirs, and finds what the whole solve finds', () => {
    // Ten rows, three active (as a few contact points of many touch): the others are the identity's rows, holding their
    // right-hand sides.
    const active = [1, 0, 0, 1, 0, 0, 0, 1, 0, 0];
    const n = active.length;
    const block = [[4, 1, 0.5], [1, 3, 0.2], [0.5, 0.2, 2]];
    const values = {};
    const on = active.flatMap((flag, index) => (flag ? [index] : []));
    for (let i = 0; i < n; i += 1) {
        values[`a${i + 1}`] = active[i];
        values[`b${i + 1}`] = (i + 1) * (i % 2 ? -0.7 : 0.3);
        for (let j = 0; j <= i; j += 1) values[`d${i + 1}x${j + 1}`] = active[i] && active[j] ? block[on.indexOf(i)][on.indexOf(j)] : Number(i === j);
    }
    const D = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => `d${Math.max(i, j) + 1}x${Math.min(i, j) + 1}`));
    const b = Array.from({ length: n }, (_, i) => `b${i + 1}`);
    const solve = (flags) => {
        const definitions = [];
        const define = (symbol, mathJson) => {
            if (typeof mathJson === 'number' || typeof mathJson === 'string') return mathJson;
            definitions.push({ symbol, mathJson });
            return symbol;
        };
        const x = symmetricSolver(define, 's', D, flags)(b, 'x');
        const { value, operations } = evaluator(definitions, values);
        return { x: x.map(value), operations: operations() };
    };
    const whole = solve(null);
    const some = solve(active.map((_, i) => `a${i + 1}`));
    assert.deepEqual(some.x, whole.x);
    active.forEach((flag, i) => { if (!flag) assert.equal(some.x[i], values[`b${i + 1}`]); });
    on.forEach((i) => assert.ok(Math.abs(block[on.indexOf(i)].reduce((sum, entry, k) => sum + entry * some.x[on[k]], 0) - values[`b${i + 1}`]) < 1e-14));
    assert.ok(some.operations < 0.5 * whole.operations, `${some.operations} terms worked out, against ${whole.operations}.`);
});

test('a box on the floor and spheres against it are equations of a size that does not grow with the corners and faces counted', async () => {
    const xml = await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'gripper.xml'), 'utf8');
    const model = readModel({ 'gripper.xml': xml }, 'gripper.xml');
    let drawing = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Gripper' }).drawing;
    for (const name of ['Left finger', 'Right finger']) drawing = updateSolid(drawing, named(drawing, name), { colliders: [{ shape: 'sphere', size: { radius: 0.012 } }] });
    drawing = updateContact(updateSolid(drawing, named(drawing, 'Part'), { colliders: [{ shape: 'box', size: { x: 0.06, y: 0.06, z: 0.06 } }] }), {});
    const size = (expression) => (Array.isArray(expression) ? 1 + expression.slice(1).reduce((sum, item) => sum + size(item), 0) : 1);
    const contacts = buildModel(drawing, { runTime: 1 }).document.edges.filter(({ name }) => / and /.test(name));
    assert.ok(contacts.some(({ name }) => name === 'Floor and Part') && contacts.some(({ name }) => name === 'Left finger and Part'));
    // Each corner's count and each face's nearest so far is named once: no definition repeats those before it.
    for (const { name, equationModel } of contacts) {
        for (const { symbol, mathJson } of equationModel.definitions) assert.ok(size(mathJson) < 150, `${name}: ${symbol} is ${size(mathJson)} terms.`);
    }
});
