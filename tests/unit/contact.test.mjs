/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Contact in the drawing (lib/contact.mjs): which pairs touch, as MuJoCo would pick them and as the user says; the
// checks; what goes with a solid; and the pairs written back as MJCF with MuJoCo's mixed parameters. Then what Build
// writes for it: equations that stay small, and the rows' forces found in the joints' coordinates as a solve over the
// rows themselves would have them.

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
import { buildModel, constraintPolishes, jointSpaceForces } from '../../packages/toolbox/lib/modelBuilder.mjs';
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
            case 'Sqrt': return Math.sqrt(all[0]);
            case 'Max': return Math.max(all[0], all[1]);
            case 'Min': return Math.min(all[0], all[1]);
            default: throw new Error(`No ${operator} here.`);
        }
    };
    return { value, operations: () => operations };
}

// Plain numbers: a seeded pseudo-random source, products, and a solve by Gaussian elimination.
const seeded = (seed) => () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
const times = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((sum, value, k) => sum + value * B[k][j], 0)));
const transpose = (A) => A[0].map((_, j) => A.map((row) => row[j]));
function solved(A, b) {
    const n = A.length;
    const M = A.map((row, i) => [...row, b[i]]);
    for (let c = 0; c < n; c += 1) {
        const pivot = M.slice(c).reduce((best, row, k) => (Math.abs(row[c]) > Math.abs(M[best][c]) ? c + k : best), c);
        [M[c], M[pivot]] = [M[pivot], M[c]];
        for (let r = c + 1; r < n; r += 1) { const factor = M[r][c] / M[c][c]; for (let k = c; k <= n; k += 1) M[r][k] -= factor * M[c][k]; }
    }
    const x = Array(n).fill(0);
    for (let r = n - 1; r >= 0; r -= 1) x[r] = (M[r][n] - M[r].slice(r + 1, n).reduce((sum, value, k) => sum + value * x[r + 1 + k], 0)) / M[r][r];
    return x;
}

// Works out the forces jointSpaceForces writes, for rows of each kind on joints of `n` coordinates, and checks them as a
// solve over the rows would have them, K = J M⁻¹ Jᵀ: the loops' rows (K_EE + ε) f_E = w_E - K_EC f_C; each searched row
// inside its bounds with nothing left over, or at a bound pushing against it; the damping rows (h b K + I) f_D =
// h b (w_D - K_DO f_O); and the joints' accelerations M⁻¹ Jᵀ f.
function checkJointSpaceForces(n, types, seed) {
    const random = seeded(seed);
    const R = types.length;
    const B = Array.from({ length: n }, () => Array.from({ length: n }, () => random()));
    const M = times(B, transpose(B)).map((row, i) => row.map((value, j) => value + (i === j ? 0.5 : 0)));
    const J = Array.from({ length: R }, () => Array.from({ length: n }, () => random() * 2));
    const values = { compliance: 1e-9 };
    const settings = types.map((type, r) => {
        const compliance = type === 'damping' ? 0.05 + Math.abs(random()) * 0.1 : 0.02 + Math.abs(random()) * 0.05;
        const bounds = { friction: [-0.3, 0.3], limit: [0, 1e30], contact: r % 3 ? [0, 1e30] : [0, 0], equality: [-1e30, 1e30] }[type] ?? [0, 0];
        return { compliance, low: bounds[0], high: bounds[1], wanted: random() * 4 };
    });
    types.forEach((type, r) => {
        values[`b${r}`] = settings[r].wanted;
        values[`warm${r}`] = 0;
        Object.assign(values, { [`kc${r}`]: settings[r].compliance, [`kn${r}`]: settings[r].low, [`kx${r}`]: settings[r].high });
        for (let d = 0; d < n; d += 1) values[`J${r}x${d}`] = J[r][d];
    });
    for (let a = 0; a < n; a += 1) for (let b = 0; b < n; b += 1) values[`M${a}x${b}`] = M[a][b];
    const definitions = [];
    const define = (symbol, mathJson) => {
        if (typeof mathJson === 'number' || typeof mathJson === 'string') return mathJson;
        definitions.push({ symbol, mathJson });
        return symbol;
    };
    const written = jointSpaceForces(define, n, {
        types, J: (r, d) => `J${r}x${d}`, M: (a, b) => `M${a}x${b}`, wanted: (r) => `b${r}`, setting: (prefix, r) => `${prefix}${r}`, warm: (r) => `warm${r}`
    }, true);
    const { value } = evaluator(definitions, values);
    const f = written.forces.map(value);
    const accelerations = written.accelerations.map(value);
    const Minv = transpose(Array.from({ length: n }, (_, k) => solved(M, Array.from({ length: n }, (__, i) => Number(i === k)))));
    const K = times(times(J, Minv), transpose(J));
    const Kf = (r, rows) => rows.reduce((sum, c) => sum + K[r][c] * f[c], 0);
    const all = types.map((_, r) => r);
    const held = all.filter((r) => types[r] === 'loop');
    const damped = all.filter((r) => types[r] === 'damping');
    const others = all.filter((r) => types[r] !== 'damping');
    const scale = Math.max(1, ...f.map(Math.abs));
    for (const r of held) {
        const left = Kf(r, held) + 1e-9 * f[r];
        assert.ok(Math.abs(left - (settings[r].wanted - Kf(r, all.filter((c) => !held.includes(c) && types[c] !== 'damping')))) < 1e-9 * scale, `loop row ${r}`);
    }
    for (const r of all.filter((c) => !held.includes(c) && types[c] !== 'damping')) {
        const { compliance, low, high, wanted } = settings[r];
        const residual = wanted - Kf(r, others) - compliance * f[r];
        assert.ok(f[r] >= low - 1e-12 && f[r] <= high + 1e-12, `row ${r} is inside its bounds`);
        if (f[r] > low + 1e-9 && f[r] < high - 1e-9) assert.ok(Math.abs(residual) < 1e-9 * scale, `row ${r}, inside its bounds, has ${residual} left`);
        else if (high > low && f[r] <= low + 1e-9) assert.ok(residual < 1e-9 * scale, `row ${r}, at its lower bound, pushes against it`);
        else if (high > low) assert.ok(residual > -1e-9 * scale, `row ${r}, at its upper bound, pushes against it`);
    }
    for (const r of damped) {
        const left = settings[r].compliance * Kf(r, damped) + f[r];
        assert.ok(Math.abs(left - settings[r].compliance * (settings[r].wanted - Kf(r, others))) < 1e-9 * scale, `damping row ${r}`);
    }
    const expected = times(Minv, transpose([times([f], J)[0]])).map(([entry]) => entry);
    // (Where the loops hold more than the joints can move, their forces are large and cancel: as large as they are.)
    expected.forEach((entry, d) => assert.ok(Math.abs(entry - accelerations[d]) < 1e-14 * scale * Math.max(1, Math.abs(entry)), `joint coordinate ${d}: ${entry} against ${accelerations[d]}`));
    return f;
}

test('the rows\' forces found in the joints\' coordinates are those a solve over the rows finds, loops and damping rows among them', () => {
    assert.equal(constraintPolishes, 6);
    // Six coordinates: two loop rows, contacts (a third with no room), dry frictions, a limit, a coupling and damping.
    checkJointSpaceForces(6, ['loop', 'contact', 'contact', 'contact', 'friction', 'loop', 'limit', 'equality', 'contact', 'friction', 'damping', 'damping'], 11);
    // More loop rows than coordinates (a link pinned at both ends): nothing is left free.
    checkJointSpaceForces(3, ['loop', 'loop', 'loop', 'loop', 'loop', 'friction', 'limit'], 5);
    // No loops, many more rows than coordinates.
    checkJointSpaceForces(4, Array.from({ length: 24 }, (_, r) => (r % 5 === 0 ? 'friction' : 'contact')), 3);
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

test('an imported model touches as MuJoCo has it: its collision geoms, contype and conaffinity, priority, condim, a pair and an exclusion', async () => {
    const xml = await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'collision.xml'), 'utf8');
    const model = readModel({ 'collision.xml': xml }, 'collision.xml');
    const { drawing, report } = importModel(model, model.bodies.map((body) => bodyLook(model, body)), { name: 'Collision' });
    assert.deepEqual(report.leftOut, []);
    assert.equal(drawing.contact.parentRule, 'mujoco');
    // Every pair of geoms that may touch, as MuJoCo 3.16 lists them with every geom's margin made 10 m, so that each
    // pair it does not leave out is in contact (one name pair per line of its contacts, the floor's 4 and 2 as one).
    const mujoco = ['ball base', 'ball brick', 'ball link1', 'ball link2', 'ball puck', 'ball tip', 'base brick', 'base link2', 'base tip', 'floor ball',
        'floor brick', 'floor drum', 'floor ghost', 'floor link1', 'floor link2', 'floor pill', 'floor puck', 'floor tip', 'link1 brick', 'link2 brick',
        'puck base', 'puck brick', 'puck link1', 'puck link2', 'puck tip', 'tip brick'];
    const nameOf = (entry) => entry.collider.name;
    const { pairs, unsupported } = contactPairs(drawing);
    assert.deepEqual(unsupported, []);
    const found = pairs.map(({ first, second }) => [nameOf(first), nameOf(second)].sort().join(' '));
    assert.deepEqual([...found].sort(), mujoco.map((pair) => pair.split(' ').sort().join(' ')).sort());
    // The settings as MuJoCo mixes them: the larger friction (the ball's 1 over the floor's 0.8); the brick's own, of its higher
    // priority, against the floor; the puck's condim 1 over all; the ghost's pair's own.
    const pairNamed = (a, b) => pairs.find(({ first, second }) => [nameOf(first), nameOf(second)].sort().join(' ') === [a, b].sort().join(' '));
    assert.deepEqual(pairNamed('floor', 'ball').surface, { friction: 1, solref: [0.02, 1], solimp: [0.9, 0.95, 0.001, 0.5, 2], margin: 0, condim: 3 });
    assert.deepEqual(pairNamed('floor', 'brick').surface, { friction: 0.4, solref: [0.03, 1], solimp: [0.9, 0.95, 0.001, 0.5, 2], margin: 0, condim: 3 });
    assert.equal(pairNamed('puck', 'brick').surface.condim, 1);
    assert.equal(pairNamed('puck', 'brick').rowsPerSlot, 1);
    assert.deepEqual(pairNamed('floor', 'ghost').surface, { friction: 0.5, solref: [0.01, 1], solimp: [0.9, 0.95, 0.001, 0.5, 2], margin: 0, condim: 3 });
    // Written back: every pair a <pair> of its own settings.
    assert.match(writeMjcf(drawing), /<pair geom1="floor" geom2="[^"]+" condim="1"/);
});
