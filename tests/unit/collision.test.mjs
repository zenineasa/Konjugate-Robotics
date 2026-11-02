/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Where two shapes touch (lib/collision.mjs), against MuJoCo 3.16's own contacts for the same shapes where MuJoCo put
// them (tests/fixtures/contact/mujocoContacts.json: random poses, and poses square to each other, for each pair the
// toolbox collides as MuJoCo's own functions do; mujocoConvexContacts.json: a cylinder against a capsule, a cylinder
// or a box, which MuJoCo finds by its convex search, run to a tolerance it reaches; mujocoMeshContacts.json: random
// convex meshes against the floor, a sphere and a capsule); and the same functions written as equations
// (lib/modelBuilder.mjs's expressionAlgebra) giving the same numbers.

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import { collide, contactSlots, numbers, pairKey } from '../../packages/toolbox/lib/collision.mjs';
import * as E from '../../packages/toolbox/lib/expressions.mjs';
import { hullOf } from '../../packages/toolbox/lib/hull.mjs';
import { expressionAlgebra } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const { cases } = JSON.parse(await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'mujocoContacts.json'), 'utf8'));
const { cases: convexCases } = JSON.parse(await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'mujocoConvexContacts.json'), 'utf8'));
const { cases: meshRecords } = JSON.parse(await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'mujocoMeshContacts.json'), 'utf8'));
// Each mesh with its hull, from the corners MuJoCo keeps.
const meshCases = meshRecords.map(({ first, second, contacts }) => ({ first, second: { ...second, hull: hullOf(second.points) }, contacts }));
const convexPairs = ['capsule|cylinder', 'cylinder|box', 'cylinder|cylinder', 'plane|mesh', 'sphere|mesh', 'capsule|mesh'];
const differ = (a, b) => Math.max(...a.map((value, index) => Math.abs(value - b[index])));

test('each pair\'s contacts are MuJoCo\'s: as many, each at its distance, point and frame', () => {
    const seen = new Map();
    for (const { first, second, contacts } of cases) {
        const key = pairKey(first.kind, second.kind);
        const found = collide(numbers, first, second, 0).filter(({ active }) => active);
        assert.equal(found.length, contacts.length, `${key}: ${found.length} contacts, MuJoCo ${contacts.length}`);
        for (const contact of contacts) {
            const match = found.reduce((best, slot) => (differ(slot.pos, contact.pos) < differ(best.pos, contact.pos) ? slot : best));
            const error = Math.max(differ(match.pos, contact.pos), Math.abs(match.dist - contact.dist), differ(match.frame, contact.frame));
            assert.ok(error < 1e-12, `${key}: differs from MuJoCo by ${error}`);
        }
        seen.set(key, (seen.get(key) ?? 0) + contacts.length);
    }
    for (const key of Object.keys(contactSlots).filter((pair) => !convexPairs.includes(pair))) assert.ok(seen.get(key) > 0, `${key} should touch in some case`);
});

// How far a shape reaches along unit n, written here from each shape's own geometry.
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const axisOf = (shape, k) => [shape.mat[k], shape.mat[3 + k], shape.mat[6 + k]];
const reach = (shape, n) => {
    const centre = dot(shape.pos, n);
    if (shape.kind === 'capsule') return centre + shape.size[1] * Math.abs(dot(n, axisOf(shape, 2))) + shape.size[0];
    if (shape.kind === 'cylinder') { const c = dot(n, axisOf(shape, 2)); return centre + shape.size[1] * Math.abs(c) + shape.size[0] * Math.sqrt(Math.max(0, 1 - c * c)); }
    return centre + [0, 1, 2].reduce((sum, k) => sum + shape.size[k] * Math.abs(dot(n, axisOf(shape, k))), 0);
};
const gapAlong = (first, second, n) => -(reach(first, n) + reach(second, n.map((value) => -value)));

test('a cylinder against a capsule, a cylinder or a box: the deepest contact exactly, as MuJoCo\'s convex search closes on it', () => {
    let single = 0;
    for (const { first, second, contacts } of convexCases) {
        const key = pairKey(first.kind, second.kind);
        const touching = collide(numbers, first, second, 0).filter(({ active }) => active);
        assert.ok(touching.length > 0, `${key}: they overlap, so they touch`);
        const deepest = touching.reduce((best, slot) => (slot.dist < best.dist ? slot : best));
        // The gap along the deepest contact's normal is its distance (within the nanometre by which a cap's or a face's
        // normal is preferred), and along MuJoCo's normals no wider: the largest.
        assert.ok(Math.abs(gapAlong(first, second, deepest.frame.slice(0, 3)) - deepest.dist) < 2e-9, `${key}: the distance is the gap along the normal`);
        for (const { frame } of contacts) assert.ok(gapAlong(first, second, frame.slice(0, 3)) <= deepest.dist + 1e-9, `${key}: MuJoCo found a wider gap`);
        // Where MuJoCo gives one contact at the gap along its normal (not one of a flat face's, which it places and
        // deepens by rules of its own), the same contact, to its search's precision.
        if (contacts.length !== 1) continue;
        const [theirs] = contacts;
        if (Math.abs(gapAlong(first, second, theirs.frame.slice(0, 3)) - theirs.dist) > 1e-8) continue;
        single += 1;
        assert.ok(Math.abs(theirs.dist - deepest.dist) < 1e-8, `${key}: ${deepest.dist} deep, MuJoCo ${theirs.dist}`);
        assert.ok(differ(theirs.frame.slice(0, 3), deepest.frame.slice(0, 3)) < 1e-4, `${key}: the normal differs from MuJoCo's by ${differ(theirs.frame.slice(0, 3), deepest.frame.slice(0, 3))}`);
    }
    assert.ok(single >= 55, `Most cases have one such contact, not ${single}.`);
});

test('flat on a face: a cylinder standing on a box, lying across one, under a lid, and cans stacked, touch at several points', () => {
    const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    const lying = [1, 0, 0, 0, 0, -1, 0, 1, 0];
    const cylinder = (pos, mat = I, size = [0.04, 0.06]) => ({ kind: 'cylinder', size, pos, mat });
    const box = (pos, size) => ({ kind: 'box', size, pos, mat: I });
    const touching = (first, second) => collide(numbers, first, second, 0).filter(({ active }) => active);
    const near = (points, expected, message) => {
        assert.equal(points.length, expected.length, message);
        for (const [x, y, z, dist] of expected) {
            assert.ok(points.some(({ pos, dist: d }) => Math.abs(pos[0] - x) < 1e-12 && Math.abs(pos[1] - y) < 1e-12 && Math.abs(pos[2] - z) < 1e-12 && Math.abs(d - dist) < 1e-12), `${message}: a point at (${x}, ${y}, ${z})`);
        }
    };
    const third = 0.04 * Math.sqrt(3) / 2;
    // Standing 9 mm into a table: three points on its rim (MuJoCo's plane against cylinder), midway through the overlap.
    near(touching(cylinder([0.01, 0, -0.009]), box([0, 0, -0.11], [0.1, 0.1, 0.05])), [[0.05, 0, -0.0645, -0.009], [-0.01, third, -0.0645, -0.009], [-0.01, -third, -0.0645, -0.009]], 'standing on a box');
    // Lying along a narrow box: the two ends of the line along its side, where it lies over the box.
    near(touching(cylinder([0, 0, 0.039], lying), box([0, 0, -0.05], [0.02, 0.1, 0.05])), [[0, -0.06, -0.0005, -0.001], [0, 0.06, -0.0005, -0.001]], 'lying on a box');
    // A big lid on a small post: the post's rim, over the lid's face.
    near(touching(cylinder([0, 0, 0]), box([0.01, 0, 0.109], [0.1, 0.1, 0.05])), [[0.04, 0, 0.0595, -0.001], [-0.02, -third, 0.0595, -0.001], [-0.02, third, 0.0595, -0.001]], 'a lid on a post');
    // A can on a can, 1 cm off its axis: the upper can's rim, where it lies over the lower can's cap.
    near(touching(cylinder([0, 0, 0]), cylinder([0.01, 0, 0.119])), [[-0.01, third, 0.0595, -0.001], [-0.01, -third, 0.0595, -0.001], [0.04, 0, 0.0595, -0.001]], 'a can on a can');
    // A can lying across a narrow post: where its side crosses the post's cap.
    near(touching(cylinder([0, 0, 0], I, [0.01, 0.06]), cylinder([0, 0, 0.099], lying)), [[0, -0.01, 0.0595, -0.001], [0, 0.01, 0.0595, -0.001]], 'lying across a post');
});

test('written as equations, each pair\'s functions give the numbers they give', () => {
    for (const { first, second } of [...cases.filter((_, index) => index % 3 === 0), ...convexCases.filter((_, index) => index % 6 === 0), ...meshCases.filter((_, index) => index % 10 === 0)]) {
        const definitions = new Map();
        const algebra = expressionAlgebra((symbol, expression) => {
            if (typeof expression === 'number') return expression;
            assert.ok(!definitions.has(symbol), `${symbol} is defined twice`);
            definitions.set(symbol, expression);
            return symbol;
        });
        // Each shape's pose given as symbols, as the model's states are.
        const values = {};
        const symbolic = (shape, tag) => ({
            kind: shape.kind, size: shape.size, hull: shape.hull, centre: shape.centre,
            pos: shape.pos.map((value, i) => { values[`${tag}p${i}`] = value; return `${tag}p${i}`; }),
            mat: shape.mat.map((value, i) => { values[`${tag}m${i}`] = value; return `${tag}m${i}`; })
        });
        const slots = collide(algebra, symbolic(first, 'a'), symbolic(second, 'b'), 0);
        const resolved = new Map();
        const evaluate = (expression) => {
            for (const symbol of E.symbolsOf(expression)) {
                if (symbol in values || !definitions.has(symbol)) continue;
                if (!resolved.has(symbol)) resolved.set(symbol, evaluate(definitions.get(symbol)));
                values[symbol] = resolved.get(symbol);
            }
            return E.evaluate(expression, values);
        };
        const expected = collide(numbers, first, second, 0);
        slots.forEach((slot, k) => {
            assert.equal(evaluate(algebra.number(slot.active)) > 0.5, expected[k].active, `${first.kind} and ${second.kind}, slot ${k + 1}: whether it touches`);
            if (!expected[k].active) return;
            const error = Math.max(Math.abs(evaluate(slot.dist) - expected[k].dist), differ(slot.pos.map(evaluate), expected[k].pos), differ(slot.frame.map(evaluate), expected[k].frame));
            assert.ok(error < 1e-12, `${first.kind} and ${second.kind}, slot ${k + 1}: differs by ${error}`);
        });
    }
});

// How far a mesh's corners, a sphere or a capsule reach along unit n.
const meshReach = (shape, n) => {
    if (shape.kind === 'sphere') return dot(shape.pos, n) + shape.size[0];
    if (shape.kind !== 'mesh') return reach(shape, n);
    return Math.max(...shape.hull.points.map((p) => dot(n, [0, 1, 2].map((i) => shape.pos[i] + dot([shape.mat[3 * i], shape.mat[3 * i + 1], shape.mat[3 * i + 2]], p)))));
};
const meshGap = (first, second, n) => -(meshReach(first, n) + meshReach(second, n.map((value) => -value)));

test('a mesh against the floor: MuJoCo\'s contacts, its deepest corner and its face\'s corners within the margin', () => {
    let several = 0;
    for (const { first, second, contacts } of meshCases.filter(({ first }) => first.kind === 'plane')) {
        const found = collide(numbers, first, second, 0).filter(({ active }) => active);
        assert.equal(found.length, contacts.length, `${found.length} contacts, MuJoCo ${contacts.length}`);
        for (const contact of contacts) {
            const match = found.reduce((best, slot) => (differ(slot.pos, contact.pos) < differ(best.pos, contact.pos) ? slot : best));
            // MuJoCo keeps a mesh's corners in single precision, these numbers its own.
            const error = Math.max(differ(match.pos, contact.pos), Math.abs(match.dist - contact.dist), differ(match.frame.slice(0, 3), contact.frame.slice(0, 3)));
            assert.ok(error < 1e-9, `differs from MuJoCo by ${error}`);
        }
        if (contacts.length > 1) several += 1;
    }
    assert.ok(several >= 15, `${several} cases with several contacts`);
});

test('a sphere or a capsule against a mesh: the deepest contact exactly, as MuJoCo\'s convex search closes on it', () => {
    let single = 0;
    let flat = 0;
    for (const { first, second, contacts } of meshCases.filter(({ first }) => first.kind !== 'plane')) {
        const touching = collide(numbers, first, second, 0).filter(({ active }) => active);
        assert.equal(touching.length > 0, contacts.length > 0, `${first.kind}: touching as MuJoCo has it`);
        if (!touching.length) continue;
        const deepest = touching.reduce((best, slot) => (slot.dist < best.dist ? slot : best));
        // The gap along the deepest contact's normal is its distance, and along MuJoCo's normals no wider.
        assert.ok(Math.abs(meshGap(first, second, deepest.frame.slice(0, 3)) - deepest.dist) < 1e-12, `${first.kind}: the distance is the gap along the normal`);
        for (const { frame } of contacts) assert.ok(meshGap(first, second, frame.slice(0, 3)) <= deepest.dist + 1e-12, `${first.kind}: MuJoCo found a wider gap`);
        // Where MuJoCo gives one contact, whose distance is the gap along its normal (its search closed): the same.
        if (contacts.length === 1 && touching.length === 1 && Math.abs(meshGap(first, second, contacts[0].frame.slice(0, 3)) - contacts[0].dist) < 1e-9) {
            assert.ok(Math.abs(deepest.dist - contacts[0].dist) < 1e-9, `${first.kind}: distance ${deepest.dist}, MuJoCo ${contacts[0].dist}`);
            assert.ok(differ(deepest.frame.slice(0, 3), contacts[0].frame.slice(0, 3)) < 1e-6, `${first.kind}: normal differs from MuJoCo's`);
            assert.ok(differ(deepest.pos, contacts[0].pos) < 1e-7, `${first.kind}: point differs from MuJoCo's`);
            single += 1;
        }
        // A capsule lying along a face: two contacts, each within its own depth, at the ends of what lies over the face.
        if (first.kind === 'capsule' && touching.length === 2 && contacts.length === 2) {
            for (const contact of contacts) {
                const match = touching.reduce((best, slot) => (differ(slot.pos, contact.pos) < differ(best.pos, contact.pos) ? slot : best));
                assert.ok(differ(match.pos, contact.pos) < 1e-3, 'a point lying along the face is MuJoCo\'s, to a millimetre');
            }
            flat += 1;
        }
    }
    assert.ok(single >= 40, `${single} single contacts compared`);
    assert.ok(flat >= 5, `${flat} capsules lying along a face`);
});

test('two equal boxes stacked, the top one turned an eighth of a turn, touch at the eight corners of their overlap', () => {
    const turned = [Math.SQRT1_2, -Math.SQRT1_2, 0, Math.SQRT1_2, Math.SQRT1_2, 0, 0, 0, 1];
    const lower = { kind: 'box', size: [0.05, 0.05, 0.05], pos: [0, 0, 0], mat: [1, 0, 0, 0, 1, 0, 0, 0, 1] };
    const upper = { kind: 'box', size: [0.05, 0.05, 0.05], pos: [0, 0, 0.099], mat: turned };
    const found = collide(numbers, lower, upper, 0).filter(({ active }) => active);
    assert.equal(found.length, 8, 'The overlap is an octagon, eight contacts, as MuJoCo has it.');
    // MuJoCo puts them at the octagon's corners, (0.05, 0.05 (2^0.5 - 1)) and its turns, midway through the 1 mm overlap.
    const near = 0.05 * (Math.SQRT2 - 1);
    const expected = [[near, 0.05], [0.05, near], [0.05, -near], [near, -0.05], [-near, -0.05], [-0.05, -near], [-0.05, near], [-near, 0.05]];
    for (const [x, y] of expected) {
        const match = found.find(({ pos }) => Math.abs(pos[0] - x) < 1e-12 && Math.abs(pos[1] - y) < 1e-12);
        assert.ok(match, `a contact at (${x}, ${y})`);
        assert.ok(Math.abs(match.pos[2] - 0.0495) < 1e-12 && Math.abs(match.dist + 0.001) < 1e-12, 'midway through the overlap, 1 mm deep');
        assert.deepEqual(match.frame.slice(0, 3).map((value) => Math.round(value * 1e12) / 1e12), [0, 0, 1]);
    }
});

test('a mesh\'s hull: its corners alone (not points on a face or an edge), faces outward, and MuJoCo\'s polygons', () => {
    // A box's corners with points on its faces, along its edges and inside: its eight corners, six square polygons.
    const box = [];
    for (const x of [-1, 0, 1]) for (const y of [-1, 0.5, 1]) for (const z of [-1, 0.25, 1]) box.push([0.03 * x, 0.02 * y, 0.01 * z]);
    const hull = hullOf(box);
    assert.equal(hull.points.length, 8);
    assert.equal(hull.triangles.length, 12);
    assert.deepEqual(hull.polygons.map(({ vertices }) => vertices.length), [4, 4, 4, 4, 4, 4]);
    // Random points: every one on or inside every face, each face outward (the centre inside), V - E + F = 2.
    let seed = 5;
    const random = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
    const points = Array.from({ length: 300 }, () => [random(), random(), random()].map((value) => value * 0.1));
    const cloud = hullOf(points);
    const centre = [0, 1, 2].map((axis) => cloud.points.reduce((sum, p) => sum + p[axis], 0) / cloud.points.length);
    cloud.normals.forEach((n, t) => {
        assert.ok(dot(n, centre) < cloud.offsets[t], 'each face faces outward');
        for (const p of points) assert.ok(dot(n, p) <= cloud.offsets[t] + 1e-12, 'every point lies within');
    });
    assert.equal(cloud.points.length - cloud.edges.length + cloud.triangles.length, 2);
    assert.throws(() => hullOf([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]), /not all in one plane/);
});
