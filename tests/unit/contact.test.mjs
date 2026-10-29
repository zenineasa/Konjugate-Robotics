/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Contact in the drawing (lib/contact.mjs): which pairs touch, as MuJoCo would pick them and as the user says; the
// checks; what goes with a solid; and the pairs written back as MJCF with MuJoCo's mixed parameters.

import assert from 'node:assert/strict';
import test from 'node:test';
import { addSolid, checkDrawing, createDrawing, removeSolids, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { collidersOf, contactPairs, setTouching, updateContact } from '../../packages/toolbox/lib/contact.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';

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
