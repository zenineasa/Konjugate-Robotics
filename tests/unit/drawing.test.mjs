/* Copyright © 2026 Zenin Easa Panthakkalakath */

import assert from 'node:assert/strict';
import test from 'node:test';
import {
    addFrameAt, addJoint, addSolid, checkDrawing, createDrawing, frameById, frameInWorld, groundId, massOf, readDrawing, removeJoints, removeSolids,
    reverseJoint, structure, treeOrder, updateJoint, updateSolid
} from '../../packages/toolbox/lib/drawing.mjs';
import { History } from '../../packages/toolbox/lib/history.mjs';
import { length, rotate, subtract } from '../../packages/toolbox/lib/math.mjs';

const close = (actual, expected, tolerance = 1e-9) => assert.ok(actual.every((value, index) => Math.abs(value - expected[index]) <= tolerance), `${actual} is not ${expected}`);

// A box on the floor and a capsule beside it.
function twoSolids() {
    let drawing = createDrawing('Test');
    const box = addSolid(drawing, 'box');
    drawing = box.drawing;
    const capsule = addSolid(drawing, 'capsule', { position: [0.3, 0, 0.12] });
    return { drawing: capsule.drawing, box: box.id, capsule: capsule.id };
}

test('a new solid rests on the floor, gets a unique name and a mass from its density', () => {
    let { drawing, box } = twoSolids();
    const solid = drawing.solids.find(({ id }) => id === box);
    assert.equal(solid.name, 'Box');
    close(solid.position, [0, 0, 0.05]);
    assert.ok(Math.abs(massOf(solid) - 1240 * 0.001) < 1e-12, 'a 10 cm PLA cube is 1.24 kg');
    drawing = addSolid(drawing, 'box').drawing;
    assert.equal(drawing.solids.at(-1).name, 'Box 2');
    drawing = updateSolid(drawing, box, { mass: { by: 'mass', value: 0.3 } });
    assert.equal(massOf(drawing.solids[0]), 0.3);
});

test('joining mates the second solid to the first: frames meet, faces together, and its frames move with it', () => {
    const { drawing, box, capsule } = twoSolids();
    // The box's +x face centre, then a point on the capsule's side.
    const first = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const second = addFrameAt(first.drawing, capsule, [0.32, 0, 0.12], [1, 0, 0]);
    const joined = addJoint(second.drawing, first.id, second.id, { kind: 'hinge' });
    const joint = joined.drawing.joints[0];
    const parent = frameInWorld(joined.drawing, frameById(joined.drawing, joint.parent));
    const child = frameInWorld(joined.drawing, frameById(joined.drawing, joint.child));
    close(parent.point, child.point);
    close(child.axis, parent.axis.map((value) => -value));
    // The capsule's middle is now its radius out from the face.
    close(joined.drawing.solids[1].position, [0.07, 0, 0.05]);
    assert.equal(joint.name, 'Hinge');
    assert.deepEqual(joint.drive, { mode: 'free', torque: 0, setpoint: 0, gain: 20, torqueLimit: 2 });
    // The original drawing is untouched.
    close(drawing.solids[1].position, [0.3, 0, 0.12]);
});

test('a joint without mating leaves both solids where they are, and the checks say how far apart its points are', () => {
    const { drawing, box, capsule } = twoSolids();
    const first = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const second = addFrameAt(first.drawing, capsule, [0.28, 0, 0.12], [-1, 0, 0]);
    const joined = addJoint(second.drawing, first.id, second.id, { mate: false });
    close(joined.drawing.solids[1].position, [0.3, 0, 0.12]);
    assert.match(checkDrawing(joined.drawing).notes[0].message, /apart/);
});

test('the Ground is only ever a joint\'s parent, and a joint joins two different solids', () => {
    const { drawing, box } = twoSolids();
    const ground = addFrameAt(drawing, groundId, [0, 0, 0], [0, 0, 1]);
    const top = addFrameAt(ground.drawing, box, [0, 0, 0.1], [0, 0, 1]);
    assert.throws(() => addJoint(top.drawing, top.id, ground.id), /Ground is where a robot starts/);
    const other = addFrameAt(top.drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    assert.throws(() => addJoint(other.drawing, top.id, other.id), /two different solids/);
    // Fixing the box to the Ground by its top turns it over onto the floor point.
    const fixed = addJoint(top.drawing, ground.id, top.id, { kind: 'fixed' });
    close(fixed.drawing.solids[0].position, [0, 0, 0.05]);
    close(rotate(fixed.drawing.solids[0].rotation, [0, 0, 1]), [0, 0, -1]);
});

test('structure walks from the Ground: parents, a loop and a floating solid', () => {
    let { drawing, box, capsule } = twoSolids();
    const ground = addFrameAt(drawing, groundId, [0, 0, 0], [0, 0, 1]);
    const bottom = addFrameAt(ground.drawing, box, [0, 0, 0], [0, 0, -1]);
    drawing = addJoint(bottom.drawing, ground.id, bottom.id, { kind: 'fixed' }).drawing;
    const side = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const end = addFrameAt(side.drawing, capsule, [0.3, 0, 0.0], [0, 0, -1]);
    drawing = addJoint(end.drawing, side.id, end.id, { mate: false }).drawing;
    const lonely = addSolid(drawing, 'sphere', { position: [1, 1, 1] });
    drawing = lonely.drawing;
    let built = structure(drawing);
    assert.equal(built.parentJoint.get(box).parent, groundId);
    assert.equal(built.parentJoint.get(capsule).parent, box);
    assert.deepEqual(built.roots, [lonely.id]);
    assert.match(checkDrawing(drawing).warnings[0].message, /Sphere is not joined to the Ground, so it falls freely/);
    assert.deepEqual(treeOrder(drawing).map(({ id, depth }) => [id, depth]), [[box, 0], [capsule, 1], [lonely.id, 0]]);
    // A second joint between the capsule and the Ground closes a loop.
    const floor = addFrameAt(drawing, groundId, [0.3, 0, 0], [0, 0, 1]);
    const tip = addFrameAt(floor.drawing, capsule, [0.3, 0, 0], [0, 0, -1]);
    drawing = addJoint(tip.drawing, floor.id, tip.id, { mate: false, kind: 'ball' }).drawing;
    built = structure(drawing);
    // Walked breadth first from the Ground, the capsule is reached through its own joint to the Ground first, so the
    // joint to the box is the one that closes the loop.
    assert.equal(built.loops.length, 1);
    assert.equal(built.loops[0].name, 'Hinge');
    assert.equal(built.parentJoint.get(capsule).parent, groundId);
    assert.deepEqual(checkDrawing(drawing).errors, []);
    drawing = updateJoint(drawing, built.loops[0].id, { kind: 'slider' });
    assert.match(checkDrawing(drawing).errors.map(({ message }) => message).join(' '), /a slider cannot close a loop yet/);
});

test('checks catch what cannot be built, each about the part it names', () => {
    let { drawing, box, capsule } = twoSolids();
    assert.deepEqual(checkDrawing(drawing).errors, []);
    drawing = updateSolid(drawing, box, { size: { x: 0 } });
    drawing = updateSolid(drawing, capsule, { name: 'Box' });
    const errors = checkDrawing(drawing).errors;
    assert.ok(errors.some(({ id, message }) => id === box && /Length \(x\) must be more than zero/.test(message)));
    assert.ok(errors.some(({ message }) => /Two things are called "Box"/.test(message)));
    assert.match(checkDrawing(createDrawing()).errors[0].message, /Add a solid to start/);
});

test('removing a solid removes its frames and joints; removing a joint removes the frames only it used', () => {
    const { drawing, box, capsule } = twoSolids();
    const first = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const second = addFrameAt(first.drawing, capsule, [0.32, 0, 0.12], [1, 0, 0]);
    const joined = addJoint(second.drawing, first.id, second.id).drawing;
    const withoutCapsule = removeSolids(joined, [capsule]);
    assert.equal(withoutCapsule.joints.length, 0);
    assert.deepEqual(withoutCapsule.frames.map(({ solid }) => solid), [box]);
    const withoutJoint = removeJoints(joined, [joined.joints[0].id]);
    assert.equal(withoutJoint.frames.length, 0);
    assert.equal(withoutJoint.solids.length, 2);
});

test('reversing a joint swaps its frames, but a joint on the Ground keeps the Ground as its parent', () => {
    const { drawing, box, capsule } = twoSolids();
    const first = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const second = addFrameAt(first.drawing, capsule, [0.32, 0, 0.12], [1, 0, 0]);
    const joined = addJoint(second.drawing, first.id, second.id).drawing;
    const reversed = reverseJoint(joined, joined.joints[0].id);
    assert.equal(reversed.joints[0].parent, joined.joints[0].child);
    const ground = addFrameAt(drawing, groundId, [0, 0, 0], [0, 0, 1]);
    const bottom = addFrameAt(ground.drawing, box, [0, 0, 0], [0, 0, -1]);
    const fixed = addJoint(bottom.drawing, ground.id, bottom.id, { kind: 'fixed' }).drawing;
    assert.throws(() => reverseJoint(fixed, fixed.joints[0].id), /keeps the Ground/);
});

test('a drawing reads back from JSON, and one of another format or version is refused by name', () => {
    const { drawing } = twoSolids();
    assert.deepEqual(readDrawing(JSON.parse(JSON.stringify(drawing))), drawing);
    assert.throws(() => readDrawing({ format: 'other' }), /not a Robotics drawing/);
    assert.throws(() => readDrawing({ ...drawing, version: 9 }), /version 9/);
});

test('undo and redo step through drawings, typing in one field in quick succession being one step', () => {
    let time = 0;
    const history = new History(createDrawing('A'), { now: () => time });
    const first = addSolid(history.current, 'box').drawing;
    history.change(first, 'Add Box');
    const typed = [0.2, 0.25, 0.3].map((x) => updateSolid(history.current, first.solids[0].id, { size: { x } }));
    for (const step of typed) { time += 300; history.change(step, 'Size of Box', 'size'); }
    assert.equal(history.undoStack.length, 2, 'three figures typed in a row are one step');
    assert.equal(history.undo().label, 'Size of Box');
    assert.equal(history.current, first);
    assert.equal(history.undo().label, 'Add Box');
    assert.equal(history.redo().label, 'Add Box');
    assert.equal(history.canRedo, true);
    history.change(createDrawing('B'), 'Replace');
    assert.equal(history.canRedo, false, 'a new change drops what could be redone');
});

test('a frame keeps its place on its solid as the solid moves', () => {
    const { drawing, box } = twoSolids();
    const framed = addFrameAt(drawing, box, [0.05, 0, 0.05], [1, 0, 0]);
    const moved = updateSolid(framed.drawing, box, { position: [1, 2, 3] });
    const world = frameInWorld(moved, frameById(moved, framed.id));
    close(world.point, [1.05, 2, 3]);
    assert.ok(length(subtract(world.axis, [1, 0, 0])) < 1e-12);
});
