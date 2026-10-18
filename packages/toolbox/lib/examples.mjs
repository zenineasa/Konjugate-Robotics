/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Drawings to start from, built with the same operations a user's clicks make (lib/drawing.mjs), so each is a drawing
// like any other: a pendulum, a two-link arm on a turning waist, and a cart on a rail carrying two pendulums.

import { addFrameAt, addJoint, addSolid, createDrawing, updateJoint, updateSolid } from './drawing.mjs';
import { add, between, length, scale, subtract } from './math.mjs';

export const examples = Object.freeze([
    { id: 'pendulum', name: 'Pendulum', description: 'An arm on a post, free to swing: let it go and watch it fall and swing back.' },
    { id: 'twoLinkArm', name: 'Two-link arm', description: 'A waist, a shoulder and an elbow, each held by a servo at its setpoint: move the sliders in the canvas.' },
    { id: 'cartPendulums', name: 'Cart and pendulums', description: 'A servo slides a cart along a rail; the two pendulums hanging from it swing as it starts and stops.' }
]);

// Places a solid, then turns and moves it to `position` and `rotation`.
function place(drawing, shape, { name, size, position, rotation, color }) {
    const added = addSolid(drawing, shape, { name, size, position, color });
    return { drawing: updateSolid(added.drawing, added.id, { position, ...(rotation ? { rotation } : {}) }), id: added.id };
}

// Joins `parent` and `child` at `point` with `axis` (both in the world), leaving both where they are.
function join(drawing, parent, child, point, axis, kind, name) {
    const parentFrame = addFrameAt(drawing, parent, point, axis);
    const childFrame = addFrameAt(parentFrame.drawing, child, point, scale(axis, -1));
    return addJoint(childFrame.drawing, parentFrame.id, childFrame.id, { kind, mate: false, name });
}

// A capsule from `from` to `to` (its straight middle between them).
function rod(drawing, { name, from, to, radius, color }) {
    const along = subtract(to, from);
    return place(drawing, 'capsule', {
        name, color, size: { radius, length: length(along) },
        position: scale(add(from, to), 0.5), rotation: between([0, 0, 1], along)
    });
}

export function exampleDrawing(id) {
    if (id === 'pendulum') {
        let drawing = createDrawing('Pendulum');
        let result = place(drawing, 'box', { name: 'Post', size: { x: 0.04, y: 0.04, z: 0.2 }, position: [0, 0, 0.1], color: '#7d8fa3' });
        drawing = result.drawing;
        const post = result.id;
        result = join(drawing, 'ground', post, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount');
        drawing = result.drawing;
        // The arm lies along x from the post's side, its end at the hinge.
        result = rod(drawing, { name: 'Arm', from: [0, 0.035, 0.18], to: [0.2, 0.035, 0.18], radius: 0.015, color: '#d08b4a' });
        drawing = result.drawing;
        const arm = result.id;
        result = join(drawing, post, arm, [0, 0.02, 0.18], [0, 1, 0], 'hinge', 'Pivot');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.002 });
        return drawing;
    }
    if (id === 'twoLinkArm') {
        let drawing = createDrawing('Two-link arm');
        let result = place(drawing, 'cylinder', { name: 'Base', size: { radius: 0.05, length: 0.04 }, position: [0, 0, 0.02], color: '#5f6f80' });
        drawing = result.drawing;
        const base = result.id;
        drawing = join(drawing, 'ground', base, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        result = place(drawing, 'box', { name: 'Turret', size: { x: 0.06, y: 0.06, z: 0.08 }, position: [0, 0, 0.08], color: '#5b8db8' });
        drawing = result.drawing;
        const turret = result.id;
        result = join(drawing, base, turret, [0, 0, 0.04], [0, 0, 1], 'hinge', 'Waist');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.05, drive: { mode: 'servo', setpoint: 0, gain: 20, torqueLimit: 2 } });
        result = place(drawing, 'box', { name: 'Upper arm', size: { x: 0.03, y: 0.025, z: 0.2 }, position: [0, 0.0425, 0.185], color: '#d08b4a' });
        drawing = result.drawing;
        const upper = result.id;
        result = join(drawing, turret, upper, [0, 0.03, 0.1], [0, 1, 0], 'hinge', 'Shoulder');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.05, limits: { lower: -Math.PI / 2, upper: Math.PI / 2 }, drive: { mode: 'servo', setpoint: 0, gain: 30, torqueLimit: 3 }
        });
        result = place(drawing, 'box', { name: 'Forearm', size: { x: 0.025, y: 0.02, z: 0.18 }, position: [0, 0.065, 0.3475], color: '#6aa86a' });
        drawing = result.drawing;
        const forearm = result.id;
        result = join(drawing, upper, forearm, [0, 0.055, 0.27], [0, 1, 0], 'hinge', 'Elbow');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.03, limits: { lower: -2.5, upper: 2.5 }, drive: { mode: 'servo', setpoint: 0.5, gain: 15, torqueLimit: 1.5 }
        });
        return drawing;
    }
    if (id === 'cartPendulums') {
        let drawing = createDrawing('Cart and pendulums');
        let result = place(drawing, 'box', { name: 'Rail', size: { x: 0.6, y: 0.03, z: 0.02 }, position: [0, 0, 0.31], color: '#5f6f80' });
        drawing = result.drawing;
        const rail = result.id;
        drawing = join(drawing, 'ground', rail, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        result = place(drawing, 'box', { name: 'Cart', size: { x: 0.08, y: 0.06, z: 0.04 }, position: [0, 0, 0.28], color: '#5b8db8' });
        drawing = result.drawing;
        const cart = result.id;
        result = join(drawing, rail, cart, [0, 0, 0.3], [1, 0, 0], 'slider', 'Track');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.5, limits: { lower: -0.25, upper: 0.25 }, drive: { mode: 'servo', setpoint: 0.1, gain: 40, torqueLimit: 5 }
        });
        // One pendulum let go from an angle, one hanging straight down, on either side of the cart.
        result = rod(drawing, { name: 'Front pendulum', from: [0, 0.04, 0.26], to: [0.1, 0.04, 0.09], radius: 0.01, color: '#d08b4a' });
        drawing = result.drawing;
        const front = result.id;
        result = join(drawing, cart, front, [0, 0.04, 0.26], [0, 1, 0], 'hinge', 'Front swing');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.001 });
        result = rod(drawing, { name: 'Back pendulum', from: [0, -0.04, 0.26], to: [0, -0.04, 0.1], radius: 0.01, color: '#6aa86a' });
        drawing = result.drawing;
        const back = result.id;
        result = join(drawing, cart, back, [0, -0.04, 0.26], [0, 1, 0], 'hinge', 'Back swing');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.001 });
        return drawing;
    }
    throw new Error(`There is no example "${id}".`);
}
