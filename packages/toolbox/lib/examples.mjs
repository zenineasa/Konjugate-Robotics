/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Drawings to start from, built with the same operations a user's clicks make (lib/drawing.mjs), so each is a drawing
// like any other: a pendulum, a two-link arm on a turning waist, a cart on a rail carrying two pendulums, an arm
// reaching out with a two-finger gripper, a four-bar linkage (a closed loop), a pendulum on two ball joints, and a crank
// driving a block along a rail (a loop closed by a slider), and the arm with a gripper on motors, keyframed through a
// pick-and-place cycle.

import { addFrameAt, addJoint, addKeyframe, addSolid, createDrawing, updateJoint, updateSolid } from './drawing.mjs';
import { add, between, length, normalize, scale, subtract } from './math.mjs';

export const examples = Object.freeze([
    { id: 'pendulum', name: 'Pendulum', description: 'An arm on a post, free to swing: let it go and watch it fall and swing back.' },
    { id: 'twoLinkArm', name: 'Two-link arm', description: 'A waist, a shoulder and an elbow, each held by a servo at its setpoint: move the sliders in the canvas.' },
    { id: 'cartPendulums', name: 'Cart and pendulums', description: 'A servo slides a cart along a rail; the two pendulums hanging from it swing as it starts and stops.' },
    { id: 'gripperArm', name: 'Arm with gripper', description: 'A waist, a shoulder, an elbow and a wrist reach out with a two-finger gripper, each held by a servo: run it and the arm turns while the fingers close.' },
    { id: 'fourBar', name: 'Four-bar linkage', description: 'A crank, a coupler and a rocker on a base, the coupler pinned to the rocker to close the loop: a steady torque turns the crank round and the rocker rocks.' },
    { id: 'ballPendulum', name: 'Ball-joint pendulum', description: 'Two links hanging from ball joints, let go out of line with each other, so they swing and twist in three dimensions.' },
    { id: 'pickAndPlace', name: 'Pick and place', description: 'The arm with a gripper on DC motors and a battery, keyframed through a pick-and-place cycle, a 120 g part in its fingers: run it for each motor\'s current and winding temperature and the battery\'s charge.' },
    { id: 'crankSlider', name: 'Crank and slider', description: 'A crank turned by a steady torque pushes a block back and forth along a rail through a connecting rod; the block\'s slider on the rail closes the loop.' }
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

// A box from `from` to `to`, its height along them: `width` across (x, before it turns) and `depth` (y).
function beam(drawing, { name, from, to, width, depth, color }) {
    const along = subtract(to, from);
    return place(drawing, 'box', {
        name, color, size: { x: width, y: depth, z: length(along) },
        position: scale(add(from, to), 0.5), rotation: between([0, 0, 1], along)
    });
}

const degrees = (angle) => angle * Math.PI / 180;

// A unit direction in the x-z plane, `angle` degrees down from straight up towards +x.
const leaning = (angle) => [Math.sin(degrees(angle)), 0, Math.cos(degrees(angle))];

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
    if (id === 'gripperArm') {
        let drawing = createDrawing('Arm with gripper');
        let result = place(drawing, 'cylinder', { name: 'Base', size: { radius: 0.07, length: 0.03 }, position: [0, 0, 0.015], color: '#5f6f80' });
        drawing = result.drawing;
        const base = result.id;
        drawing = join(drawing, 'ground', base, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        result = place(drawing, 'cylinder', { name: 'Turret', size: { radius: 0.045, length: 0.07 }, position: [0, 0, 0.065], color: '#5b8db8' });
        drawing = result.drawing;
        const turret = result.id;
        result = join(drawing, base, turret, [0, 0, 0.03], [0, 0, 1], 'hinge', 'Waist');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.05, drive: { mode: 'servo', setpoint: 0.8, gain: 20, torqueLimit: 2 } });

        // Drawn reaching out: the upper arm leant forward, the forearm reaching on and a little down, the hand tipped down
        // further. Each servo holds the angle it is drawn at (its setpoint 0), but for the waist, which turns the arm.
        const shoulder = [0, 0, 0.1];
        const elbow = add(shoulder, scale(leaning(30), 0.2));
        const wrist = add(elbow, scale(leaning(100), 0.17));
        const reach = normalize(leaning(125));
        result = beam(drawing, { name: 'Upper arm', from: shoulder, to: elbow, width: 0.035, depth: 0.03, color: '#d08b4a' });
        drawing = result.drawing;
        const upper = result.id;
        result = join(drawing, turret, upper, shoulder, [0, 1, 0], 'hinge', 'Shoulder');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.05, limits: { lower: -degrees(70), upper: degrees(70) }, drive: { mode: 'servo', setpoint: 0, gain: 60, torqueLimit: 5 }
        });
        result = beam(drawing, { name: 'Forearm', from: elbow, to: wrist, width: 0.028, depth: 0.025, color: '#e0b04f' });
        drawing = result.drawing;
        const forearm = result.id;
        result = join(drawing, upper, forearm, elbow, [0, 1, 0], 'hinge', 'Elbow');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.03, limits: { lower: -degrees(125), upper: degrees(125) }, drive: { mode: 'servo', setpoint: 0, gain: 25, torqueLimit: 2.5 }
        });
        const palmEnd = add(wrist, scale(reach, 0.03));
        result = beam(drawing, { name: 'Hand', from: wrist, to: palmEnd, width: 0.03, depth: 0.08, color: '#6aa86a' });
        drawing = result.drawing;
        const hand = result.id;
        result = join(drawing, forearm, hand, wrist, [0, 1, 0], 'hinge', 'Wrist');
        drawing = updateJoint(result.drawing, result.id, {
            damping: 0.01, limits: { lower: -degrees(90), upper: degrees(90) }, drive: { mode: 'servo', setpoint: 0, gain: 8, torqueLimit: 1 }
        });
        // Two fingers on sliders across the palm, drawn open; their servos close them towards each other, damped enough
        // (about critically, for a finger's few grams) to close by 15 mm without overshooting into their 20 mm stops, even as the arm turns.
        for (const [name, side] of [['Left finger', 1], ['Right finger', -1]]) {
            const root = add(palmEnd, [0, side * 0.03, 0]);
            result = beam(drawing, { name, from: root, to: add(root, scale(reach, 0.06)), width: 0.012, depth: 0.012, color: '#9aa9b8' });
            drawing = result.drawing;
            const finger = result.id;
            result = join(drawing, hand, finger, root, [0, side, 0], 'slider', `${name} slide`);
            drawing = updateJoint(result.drawing, result.id, {
                damping: 3, limits: { lower: -0.02, upper: 0.005 }, drive: { mode: 'servo', setpoint: -0.015, gain: 200, torqueLimit: 5 }
            });
        }
        return drawing;
    }
    if (id === 'fourBar') {
        let drawing = createDrawing('Four-bar linkage');
        let result = place(drawing, 'box', { name: 'Base', size: { x: 0.26, y: 0.02, z: 0.02 }, position: [0.1, -0.03, 0.2], color: '#5f6f80' });
        drawing = result.drawing;
        const base = result.id;
        drawing = join(drawing, 'ground', base, [0.1, -0.03, 0.19], [0, 0, 1], 'fixed', 'Mount').drawing;
        // A crank-rocker: the crank (60 mm) is the shortest link, beside the base's 200 mm, so it turns all the way round
        // while the 150 mm rocker rocks.
        const crankPivot = [0, 0, 0.2];
        const rockerPivot = [0.2, 0, 0.2];
        const crankPin = add(crankPivot, scale([Math.cos(1.05), 0, Math.sin(1.05)], 0.06));
        const rockerPin = add(rockerPivot, scale([Math.cos(1.75), 0, Math.sin(1.75)], 0.15));
        result = rod(drawing, { name: 'Crank', from: crankPivot, to: crankPin, radius: 0.008, color: '#d08b4a' });
        drawing = result.drawing;
        const crank = result.id;
        result = join(drawing, base, crank, crankPivot, [0, 1, 0], 'hinge', 'Crank pivot');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.01, drive: { mode: 'torque', torque: 0.05 } });
        result = rod(drawing, { name: 'Rocker', from: rockerPivot, to: rockerPin, radius: 0.008, color: '#6aa86a' });
        drawing = result.drawing;
        const rocker = result.id;
        result = join(drawing, base, rocker, rockerPivot, [0, 1, 0], 'hinge', 'Rocker pivot');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.0005 });
        result = rod(drawing, { name: 'Coupler', from: crankPin, to: rockerPin, radius: 0.008, color: '#5b8db8' });
        drawing = result.drawing;
        const coupler = result.id;
        result = join(drawing, crank, coupler, crankPin, [0, 1, 0], 'hinge', 'Coupler pin');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.0005 });
        // The last joint closes the loop: Build holds the coupler's end to the rocker's.
        return join(drawing, coupler, rocker, rockerPin, [0, 1, 0], 'hinge', 'Closing pin').drawing;
    }
    if (id === 'ballPendulum') {
        let drawing = createDrawing('Ball-joint pendulum');
        let result = place(drawing, 'box', { name: 'Post', size: { x: 0.04, y: 0.04, z: 0.3 }, position: [0, 0, 0.15], color: '#7d8fa3' });
        drawing = result.drawing;
        const post = result.id;
        drawing = join(drawing, 'ground', post, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        // The upper link leans out one way, the lower one another, so the two swing out of any one plane.
        const top = [0, 0, 0.3];
        const middle = [0.12, 0.05, 0.22];
        const end = [0.15, 0.17, 0.12];
        result = rod(drawing, { name: 'Upper link', from: top, to: middle, radius: 0.012, color: '#d08b4a' });
        drawing = result.drawing;
        const upper = result.id;
        result = join(drawing, post, upper, top, [0, 0, 1], 'ball', 'Shoulder');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.0005 });
        result = beam(drawing, { name: 'Lower link', from: middle, to: end, width: 0.02, depth: 0.05, color: '#5b8db8' });
        drawing = result.drawing;
        const lower = result.id;
        result = join(drawing, upper, lower, middle, [0, 0, 1], 'ball', 'Elbow');
        return updateJoint(result.drawing, result.id, { damping: 0.0005, friction: 0.0002 });
    }
    if (id === 'pickAndPlace') {
        let drawing = { ...exampleDrawing('gripperArm'), name: 'Pick and place' };
        const named = (name) => drawing.joints.find((joint) => joint.name === name).id;
        // The arm's four joints on motors (each holding where it is drawn until the keyframes move it), the fingers on
        // their servos.
        for (const [name, damping] of [['Waist', 0.02], ['Shoulder', 0.02], ['Elbow', 0.01], ['Wrist', 0.005]]) {
            drawing = updateJoint(drawing, named(name), { damping, drive: { mode: 'motor', setpoint: 0, proportional: 10, derivative: 0.3 } });
        }
        for (const name of ['Left finger slide', 'Right finger slide']) drawing = updateJoint(drawing, named(name), { drive: { setpoint: 0 } });
        // The part, held between the fingers (fixed to the hand: picking it up and letting it go need contact, which comes
        // later).
        const hand = drawing.solids.find((solid) => solid.name === 'Hand');
        const reach = normalize(subtract(drawing.solids.find((solid) => solid.name === 'Left finger').position, hand.position));
        const centre = add(hand.position, scale([reach[0], 0, reach[2]], 0.06));
        const result = place(drawing, 'box', { name: 'Part', size: { x: 0.025, y: 0.025, z: 0.025 }, position: centre, color: '#c9b44a' });
        drawing = updateSolid(result.drawing, result.id, { mass: { by: 'density', density: 7850, material: 'steel' } });
        drawing = join(drawing, hand.id, result.id, centre, [0, 0, 1], 'fixed', 'Grip').drawing;
        // The cycle: turn to the part and reach down, close on it, lift, swing round, set it down, open, and go home.
        const pose = (waist, shoulder, elbow, wrist, fingers) => ({
            [named('Waist')]: waist, [named('Shoulder')]: shoulder, [named('Elbow')]: elbow, [named('Wrist')]: wrist,
            [named('Left finger slide')]: fingers, [named('Right finger slide')]: fingers
        });
        const cycle = [
            [0, pose(0, 0, 0, 0, 0)],
            [1, pose(0.6, 0.25, 0.2, -0.3, 0)],
            [1.5, pose(0.6, 0.25, 0.2, -0.3, -0.008)],
            [2.5, pose(0.6, -0.1, -0.1, 0, -0.008)],
            [3.5, pose(-0.6, -0.1, -0.1, 0, -0.008)],
            [4.5, pose(-0.6, 0.25, 0.2, -0.3, -0.008)],
            [5, pose(-0.6, 0.25, 0.2, -0.3, 0)],
            [6, pose(0, 0, 0, 0, 0)]
        ];
        for (const [time, setpoints] of cycle) drawing = addKeyframe(drawing, { time, setpoints }).drawing;
        return drawing;
    }
    if (id === 'crankSlider') {
        let drawing = createDrawing('Crank and slider');
        const pivot = [0, 0, 0.2];
        const pin = [0.04, 0, 0.23];
        const wrist = [0.18, 0, 0.2];
        let result = rod(drawing, { name: 'Crank', from: pivot, to: pin, radius: 0.008, color: '#d08b4a' });
        drawing = result.drawing;
        const crank = result.id;
        result = join(drawing, 'ground', crank, pivot, [0, 1, 0], 'hinge', 'Crank pivot');
        drawing = updateJoint(result.drawing, result.id, { damping: 0.002, drive: { mode: 'torque', torque: 0.02 } });
        result = rod(drawing, { name: 'Connecting rod', from: pin, to: wrist, radius: 0.008, color: '#5b8db8' });
        drawing = result.drawing;
        const connecting = result.id;
        drawing = join(drawing, crank, connecting, pin, [0, 1, 0], 'hinge', 'Crank pin').drawing;
        result = place(drawing, 'box', { name: 'Block', size: { x: 0.04, y: 0.03, z: 0.03 }, position: wrist, color: '#6aa86a' });
        drawing = result.drawing;
        const block = result.id;
        drawing = join(drawing, connecting, block, wrist, [0, 1, 0], 'hinge', 'Wrist pin').drawing;
        // The rail on a stand, a step further from the Ground than the crank, so the block's slider on it is the joint
        // that closes the loop.
        result = place(drawing, 'box', { name: 'Stand', size: { x: 0.02, y: 0.02, z: 0.15 }, position: [0.3, -0.04, 0.075], color: '#7d8fa3' });
        drawing = result.drawing;
        const stand = result.id;
        drawing = join(drawing, 'ground', stand, [0.3, -0.04, 0], [0, 0, 1], 'fixed', 'Stand mount').drawing;
        result = place(drawing, 'box', { name: 'Rail', size: { x: 0.3, y: 0.01, z: 0.01 }, position: [0.15, -0.04, 0.19], color: '#5f6f80' });
        drawing = result.drawing;
        const rail = result.id;
        drawing = join(drawing, stand, rail, [0.3, -0.04, 0.15], [0, 0, 1], 'fixed', 'Rail mount').drawing;
        return join(drawing, rail, block, wrist, [1, 0, 0], 'slider', 'Guide').drawing;
    }
    throw new Error(`There is no example "${id}".`);
}
