/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A second, independent solution of a drawn robot's motion, to check the built model against: the recursive
// Newton-Euler method for the joint torques, its mass matrix column by column, and a linear solve for the joint
// accelerations. Plain world-frame vectors about each solid's centre, nothing shared with lib/modelBuilder.mjs but
// the drawing's own reading (its tree, poses and masses). Stepped like the built model's nodes: the joint speeds from
// the accelerations, then the joint positions from the new speeds.
//
// A joint's torque follows the drawing: a constant torque, or a servo stiffness times (setpoint minus position)
// clamped to its limit; minus damping times speed and friction times tanh(100 speed). Joint limits are not modelled
// here, so drawings checked with it keep inside theirs.

import { frameById, frameInWorld, groundId, massOf, poseOf, structure, treeOrder } from '../../packages/toolbox/lib/drawing.mjs';
import { add, cross, dot, fromAxisAngle, multiply, normalize, rotate, scale, subtract } from '../../packages/toolbox/lib/math.mjs';

export const gravity = 9.81;

function unitInertia(shape, size) {
    if (shape === 'box') return [(size.y ** 2 + size.z ** 2) / 12, (size.x ** 2 + size.z ** 2) / 12, (size.x ** 2 + size.y ** 2) / 12];
    if (shape === 'sphere') return Array(3).fill(0.4 * size.radius ** 2);
    if (shape === 'cylinder') {
        const across = (3 * size.radius ** 2 + size.length ** 2) / 12;
        return [across, across, size.radius ** 2 / 2];
    }
    // A capsule: a cylinder and two half spheres, each its share of the mass by volume.
    const { radius: r, length: l } = size;
    const middle = Math.PI * r * r * l;
    const ends = (4 / 3) * Math.PI * r ** 3;
    const across = (middle * (l * l / 12 + r * r / 4) + ends * (0.4 * r * r + l * l / 4 + 0.375 * l * r)) / (middle + ends);
    return [across, across, (middle * r * r / 2 + ends * 0.4 * r * r) / (middle + ends)];
}

// The tree: for each solid in order from the Ground, its parent, its joint's anchor and axis in the parent's frame (as
// the parent stands at rest), its rest pose relative to the parent, its mass and inertia.
export function treeOf(drawing) {
    const built = structure(drawing);
    const bodies = [];
    const index = new Map([[groundId, -1]]);
    for (const { id } of treeOrder(drawing, built)) {
        const { joint, parent } = built.parentJoint.get(id);
        const solid = drawing.solids.find((candidate) => candidate.id === id);
        const parentPose = poseOf(drawing, parent);
        const pose = poseOf(drawing, id);
        // The joint's own parent frame gives its point and axis, the child turning about that axis. When that frame is
        // on the solid further from the Ground, it is that solid's parent that turns by the angle, so the solid turns
        // the other way.
        const parentFrame = frameById(drawing, joint.parent);
        const flipped = parentFrame.solid !== parent;
        const anchor = frameInWorld(drawing, parentFrame);
        const inverse = [parentPose.rotation[0], -parentPose.rotation[1], -parentPose.rotation[2], -parentPose.rotation[3]];
        bodies.push({
            id, kind: joint.kind, joint, parent: index.get(parent),
            anchor: rotate(inverse, subtract(anchor.point, parentPose.position)),
            axis: normalize(scale(rotate(inverse, anchor.axis), flipped ? -1 : 1)),
            restRotation: multiply(inverse, pose.rotation),
            restOffset: rotate(inverse, subtract(pose.position, parentPose.position)),
            mass: massOf(solid), unit: unitInertia(solid.shape, solid.size)
        });
        index.set(id, bodies.length - 1);
    }
    return bodies.map((body) => (body.parent === -1 ? { ...body, parentPose: poseOf(drawing, groundId) } : body));
}

const moving = (body) => body.kind === 'hinge' || body.kind === 'slider';

// Each solid's world pose, its joint's world anchor and axis, for joint positions `q` (one per moving joint, in order).
function place(bodies, q) {
    const poses = [];
    let next = 0;
    for (const body of bodies) {
        const parent = body.parent === -1 ? body.parentPose : poses[body.parent];
        const anchor = add(parent.position, rotate(parent.rotation, body.anchor));
        const axis = rotate(parent.rotation, body.axis);
        let rotation = multiply(parent.rotation, body.restRotation);
        let position = add(parent.position, rotate(parent.rotation, body.restOffset));
        let coordinate = 0;
        if (moving(body)) coordinate = q[next++];
        if (body.kind === 'hinge') {
            const turn = fromAxisAngle(axis, coordinate);
            rotation = multiply(turn, rotation);
            position = add(anchor, rotate(turn, subtract(position, anchor)));
        } else if (body.kind === 'slider') {
            position = add(position, scale(axis, coordinate));
        }
        poses.push({ position, rotation, anchor, axis });
    }
    return poses;
}

// The world inertia of a body about its centre, times a vector.
function inertiaTimes(body, pose, vector) {
    const local = rotate([pose.rotation[0], -pose.rotation[1], -pose.rotation[2], -pose.rotation[3]], vector);
    return rotate(pose.rotation, local.map((value, axis) => body.mass * body.unit[axis] * value));
}

// The joint forces that give accelerations `qdd` at positions `q` and speeds `qd`, with gravity `g` (downward).
export function inverseDynamics(bodies, q, qd, qdd, g = gravity) {
    const poses = place(bodies, q);
    const states = [];
    let next = 0;
    for (const [i, body] of bodies.entries()) {
        const pose = poses[i];
        const parent = body.parent === -1
            ? { omega: [0, 0, 0], alpha: [0, 0, 0], acceleration: [0, 0, g], velocity: [0, 0, 0], centre: body.parentPose.position }
            : states[body.parent];
        // The anchor point moving with the parent.
        const fromParent = subtract(pose.anchor, parent.centre);
        let velocity = add(parent.velocity, cross(parent.omega, fromParent));
        let acceleration = add(parent.acceleration, add(cross(parent.alpha, fromParent), cross(parent.omega, cross(parent.omega, fromParent))));
        let omega = parent.omega;
        let alpha = parent.alpha;
        let speed = 0;
        let rate = 0;
        if (moving(body)) { speed = qd[next]; rate = qdd[next]; next += 1; }
        if (body.kind === 'hinge') {
            omega = add(omega, scale(pose.axis, speed));
            alpha = add(alpha, add(cross(parent.omega, scale(pose.axis, speed)), scale(pose.axis, rate)));
        } else if (body.kind === 'slider') {
            velocity = add(velocity, scale(pose.axis, speed));
            acceleration = add(acceleration, add(scale(cross(parent.omega, pose.axis), 2 * speed), scale(pose.axis, rate)));
        }
        const toCentre = subtract(pose.position, pose.anchor);
        velocity = add(velocity, cross(omega, toCentre));
        acceleration = add(acceleration, add(cross(alpha, toCentre), cross(omega, cross(omega, toCentre))));
        states.push({ omega, alpha, velocity, acceleration, centre: pose.position });
    }
    // Back from the tips: each subtree's force and its moment about the body's centre.
    const force = bodies.map((body, i) => scale(states[i].acceleration, body.mass));
    const moment = bodies.map((body, i) => add(inertiaTimes(body, poses[i], states[i].alpha),
        cross(states[i].omega, inertiaTimes(body, poses[i], states[i].omega))));
    for (let i = bodies.length - 1; i >= 0; i -= 1) {
        const parent = bodies[i].parent;
        if (parent === -1) continue;
        force[parent] = add(force[parent], force[i]);
        moment[parent] = add(moment[parent], add(moment[i], cross(subtract(states[i].centre, states[parent].centre), force[i])));
    }
    const result = [];
    for (const [i, body] of bodies.entries()) {
        if (body.kind === 'hinge') result.push(dot(poses[i].axis, add(moment[i], cross(subtract(states[i].centre, poses[i].anchor), force[i]))));
        else if (body.kind === 'slider') result.push(dot(poses[i].axis, force[i]));
    }
    return result;
}

function solve(matrix, vector) {
    const n = vector.length;
    const a = matrix.map((row, i) => [...row, vector[i]]);
    for (let column = 0; column < n; column += 1) {
        let pivot = column;
        for (let row = column + 1; row < n; row += 1) if (Math.abs(a[row][column]) > Math.abs(a[pivot][column])) pivot = row;
        [a[column], a[pivot]] = [a[pivot], a[column]];
        for (let row = 0; row < n; row += 1) {
            if (row === column) continue;
            const factor = a[row][column] / a[column][column];
            for (let k = column; k <= n; k += 1) a[row][k] -= factor * a[column][k];
        }
    }
    return a.map((row, i) => row[n] / row[i]);
}

export function jointTorque(joint, position, speed) {
    const drive = joint.drive ?? { mode: 'free' };
    let torque = 0;
    if (drive.mode === 'torque') torque = drive.torque ?? 0;
    if (drive.mode === 'servo') torque = Math.max(-drive.torqueLimit, Math.min(drive.torqueLimit, drive.gain * (drive.setpoint - position)));
    return torque - (joint.damping ?? 0) * speed - (joint.friction ?? 0) * Math.tanh(100 * speed);
}

export function forwardDynamics(bodies, q, qd) {
    const movers = bodies.filter(moving);
    const zero = movers.map(() => 0);
    const bias = inverseDynamics(bodies, q, qd, zero);
    const mass = movers.map((_, column) => inverseDynamics(bodies, q, zero, zero.map((__, k) => (k === column ? 1 : 0)), 0));
    // inverseDynamics gives a column of the mass matrix per unit acceleration; it is symmetric.
    const matrix = movers.map((body, row) => movers.map((__, column) => mass[column][row] + (row === column ? body.joint.armature ?? 0 : 0)));
    const applied = movers.map((body, k) => jointTorque(body.joint, q[k], qd[k]));
    return solve(matrix, applied.map((value, k) => value - bias[k]));
}

// Steps the drawing from rest at its drawn pose; returns { [jointId]: positions at each of `times` } and the solids'
// centres { [solidId]: [[x, y, z], ...] }.
export function simulate(drawing, times, timeStep = 0.001) {
    const bodies = treeOf(drawing);
    const movers = bodies.filter(moving);
    let q = movers.map(() => 0);
    let qd = movers.map(() => 0);
    const out = Object.fromEntries([...movers.map(({ joint }) => [joint.id, []]), ...bodies.map(({ id }) => [id, []])]);
    let time = 0;
    for (const target of times) {
        while (time < target - timeStep / 2) {
            const qdd = forwardDynamics(bodies, q, qd);
            qd = qd.map((value, k) => value + timeStep * qdd[k]);
            q = q.map((value, k) => value + timeStep * qd[k]);
            time += timeStep;
        }
        movers.forEach(({ joint }, k) => out[joint.id].push(q[k]));
        place(bodies, q).forEach((pose, i) => out[bodies[i].id].push(pose.position));
    }
    return out;
}
