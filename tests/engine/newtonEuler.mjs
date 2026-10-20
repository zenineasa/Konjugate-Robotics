/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A second, independent solution of a drawn robot's motion, to check the built model against: the recursive
// Newton-Euler method for the joint forces, its mass matrix column by column, and a linear solve for the joint
// accelerations. Plain world-frame vectors about each solid's centre, nothing shared with lib/modelBuilder.mjs but
// the drawing's own reading (its tree, poses and masses) and the two numbers a loop's closure is held by.
//
// Each joint's coordinates and speeds, as the built model's nodes hold them:
// - a hinge or a slider: its angle or travel, and its speed;
// - a ball: its turn as a quaternion (in its parent's frame, from where it is drawn), and its angular velocity in its
//   parent's frame;
// - a solid joined to nothing (a free joint on the Ground): its centre and its rotation as a quaternion, in the world,
//   and its spatial velocity (angular velocity, then the velocity of its point passing through the world's origin).
// Stepped as the engine steps each node, one state after another in the node's order: the speeds from the
// accelerations, then the coordinates from the new speeds, each coordinate from the ones updated before it.
//
// A joint's torque follows the drawing: a constant torque, or a servo stiffness times (setpoint minus position)
// clamped to its limit; minus damping times speed and friction times tanh(100 speed), each axis of a ball on its own.
// Joint limits are not modelled here, so drawings checked with it keep inside theirs.
//
// A joint that closes a loop holds its two frames' points together: the force λ there solves
// (J M⁻¹ Jᵀ + εI) λ = -2 ω ė - ω² e - (J q̈₀ + J̇ q̇), where e is the gap between the points, J maps the joint speeds
// to its rate, q̈₀ is the motion without the force, ω the closure's rate and ε its compliance; then q̈ = q̈₀ + M⁻¹ Jᵀ λ.

import { frameById, frameInWorld, groundId, massOf, poseOf, structure, treeOrder } from '../../packages/toolbox/lib/drawing.mjs';
import { add, cross, dot, fromAxisAngle, multiply, normalize, rotate, scale, subtract } from '../../packages/toolbox/lib/math.mjs';
import { closure, closureRate } from '../../packages/toolbox/lib/modelBuilder.mjs';

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

const conjugate = (q) => [q[0], -q[1], -q[2], -q[3]];
const unitQuaternion = (q) => { const size = Math.hypot(...q); return q.map((value) => value / size); };
const speedCount = { hinge: 1, slider: 1, ball: 3, free: 6, fixed: 0 };

// The tree: for each solid in order from the Ground, its parent, its joint's anchor and axis in the parent's frame (as
// the parent stands at rest), its rest pose relative to the parent, its mass and inertia. A solid joined to nothing
// hangs from the Ground by a free joint.
export function treeOf(drawing) {
    const built = structure(drawing);
    const bodies = [];
    const index = new Map([[groundId, -1]]);
    for (const { id } of treeOrder(drawing, built)) {
        const solid = drawing.solids.find((candidate) => candidate.id === id);
        const link = built.parentJoint.get(id);
        const pose = poseOf(drawing, id);
        const common = { id, mass: massOf(solid), unit: unitInertia(solid.shape, solid.size), firstSpeed: 0 };
        if (!link) {
            bodies.push({ ...common, kind: 'free', joint: { id: `free:${id}`, kind: 'free' }, parent: -1, restRotation: pose.rotation, restOffset: pose.position });
            index.set(id, bodies.length - 1);
            continue;
        }
        const { joint, parent } = link;
        const parentPose = poseOf(drawing, parent);
        // The joint's own parent frame gives its point and axis, the child turning about that axis. When that frame is
        // on the solid further from the Ground, it is that solid's parent that turns by the angle, so the solid turns
        // the other way.
        const parentFrame = frameById(drawing, joint.parent);
        const flipped = parentFrame.solid !== parent;
        const anchor = frameInWorld(drawing, parentFrame);
        const inverse = conjugate(parentPose.rotation);
        bodies.push({
            ...common, kind: joint.kind, joint, parent: index.get(parent),
            anchor: rotate(inverse, subtract(anchor.point, parentPose.position)),
            axis: normalize(scale(rotate(inverse, anchor.axis), flipped ? -1 : 1)),
            restRotation: multiply(inverse, pose.rotation),
            restOffset: rotate(inverse, subtract(pose.position, parentPose.position))
        });
        index.set(id, bodies.length - 1);
    }
    const ground = poseOf(drawing, groundId);
    let speeds = 0;
    for (const body of bodies) {
        if (body.parent === -1) body.parentPose = ground;
        body.firstSpeed = speeds;
        speeds += speedCount[body.kind];
    }
    return bodies;
}

// The coordinates every joint starts from: as drawn.
function restCoordinates(bodies) {
    return bodies.map((body) => {
        if (body.kind === 'hinge' || body.kind === 'slider') return [0];
        if (body.kind === 'ball') return [1, 0, 0, 0];
        if (body.kind === 'free') return [...body.restOffset, ...body.restRotation];
        return [];
    });
}

// Each solid's world pose and its joint's world anchor and axis, for coordinates `q` (one array per solid).
function place(bodies, q) {
    const poses = [];
    for (const [i, body] of bodies.entries()) {
        if (body.kind === 'free') {
            poses.push({ position: q[i].slice(0, 3), rotation: unitQuaternion(q[i].slice(3)), anchor: q[i].slice(0, 3), axis: [0, 0, 1] });
            continue;
        }
        const parent = body.parent === -1 ? body.parentPose : poses[body.parent];
        const anchor = add(parent.position, rotate(parent.rotation, body.anchor));
        const axis = rotate(parent.rotation, body.axis);
        let rotation = multiply(parent.rotation, body.restRotation);
        let position = add(parent.position, rotate(parent.rotation, body.restOffset));
        if (body.kind === 'hinge') {
            const turn = fromAxisAngle(axis, q[i][0]);
            rotation = multiply(turn, rotation);
            position = add(anchor, rotate(turn, subtract(position, anchor)));
        } else if (body.kind === 'slider') {
            position = add(position, scale(axis, q[i][0]));
        } else if (body.kind === 'ball') {
            // The turn is in the parent's frame: the world turn is the parent's rotation, then it, then back.
            const turn = multiply(multiply(parent.rotation, unitQuaternion(q[i])), conjugate(parent.rotation));
            rotation = multiply(turn, rotation);
            position = add(anchor, rotate(turn, subtract(position, anchor)));
        }
        poses.push({ position, rotation, anchor, axis, parentRotation: parent.rotation });
    }
    return poses;
}

// The world inertia of a body about its centre, times a vector.
function inertiaTimes(body, pose, vector) {
    const local = rotate(conjugate(pose.rotation), vector);
    return rotate(pose.rotation, local.map((value, axis) => body.mass * body.unit[axis] * value));
}

// Each solid's angular velocity and acceleration, and its centre's velocity and acceleration, for speeds `qd` and
// accelerations `qdd` (each one array over every joint's speeds), with the Ground accelerating upward at `g`.
function motionOf(bodies, poses, qd, qdd, g) {
    const states = [];
    for (const [i, body] of bodies.entries()) {
        const pose = poses[i];
        const speeds = qd.slice(body.firstSpeed, body.firstSpeed + speedCount[body.kind]);
        const rates = qdd.slice(body.firstSpeed, body.firstSpeed + speedCount[body.kind]);
        if (body.kind === 'free') {
            const omega = speeds.slice(0, 3);
            const alpha = rates.slice(0, 3);
            const velocity = add(speeds.slice(3), cross(omega, pose.position));
            const acceleration = add(add(rates.slice(3), cross(alpha, pose.position)), cross(omega, velocity));
            states.push({ omega, alpha, velocity, acceleration: add(acceleration, [0, 0, g]), centre: pose.position });
            continue;
        }
        const parent = body.parent === -1
            ? { omega: [0, 0, 0], alpha: [0, 0, 0], acceleration: [0, 0, g], velocity: [0, 0, 0], centre: body.parentPose.position }
            : states[body.parent];
        // The anchor point moving with the parent.
        const fromParent = subtract(pose.anchor, parent.centre);
        let velocity = add(parent.velocity, cross(parent.omega, fromParent));
        let acceleration = add(parent.acceleration, add(cross(parent.alpha, fromParent), cross(parent.omega, cross(parent.omega, fromParent))));
        let omega = parent.omega;
        let alpha = parent.alpha;
        if (body.kind === 'hinge' || body.kind === 'ball') {
            // The joint's own angular velocity and acceleration in the world: a hinge about its axis; a ball's given in
            // its parent's frame, so turned into the world by the parent's rotation (which turns at the parent's rate).
            const turning = body.kind === 'hinge' ? scale(pose.axis, speeds[0]) : rotate(pose.parentRotation, speeds);
            const rate = body.kind === 'hinge' ? scale(pose.axis, rates[0]) : rotate(pose.parentRotation, rates);
            omega = add(omega, turning);
            alpha = add(alpha, add(cross(parent.omega, turning), rate));
        } else if (body.kind === 'slider') {
            velocity = add(velocity, scale(pose.axis, speeds[0]));
            acceleration = add(acceleration, add(scale(cross(parent.omega, pose.axis), 2 * speeds[0]), scale(pose.axis, rates[0])));
        }
        const toCentre = subtract(pose.position, pose.anchor);
        velocity = add(velocity, cross(omega, toCentre));
        acceleration = add(acceleration, add(cross(alpha, toCentre), cross(omega, cross(omega, toCentre))));
        states.push({ omega, alpha, velocity, acceleration, centre: pose.position });
    }
    return states;
}

// The joint forces that give accelerations `qdd` at coordinates `q` and speeds `qd`, with gravity `g` (downward).
export function inverseDynamics(bodies, q, qd, qdd, g = gravity, poses = place(bodies, q)) {
    const states = motionOf(bodies, poses, qd, qdd, g);
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
        const aboutAnchor = () => add(moment[i], cross(subtract(states[i].centre, poses[i].anchor), force[i]));
        if (body.kind === 'hinge') result.push(dot(poses[i].axis, aboutAnchor()));
        else if (body.kind === 'slider') result.push(dot(poses[i].axis, force[i]));
        else if (body.kind === 'ball') result.push(...rotate(conjugate(poses[i].parentRotation), aboutAnchor()));
        // A free joint's speeds are an angular velocity and the velocity at the origin: the moment about the origin, and
        // the force.
        else if (body.kind === 'free') result.push(...add(moment[i], cross(states[i].centre, force[i])), ...force[i]);
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

function appliedForces(bodies, q, qd) {
    return bodies.flatMap((body, i) => {
        const speeds = qd.slice(body.firstSpeed, body.firstSpeed + speedCount[body.kind]);
        if (body.kind === 'hinge' || body.kind === 'slider') return [jointTorque(body.joint, q[i][0], speeds[0])];
        if (body.kind === 'ball') return speeds.map((speed) => -(body.joint.damping ?? 0) * speed - (body.joint.friction ?? 0) * Math.tanh(100 * speed));
        return speeds.map(() => 0);
    });
}

// The loop a drawing closes, if any: the closing joint's two solids and its frames' points on each, in their own frames.
export function loopOf(drawing, bodies) {
    const [joint] = structure(drawing).loops;
    if (!joint) return null;
    const end = (frameId) => {
        const frame = frameById(drawing, frameId);
        return { body: bodies.findIndex(({ id }) => id === frame.solid), point: frame.point };
    };
    return { joint, a: end(joint.parent), b: end(joint.child) };
}

// A point fixed in a solid (`body` -1 for the Ground): where it is, how fast it moves and how fast that changes.
function pointMotion(end, poses, states, g) {
    if (end.body === -1) return { position: end.point, velocity: [0, 0, 0], acceleration: [0, 0, g] };
    const pose = poses[end.body];
    const state = states[end.body];
    const position = add(pose.position, rotate(pose.rotation, end.point));
    const arm = subtract(position, state.centre);
    return {
        position,
        velocity: add(state.velocity, cross(state.omega, arm)),
        acceleration: add(state.acceleration, add(cross(state.alpha, arm), cross(state.omega, cross(state.omega, arm))))
    };
}

export function forwardDynamics(bodies, q, qd, loop = null, timeStep = 0.001) {
    const n = qd.length;
    const zero = Array(n).fill(0);
    const poses = place(bodies, q);
    const bias = inverseDynamics(bodies, q, qd, zero, gravity, poses);
    const columns = zero.map((_, column) => inverseDynamics(bodies, q, zero, zero.map((__, k) => (k === column ? 1 : 0)), 0, poses));
    // inverseDynamics gives a column of the mass matrix per unit acceleration; it is symmetric. A joint's armature adds
    // to each of its own axes.
    const armature = bodies.flatMap((body) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : body.joint.armature ?? 0));
    const matrix = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? armature[row] : 0)));
    const applied = appliedForces(bodies, q, qd);
    const free = solve(matrix, applied.map((value, k) => value - bias[k]));
    if (!loop) return free;

    // The gap's rate per unit speed of each joint (J's columns), and its acceleration from the speeds alone (J̇ q̇).
    const gapMotion = (speeds, rates, g) => {
        const states = motionOf(bodies, poses, speeds, rates, g);
        const [a, b] = [loop.a, loop.b].map((end) => pointMotion(end, poses, states, g));
        return { gap: subtract(a.position, b.position), rate: subtract(a.velocity, b.velocity), acceleration: subtract(a.acceleration, b.acceleration) };
    };
    const jacobian = zero.map((_, column) => gapMotion(zero.map((__, k) => (k === column ? 1 : 0)), zero, 0).rate);
    const J = [0, 1, 2].map((row) => jacobian.map((column) => column[row]));
    const current = gapMotion(qd, zero, 0);
    const unconstrained = add(current.acceleration, J.map((row) => dot3(row, free)));
    const response = [0, 1, 2].map((column) => solve(matrix, J[column]));
    const K = J.map((row) => response.map((column) => dot3(row, column)));
    const omega = closureRate(timeStep);
    const { compliance } = closure;
    const wanted = [0, 1, 2].map((axis) => -2 * omega * current.rate[axis] - omega * omega * current.gap[axis] - unconstrained[axis]);
    const force = solve(K.map((row, i) => row.map((value, j) => value + (i === j ? compliance : 0))), wanted);
    return free.map((value, k) => value + response.reduce((sum, column, axis) => sum + column[k] * force[axis], 0));
}

const dot3 = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);

// q̇ = ½ [0, ω] ⊗ q, one component at a time from the ones updated before it, as the engine updates a node's states.
function stepQuaternion(q, omega, timeStep) {
    const [w1, w2, w3] = omega;
    q[0] += timeStep * -0.5 * (w1 * q[1] + w2 * q[2] + w3 * q[3]);
    q[1] += timeStep * 0.5 * (w1 * q[0] + w2 * q[3] - w3 * q[2]);
    q[2] += timeStep * 0.5 * (w2 * q[0] + w3 * q[1] - w1 * q[3]);
    q[3] += timeStep * 0.5 * (w3 * q[0] + w1 * q[2] - w2 * q[1]);
}

// One step: the speeds from the accelerations, then each joint's coordinates from its new speeds.
function step(bodies, q, qd, qdd, timeStep) {
    const speeds = qd.map((value, k) => value + timeStep * qdd[k]);
    const coordinates = q.map((values, i) => {
        const body = bodies[i];
        const own = speeds.slice(body.firstSpeed, body.firstSpeed + speedCount[body.kind]);
        const next = [...values];
        if (body.kind === 'hinge' || body.kind === 'slider') next[0] += timeStep * own[0];
        if (body.kind === 'ball') stepQuaternion(next, own, timeStep);
        if (body.kind === 'free') {
            // The centre moves at the velocity at the origin plus ω × the centre.
            const [w1, w2, w3, v1, v2, v3] = own;
            next[0] += timeStep * (v1 + w2 * next[2] - w3 * next[1]);
            next[1] += timeStep * (v2 + w3 * next[0] - w1 * next[2]);
            next[2] += timeStep * (v3 + w1 * next[1] - w2 * next[0]);
            const turn = next.slice(3);
            stepQuaternion(turn, [w1, w2, w3], timeStep);
            next.splice(3, 4, ...turn);
        }
        return next;
    });
    return { q: coordinates, qd: speeds };
}

// Steps the drawing from rest at its drawn pose. Returns, at each of `times`: { [jointId]: coordinates } (a number for
// a hinge or a slider, the quaternion for a ball), { [solidId]: centre }, and `rotations: { [solidId]: quaternion }`;
// a solid joined to nothing has its joint as `free:<solid id>`.
export function simulate(drawing, times, timeStep = 0.001) {
    const bodies = treeOf(drawing);
    const loop = loopOf(drawing, bodies);
    let q = restCoordinates(bodies);
    let qd = Array(bodies.reduce((sum, body) => sum + speedCount[body.kind], 0)).fill(0);
    const out = Object.fromEntries([...bodies.filter(({ kind }) => kind !== 'fixed').map(({ joint }) => [joint.id, []]), ...bodies.map(({ id }) => [id, []])]);
    out.rotations = Object.fromEntries(bodies.map(({ id }) => [id, []]));
    out.gaps = [];
    let time = 0;
    for (const target of times) {
        while (time < target - timeStep / 2) {
            const qdd = forwardDynamics(bodies, q, qd, loop, timeStep);
            ({ q, qd } = step(bodies, q, qd, qdd, timeStep));
            time += timeStep;
        }
        const poses = place(bodies, q);
        bodies.forEach((body, i) => {
            if (body.kind === 'hinge' || body.kind === 'slider') out[body.joint.id].push(q[i][0]);
            else if (body.kind !== 'fixed') out[body.joint.id].push([...q[i]]);
            out[body.id].push(poses[i].position);
            out.rotations[body.id].push(poses[i].rotation);
        });
        if (loop) {
            const at = (end) => (end.body === -1 ? end.point : add(poses[end.body].position, rotate(poses[end.body].rotation, end.point)));
            out.gaps.push(Math.hypot(...subtract(at(loop.a), at(loop.b))));
        }
    }
    return out;
}

