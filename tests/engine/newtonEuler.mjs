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
//   and its angular velocity and its centre's velocity, as MuJoCo's free joint has them.
// Stepped as the engine steps each node, one state after another in the node's order: the speeds from the
// accelerations, then the coordinates from the new speeds, each coordinate from the ones updated before it.
//
// A joint's torque follows the drawing: a constant torque, or a servo stiffness times (setpoint minus position)
// clamped to its limit; minus damping times speed; a ball's friction times tanh(100 speed) on each axis. A hinge's or
// a slider's dry friction and limits are MuJoCo's soft constraints, each row in the regime the step before found: while
// it holds, an inertia D = imp / ((1 - imp) w) on the joint and a force D aref (aref = -B v - K imp r), w the joint's
// inverse inertia where drawn, (M⁻¹)ᵢᵢ of this file's own mass matrix; a slipping friction its friction either way; a
// limit let go, nothing (lib/modelBuilder.mjs's rowReference and impedanceAt give B, K and imp, which are numbers from
// the joint's settings). A ball's limit is not modelled here, so drawings checked with it keep inside theirs.
//
// A joint that closes a loop holds what its kind holds, row by row along the first solid's frame axes (a ball its
// point, a hinge its point and axis, a slider its axis's line and its turn, a fixed joint everything): each row's rate
// c is the second solid's motion relative to the first's at the first's point, e its gap (or twist), and the forces λ of
// every loop together solve (J M⁻¹ Jᵀ + εI) λ = -Ġ q̇-terms - 2 ω c - ω² e - J q̈₀, where J maps the joint speeds to
// the rows' rates, q̈₀ is the motion without the forces, ω the closure's rate and ε its compliance; then
// q̈ = q̈₀ + M⁻¹ Jᵀ λ.

import { frameById, frameInWorld, groundId, massOf, poseOf, structure, treeOrder } from '../../packages/toolbox/lib/drawing.mjs';
import { add, cross, dot, fromAxisAngle, multiply, normalize, rotate, scale, subtract } from '../../packages/toolbox/lib/math.mjs';
import { closure, closureFrame, closureRate, impedanceAt, rowReference, softnessOf } from '../../packages/toolbox/lib/modelBuilder.mjs';

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
        const common = { id, mass: massOf(solid), unit: solid.shape === 'mesh' ? solid.unitInertia : unitInertia(solid.shape, solid.size), firstSpeed: 0 };
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
export function restCoordinates(bodies) {
    return bodies.map((body) => {
        if (body.kind === 'hinge' || body.kind === 'slider') return [0];
        if (body.kind === 'ball') return [1, 0, 0, 0];
        if (body.kind === 'free') return [...body.restOffset, ...body.restRotation];
        return [];
    });
}

// Each solid's world pose and its joint's world anchor and axis, for coordinates `q` (one array per solid).
export function place(bodies, q) {
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
            states.push({ omega: speeds.slice(0, 3), alpha: rates.slice(0, 3), velocity: speeds.slice(3), acceleration: add(rates.slice(3), [0, 0, g]), centre: pose.position });
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
        // A free joint's speeds are an angular velocity and its centre's velocity: the moment about the centre, and the
        // force.
        else if (body.kind === 'free') result.push(...moment[i], ...force[i]);
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

// `setpoint` is the servo's setpoint now (its keyframes' schedule, or its drive's), `motorTorque` a motor's torque now.
export function jointTorque(joint, position, speed, setpoint = joint.drive?.setpoint, motorTorque = 0) {
    const drive = joint.drive ?? { mode: 'free' };
    let torque = 0;
    if (drive.mode === 'torque') torque = drive.torque ?? 0;
    if (drive.mode === 'servo') torque = Math.max(-drive.torqueLimit, Math.min(drive.torqueLimit, servoRaw(drive, position, speed, setpoint)));
    if (drive.mode === 'motor') torque = motorTorque;
    return torque - (joint.damping ?? 0) * speed;
}

// A servo's torque before its limit: its stiffness times the setpoint left to reach, less its own damping times the
// speed.
const servoRaw = (drive, position, speed, setpoint) => drive.gain * (setpoint - position) - (drive.damping ?? 0) * speed;
// A servo's own damping, taken at the step's end speed while it is within its limit, as the joint's damping is.
function servoStepDamping(body, q, qd, inputs) {
    const drive = body.joint?.drive;
    if (!(body.kind === 'hinge' || body.kind === 'slider') || drive?.mode !== 'servo' || !(drive.damping > 0)) return 0;
    const raw = servoRaw(drive, q[0], qd[body.firstSpeed], inputs.setpoints?.[body.joint.id] ?? drive.setpoint);
    return Math.abs(raw) <= drive.torqueLimit ? drive.damping : 0;
}

// A ball's drive, about its parent's axes: a constant torque, or a servo turning it towards its setpoint (a turn
// vector): the turn left to make is the quaternion q ⊗ d*, of angle θ about n, and the servo gives -stiffness sin θ n,
// held to its most torque.
function ballDrive(joint, turn) {
    const drive = joint.drive ?? { mode: 'free' };
    if (drive.mode === 'torque') return drive.torque;
    if (drive.mode !== 'servo') return [0, 0, 0];
    const angle = Math.hypot(...drive.setpoint);
    const wanted = angle > 1e-9 ? [Math.cos(angle / 2), ...scale(drive.setpoint, Math.sin(angle / 2) / angle)] : [1, ...scale(drive.setpoint, 0.5)];
    const left = multiply(turn, conjugate(wanted));
    const torque = scale(left.slice(1), -2 * drive.gain * left[0]);
    return scale(torque, Math.min(1, drive.torqueLimit / Math.max(Math.hypot(...torque), 1e-12)));
}

function appliedForces(bodies, q, qd, inputs) {
    return bodies.flatMap((body, i) => {
        const speeds = qd.slice(body.firstSpeed, body.firstSpeed + speedCount[body.kind]);
        if (body.kind === 'hinge' || body.kind === 'slider') {
            return [jointTorque(body.joint, q[i][0], speeds[0], inputs.setpoints?.[body.joint.id] ?? body.joint.drive?.setpoint, inputs.motorTorques?.[body.joint.id] ?? 0)];
        }
        if (body.kind === 'ball') {
            const drive = ballDrive(body.joint, q[i]);
            return speeds.map((speed, axis) => drive[axis] - (body.joint.damping ?? 0) * speed - (body.joint.friction ?? 0) * Math.tanh(100 * speed));
        }
        return speeds.map(() => 0);
    });
}

// The rows each kind of closing joint holds: [linear or angular, which of the first solid's axes (t1, t2, n)].
const closureRows = {
    ball: [['linear', 0], ['linear', 1], ['linear', 2]],
    hinge: [['linear', 0], ['linear', 1], ['linear', 2], ['angular', 0], ['angular', 1]],
    slider: [['linear', 0], ['linear', 1], ['angular', 0], ['angular', 1], ['angular', 2]],
    fixed: [['linear', 0], ['linear', 1], ['linear', 2], ['angular', 0], ['angular', 1], ['angular', 2]]
};

// The loops a drawing closes: each closing joint's two solids (-1 for the Ground), its frames' points on each in their
// own frames, the first frame's axes, the rest turn of the second solid relative to the first, and its rows.
export function loopsOf(drawing, bodies) {
    return structure(drawing).loops.map((joint) => {
        const [first, second] = [joint.parent, joint.child].map((frameId) => {
            const frame = frameById(drawing, frameId);
            return { body: bodies.findIndex(({ id }) => id === frame.solid), point: frame.point, rotation: poseOf(drawing, frame.solid).rotation };
        });
        return {
            joint, first, second, rows: closureRows[joint.kind],
            axes: closureFrame(normalize(frameById(drawing, joint.parent).axis)),
            rest: multiply(conjugate(first.rotation), second.rotation)
        };
    });
}

// A solid's pose and its spatial velocity and acceleration (angular, then at the world's origin); the Ground's is still.
function spatialOf(index, poses, states) {
    if (index === -1) return { rotation: [1, 0, 0, 0], position: [0, 0, 0], omega: [0, 0, 0], origin: [0, 0, 0], alpha: [0, 0, 0], originAcceleration: [0, 0, 0] };
    const pose = poses[index];
    const { omega, alpha, velocity, acceleration, centre } = states[index];
    return {
        rotation: pose.rotation, position: pose.position, omega, alpha,
        origin: subtract(velocity, cross(omega, centre)),
        originAcceleration: subtract(subtract(acceleration, cross(alpha, centre)), cross(omega, velocity))
    };
}

// A loop's rows at speeds `qd` and accelerations `qdd` (no gravity): each row's rate, its Ġ Δ, its acceleration and its
// gap or twist.
function loopRows(loop, poses, states) {
    const a = spatialOf(loop.first.body, poses, states);
    const b = spatialOf(loop.second.body, poses, states);
    const point = add(a.position, rotate(a.rotation, loop.first.point));
    const otherPoint = add(b.position, rotate(b.rotation, loop.second.point));
    const dOmega = subtract(b.omega, a.omega);
    const dOrigin = subtract(b.origin, a.origin);
    const dAlpha = subtract(b.alpha, a.alpha);
    const dOriginAcceleration = subtract(b.originAcceleration, a.originAcceleration);
    const atPoint = add(dOrigin, cross(dOmega, point));
    const pointRate = add(a.origin, cross(a.omega, point));
    // Q = R₂ (R₁ R₀)ᵀ, as a rotation; its small-angle vector is ½ vee(Q - Qᵀ), the vector part of its quaternion times
    // its scalar part, twice.
    const turn = multiply(b.rotation, conjugate(multiply(a.rotation, loop.rest)));
    const twist = scale(turn.slice(1), 2 * turn[0]);
    return loop.rows.map(([type, index]) => {
        const axis = rotate(a.rotation, loop.axes[index]);
        const spin = cross(a.omega, axis);
        if (type === 'angular') return { rate: dot(axis, dOmega), drift: dot(spin, dOmega), acceleration: dot(axis, dAlpha), error: dot(axis, twist) };
        return {
            rate: dot(axis, atPoint),
            drift: dot(spin, atPoint) + dot(axis, cross(dOmega, pointRate)),
            acceleration: dot(axis, add(dOriginAcceleration, cross(dAlpha, point))),
            error: dot(axis, subtract(otherPoint, point))
        };
    });
}

export function forwardDynamics(bodies, q, qd, loops = [], timeStep = 0.001, inputs = {}) {
    const n = qd.length;
    const zero = Array(n).fill(0);
    const poses = place(bodies, q);
    const bias = inverseDynamics(bodies, q, qd, zero, gravity, poses);
    const columns = zero.map((_, column) => inverseDynamics(bodies, q, zero, zero.map((__, k) => (k === column ? 1 : 0)), 0, poses));
    // inverseDynamics gives a column of the mass matrix per unit acceleration; it is symmetric. A joint's armature adds
    // to each of its own axes, and so does its damping times the step: the damping taken at the step's end speed.
    // A motor's rotor adds its inertia times the gear ratio squared.
    const rotor = (joint) => (joint.drive?.mode === 'motor' ? joint.drive.motor.rotorInertia * joint.drive.motor.gearRatio ** 2 : 0);
    const armature = bodies.flatMap((body, i) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : (body.joint.armature ?? 0) + rotor(body.joint) + ((body.joint.damping ?? 0) + servoStepDamping(body, q[i], qd, inputs)) * timeStep));
    // Each hinge's or slider's soft rows, in their regimes: the inertia each adds to its joint and the force it puts on.
    const rows = softRows(bodies, q, qd, inputs.regimes ?? new Map(), inputs.invweight ?? [], timeStep);
    for (const row of rows) armature[row.speed] += row.inertia;
    const matrix = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? armature[row] : 0)));
    const applied = appliedForces(bodies, q, qd, inputs);
    for (const row of rows) applied[row.speed] += row.torque;
    const free = solve(matrix, applied.map((value, k) => value - bias[k]));
    if (!loops.length) return { qdd: free, rows };

    // Every loop's rows: J's rows (each row's rate per unit speed of each joint), and, at the speeds, Ġ Δ and the gap,
    // and at q̈₀ the rows' accelerations.
    const rowsAt = (speeds, rates) => loops.flatMap((loop) => loopRows(loop, poses, motionOf(bodies, poses, speeds, rates, 0)));
    const unit = zero.map((_, column) => rowsAt(zero.map((__, k) => (k === column ? 1 : 0)), zero).map(({ rate }) => rate));
    const J = unit[0].map((_, row) => unit.map((column) => column[row]));
    const current = rowsAt(qd, free);
    const omega = closureRate(timeStep);
    const wanted = current.map(({ rate, drift, acceleration, error }) => -drift - 2 * omega * rate - omega * omega * error - acceleration);
    const response = J.map((row) => solve(matrix, row));
    const K = J.map((row, i) => response.map((column) => dot3(row, column) + (i === response.indexOf(column) ? closure.compliance : 0)));
    const force = solve(K, wanted);
    return { qdd: free.map((value, k) => value + response.reduce((sum, column, row) => sum + column[k] * force[row], 0)), rows };
}

// A hinge's or a slider's soft rows now: its dry friction (if it has one) and each limit it is past, each with the
// inertia and force its regime puts on the joint, and how to find its next regime from the joint's acceleration.
function softRows(bodies, q, qd, regimes, invweight, timeStep) {
    const rows = [];
    bodies.forEach((body, i) => {
        if (!(body.kind === 'hinge' || body.kind === 'slider')) return;
        const { joint } = body;
        const speed = body.firstSpeed;
        const v = qd[speed];
        const soft = softnessOf(joint);
        if (joint.friction > 0) {
            const { B } = rowReference(soft.friction, timeStep);
            const imp = impedanceAt(soft.friction.solimp, 0);
            const D = imp / ((1 - imp) * invweight[speed]);
            const regime = regimes.get(`friction:${joint.id}`) ?? 2;
            const holding = Math.abs(regime) < 0.5;
            rows.push({
                key: `friction:${joint.id}`, speed, inertia: holding ? D : 0,
                torque: holding ? -D * B * v : regime > 1.5 ? 0 : regime > 0.5 ? joint.friction : -joint.friction,
                next: (qdd) => { const wanted = D * (-B * v - qdd); return wanted > joint.friction ? 1 : wanted < -joint.friction ? -1 : 0; }
            });
        }
        if (joint.limits) {
            const { B, K } = rowReference(soft.limit, timeStep);
            for (const [side, sign] of [['lower', 1], ['upper', -1]]) {
                const r = sign > 0 ? q[i][0] - joint.limits.lower - soft.margin : joint.limits.upper - q[i][0] - soft.margin;
                const imp = impedanceAt(soft.limit.solimp, r);
                const D = imp / ((1 - imp) * invweight[speed]);
                const reference = -B * sign * v - K * imp * r;
                const holding = r < 0 && (regimes.get(`${side}:${joint.id}`) ?? 1) > 0.5;
                rows.push({
                    key: `${side}:${joint.id}`, speed, inertia: holding ? D : 0, torque: holding ? sign * D * reference : 0,
                    next: (qdd) => (r < 0 ? (sign * qdd - reference < 0 ? 1 : 0) : 1)
                });
            }
        }
    });
    return rows;
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
            const [w1, w2, w3, v1, v2, v3] = own;
            next[0] += timeStep * v1;
            next[1] += timeStep * v2;
            next[2] += timeStep * v3;
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
    const loops = loopsOf(drawing, bodies);
    let q = restCoordinates(bodies);
    let qd = Array(bodies.reduce((sum, body) => sum + speedCount[body.kind], 0)).fill(0);
    // The soft rows' regimes, by row, as the step before found them; and each joint's inverse inertia where drawn.
    const regimes = new Map();
    const invweight = (() => {
        const zero = Array(qd.length).fill(0);
        const columns = zero.map((_, column) => inverseDynamics(bodies, q, zero, zero.map((__, k) => (k === column ? 1 : 0)), 0));
        const rotor = (joint) => (joint.drive?.mode === 'motor' ? joint.drive.motor.rotorInertia * joint.drive.motor.gearRatio ** 2 : 0);
        const armature = bodies.flatMap((body) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : (body.joint.armature ?? 0) + rotor(body.joint)));
        const M0 = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? armature[row] : 0)));
        return zero.map((_, k) => solve(M0, zero.map((__, j) => (j === k ? 1 : 0)))[k]);
    })();
    const out = Object.fromEntries([...bodies.filter(({ kind }) => kind !== 'fixed').map(({ joint }) => [joint.id, []]), ...bodies.map(({ id }) => [id, []])]);
    out.rotations = Object.fromEntries(bodies.map(({ id }) => [id, []]));
    out.gaps = [];
    // Motors and their battery: each motor's winding temperature and the battery's charge, stepped beside the joints.
    const motors = bodies.filter((body) => (body.kind === 'hinge' || body.kind === 'slider') && body.joint.drive?.mode === 'motor');
    const battery = { fullVoltage: 12.6, emptyVoltage: 9.9, capacity: 7920, ...(drawing.battery ?? {}) };
    let charge = 100;
    const temperatures = Object.fromEntries(motors.map(({ joint }) => [joint.id, joint.drive.motor.ambientTemperature]));
    out.motors = Object.fromEntries(motors.map(({ joint }) => [joint.id, { duty: [], current: [], torque: [], voltage: [], temperature: [] }]));
    out.charge = [];
    // Each keyframed joint's setpoint at a time: in straight lines between its keyframes, the nearest one's beyond them.
    const keyframes = [...(drawing.keyframes ?? [])].sort((a, b) => a.time - b.time);
    const scheduleOf = (jointId) => keyframes.filter(({ setpoints }) => Number.isFinite(setpoints[jointId])).map(({ time: at, setpoints }) => [at, setpoints[jointId]]);
    const schedules = Object.fromEntries(bodies.filter(({ kind, joint }) => (kind === 'hinge' || kind === 'slider') && ['servo', 'motor'].includes(joint.drive?.mode))
        .map(({ joint }) => [joint.id, scheduleOf(joint.id)]).filter(([, samples]) => samples.length));
    const lookup = (samples, at) => {
        if (at <= samples[0][0]) return samples[0][1];
        if (at >= samples.at(-1)[0]) return samples.at(-1)[1];
        const after = samples.findIndex(([sampleTime]) => sampleTime > at);
        const [[t0, v0], [t1, v1]] = [samples[after - 1], samples[after]];
        return v0 + (v1 - v0) * (at - t0) / (t1 - t0);
    };
    // The motors' duty, current and torque now, from the joints' positions and speeds and the battery's voltage.
    // The supply sags under load: through its internal resistance, and held to its most current (each motor's current
    // linear in the voltage at its terminals, the supply giving G V - J; see lib/modelBuilder.mjs).
    const motorsNow = (at) => {
        const each = motors.map((body) => {
            const { joint } = body;
            const index = bodies.indexOf(body);
            const position = q[index][0];
            const speed = qd[body.firstSpeed];
            const setpoint = schedules[joint.id] ? lookup(schedules[joint.id], at) : joint.drive.setpoint ?? 0;
            const duty = Math.max(-1, Math.min(1, joint.drive.proportional * (setpoint - position) - joint.drive.derivative * speed));
            return { joint, duty, backEmf: joint.drive.motor.torqueConstant * joint.drive.motor.gearRatio * speed };
        });
        const open = battery.emptyVoltage + (battery.fullVoltage - battery.emptyVoltage) * charge / 100;
        const { internalResistance = 0, currentLimit = 0 } = battery;
        let voltage = open;
        if (internalResistance > 0 || currentLimit > 0) {
            const G = each.reduce((sum, { joint, duty }) => sum + duty * duty / joint.drive.motor.resistance, 0);
            const J = each.reduce((sum, { joint, duty, backEmf }) => sum + duty * backEmf / joint.drive.motor.resistance, 0);
            voltage = (open + internalResistance * J) / (1 + internalResistance * G);
            if (currentLimit > 0 && G * voltage - J > currentLimit) voltage = (currentLimit + J) / G;
        }
        return Object.fromEntries(each.map(({ joint, duty, backEmf }) => {
            const { resistance, torqueConstant, gearRatio, gearEfficiency = 1 } = joint.drive.motor;
            const current = (duty * voltage - backEmf) / resistance;
            return [joint.id, { duty, current, voltage, torque: gearEfficiency * gearRatio * torqueConstant * current }];
        }));
    };
    let time = 0;
    for (const target of times) {
        while (time < target - timeStep / 2) {
            const setpoints = Object.fromEntries(Object.entries(schedules).map(([jointId, samples]) => [jointId, lookup(samples, time)]));
            const now = motorsNow(time);
            const motorTorques = Object.fromEntries(Object.entries(now).map(([jointId, { torque }]) => [jointId, torque]));
            const { qdd, rows } = forwardDynamics(bodies, q, qd, loops, timeStep, { setpoints, motorTorques, regimes, invweight });
            for (const row of rows) regimes.set(row.key, row.next(qdd[row.speed]));
            ({ q, qd } = step(bodies, q, qd, qdd, timeStep));
            for (const { joint } of motors) {
                const { resistance, thermalResistance, thermalCapacitance, ambientTemperature } = joint.drive.motor;
                const { current } = now[joint.id];
                temperatures[joint.id] += timeStep * (resistance * current * current - (temperatures[joint.id] - ambientTemperature) / thermalResistance) / thermalCapacitance;
            }
            charge += timeStep * Object.values(now).reduce((sum, { duty, current }) => sum - 100 * duty * current / battery.capacity, 0);
            time += timeStep;
        }
        const now = motorsNow(time);
        for (const { joint } of motors) {
            for (const key of ['duty', 'current', 'torque', 'voltage']) out.motors[joint.id][key].push(now[joint.id][key]);
            out.motors[joint.id].temperature.push(temperatures[joint.id]);
        }
        out.charge.push(charge);
        const poses = place(bodies, q);
        bodies.forEach((body, i) => {
            if (body.kind === 'hinge' || body.kind === 'slider') out[body.joint.id].push(q[i][0]);
            else if (body.kind !== 'fixed') out[body.joint.id].push([...q[i]]);
            out[body.id].push(poses[i].position);
            out.rotations[body.id].push(poses[i].rotation);
        });
        // The largest gap left at any loop's point (along what it holds).
        if (loops.length) {
            const states = motionOf(bodies, poses, qd, qd.map(() => 0), 0);
            out.gaps.push(Math.max(...loops.map((loop) => Math.hypot(...loopRows(loop, poses, states).filter((_, k) => loop.rows[k][0] === 'linear').map(({ error }) => error)))));
        }
    }
    return out;
}

