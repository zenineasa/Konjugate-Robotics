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
// clamped to its limit; minus damping times speed; a hinge's or a slider's spring, -stiffness (q - rest); a ball's friction times tanh(100 speed) on each axis. A ball's limit
// is not modelled here, so drawings checked with it keep inside theirs.
//
// A joint that closes a loop holds what its kind holds, row by row along the first solid's frame axes (a ball its
// point, a hinge its point and axis, a slider its axis's line and its turn, a fixed joint everything): each row's rate
// c is the second solid's motion relative to the first's at the first's point, e its gap (or twist), and the forces λ of
// every loop together solve (J M⁻¹ Jᵀ + εI) λ = -Ġ q̇-terms - 2 ω c - ω² e - J q̈₀, where J maps the joint speeds to
// the rows' rates, q̈₀ is the motion without the forces, ω the closure's rate and ε its compliance; then
// q̈ = q̈₀ + M⁻¹ Jᵀ λ, M with each joint's damping times the step on its diagonal.
//
// A hinge's or a slider's dry friction and limits are MuJoCo's soft constraints, as MuJoCo finds them: rows of
// generalized force (a friction held to ±its friction, a limit pushing only while past either end), each with its
// compliance R = (1 - imp) w / imp (w the joint's inverse inertia where drawn, (M⁻¹)ᵢᵢ of this file's own mass matrix)
// and its reference acceleration aref = -B v - K imp r (lib/modelBuilder.mjs's rowReference and impedanceAt give B,
// K and imp, numbers from the joint's settings). With any such row, the rows' and the loops' forces are found from M
// without the damping: (A + R) f = aref - J q̈₀ within the rows' bounds, A = J M⁻¹ Jᵀ, the loops' rows taken out
// exactly and the rest by projected Gauss-Seidel from the step before's forces, then polished (lib/modelBuilder.mjs's
// constraintSweeps); then q̈ = (M + h B)⁻¹ (τ + Jᵀ f), the damping B taken at the step's end speed.
//
// A coupling (q₁ = c₀ + c₁ q₂ + ... + c₄ q₂⁴, MuJoCo's joint equality) is a row of its own the same way, unbounded,
// J = [1 at q₁, -p'(q₂) at q₂], r = q₁ - p(q₂), w the two joints' inverse inertias summed as MuJoCo sums them. A fixed
// tendon (L = offset + Σ cᵢ qᵢ, MuJoCo's fixed tendon) pulls each of its joints by cᵢ F, F its spring, its damping and its
// drive (a force, or a servo kp (setpoint - L) - kv L̇ held to its most force); its dry friction and its limit are rows
// with J = ±c and w = cᵀ M⁻¹ c where drawn; its damping and its servo's kv (within its most force) add b c cᵀ to B,
// as MuJoCo's implicitfast adds it: only where M has an entry, two joints side by side (a gripper's fingers) not
// coupled.

import { frameById, frameInWorld, groundId, massOf, poseOf, structure, treeOrder } from '../../packages/toolbox/lib/drawing.mjs';
import { add, cross, dot, fromAxisAngle, multiply, normalize, rotate, scale, subtract } from '../../packages/toolbox/lib/math.mjs';
import { closure, closureFrame, closureRate, constraintPolishes, constraintSweeps, impedanceAt, rowReference, softnessOf } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { couplingsOf, tendonsOf } from '../../packages/toolbox/lib/drawing.mjs';
import { collide, numbers } from '../../packages/toolbox/lib/collision.mjs';
import { contactPairs, floorSurfaceOf, leastFriction, surfaceOf } from '../../packages/toolbox/lib/contact.mjs';

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
    // A spring, pulling it to its rest.
    const spring = joint.stiffness > 0 ? -joint.stiffness * (position - (joint.springRest ?? 0)) : 0;
    return torque - (joint.damping ?? 0) * speed + spring;
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
    const own = bodies.flatMap((body) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : (body.joint.armature ?? 0) + rotor(body.joint)));
    const damping = bodies.flatMap((body, i) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : (body.joint.damping ?? 0) + servoStepDamping(body, q[i], qd, inputs)));
    const applied = appliedForces(bodies, q, qd, inputs);
    // The tendons: each pulls its joints by cᵢ F; its damping and its servo's kv add b c cᵀ to the damping.
    const tendonDamping = zero.map(() => [...zero]);
    for (const tendon of inputs.tendons ?? []) {
        const { force, implicit } = tendonNow(tendon, q, qd);
        tendon.c.forEach((c, k) => { applied[k] += c * force; });
        // As MuJoCo's implicitfast takes it: only where the mass matrix has an entry (the two joints one above the
        // other in the tree, or one joint), its sparsity; two fingers side by side are each damped alone.
        if (implicit) tendon.c.forEach((ci, i) => tendon.c.forEach((cj, j) => { if (inputs.related(i, j)) tendonDamping[i][j] += implicit * ci * cj; }));
    }
    const net = applied.map((value, k) => value - bias[k]);
    const dampingMatrix = (row, column) => (row === column ? damping[row] : 0) + tendonDamping[row][column];
    const loopJ = () => {
        const rowsAt = (speeds, rates) => loops.flatMap((loop) => loopRows(loop, poses, motionOf(bodies, poses, speeds, rates, 0)));
        const unit = zero.map((_, column) => rowsAt(zero.map((__, k) => (k === column ? 1 : 0)), zero).map(({ rate }) => rate));
        const J = unit.length ? unit[0].map((_, row) => unit.map((column) => column[row])) : [];
        const omega = closureRate(timeStep);
        return { J, wanted: (a0) => rowsAt(qd, a0).map(({ rate, drift, acceleration, error }) => -drift - 2 * omega * rate - omega * omega * error - acceleration) };
    };
    if (!inputs.soft) {
        const matrix = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? own[row] : 0) + dampingMatrix(row, column) * timeStep));
        const free = solve(matrix, net);
        if (!loops.length) return { qdd: free };
        const { J, wanted: wantedAt } = loopJ();
        const wanted = wantedAt(free);
        const response = J.map((row) => solve(matrix, row));
        const K = J.map((row, i) => response.map((column) => dot3(row, column) + (i === response.indexOf(column) ? closure.compliance : 0)));
        const force = solve(K, wanted);
        return { qdd: free.map((value, k) => value + response.reduce((sum, column, row) => sum + column[k] * force[row], 0)) };
    }
    // With joint rows, as lib/modelBuilder.mjs finds them at the Ground: from the mass matrix alone, the loops' rows and
    // the constraint rows (the loops' taken out exactly, the constraint rows by projected Gauss-Seidel from the step
    // before's forces, then polished); then the step's acceleration with the damping taken at its end speed, those
    // forces on.
    const M0 = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? own[row] : 0)));
    const a0 = solve(M0, net);
    const loop = loops.length ? loopJ() : { J: [], wanted: () => [] };
    const rows = [
        ...loop.J.map((J, k) => ({ type: 'loop', J, wanted: null, index: k })),
        ...jointRows(bodies, q, qd, inputs.invweight, timeStep, n),
        ...couplingRows(inputs.couplings ?? [], q, qd, timeStep, n),
        ...tendonRows(inputs.tendons ?? [], q, qd, timeStep),
        ...contactRows(inputs.contacts ?? [], bodies, poses, qd, timeStep)
    ];
    const loopWanted = loop.wanted(a0);
    rows.forEach((row, index) => {
        row.index = index;
        if (row.type === 'loop') row.wanted = loopWanted[index];
        else row.wanted = row.reference - dot3(row.J, a0);
    });
    const response = rows.map((row) => solve(M0, row.J));
    const K = rows.map((row) => response.map((column) => dot3(row.J, column)));
    const held = rows.filter(({ type }) => type === 'loop').map(({ index }) => index);
    const bounded = rows.filter(({ type }) => type === 'friction' || type === 'limit' || type === 'equality' || type === 'contact').map(({ index }) => index);
    const forces = rows.map(() => 0);
    let A = (r, c) => K[r][c];
    let b = (r) => rows[r].wanted;
    let held0 = [];
    let heldPer = new Map();
    if (held.length) {
        const KE = held.map((i) => held.map((j) => K[i][j] + (i === j ? closure.compliance : 0)));
        held0 = solve(KE, held.map((i) => rows[i].wanted));
        heldPer = new Map(bounded.map((c) => [c, solve(KE, held.map((i) => K[i][c]))]));
        A = (r, c) => K[r][c] - held.reduce((sum, e, k) => sum + K[r][e] * heldPer.get(c)[k], 0);
        b = (r) => rows[r].wanted - held.reduce((sum, e, k) => sum + K[r][e] * held0[k], 0);
    }
    const f = new Map(bounded.map((r) => [r, inputs.warm?.get(r) ?? 0]));
    for (let sweep = 0; sweep < (inputs.sweeps ?? constraintSweeps); sweep += 1) {
        for (const r of bounded) {
            const { compliance, low, high } = rows[r];
            const residual = b(r) - bounded.reduce((sum, c) => sum + A(r, c) * f.get(c), 0) - compliance * f.get(r);
            f.set(r, Math.max(low, Math.min(high, f.get(r) + residual / (A(r, r) + compliance))));
        }
    }
    // Polishing, as an active set: each row's next value were it alone (its force plus its residual over its own
    // coupling and compliance) says whether it is inside its bounds; the rows inside are solved exactly, the others held
    // at the bound that value reaches. Taken where consistent (each solved force inside its bounds, each held one
    // pushing against its bound); tried again from that solution, and the sweeps' forces where no try is.
    const polish = (start) => {
        const residualOf = (r, values) => b(r) - bounded.reduce((sum, c) => sum + A(r, c) * values.get(c), 0) - rows[r].compliance * values.get(r);
        const next = new Map(bounded.map((r) => [r, start.get(r) + residualOf(r, start) / (A(r, r) + rows[r].compliance)]));
        const inside = new Map(bounded.map((r) => [r, next.get(r) > rows[r].low && next.get(r) < rows[r].high]));
        const held = new Map(bounded.map((r) => [r, Math.max(rows[r].low, Math.min(rows[r].high, next.get(r)))]));
        const values = bounded.length ? solve(
            bounded.map((i) => bounded.map((j) => (i === j ? (inside.get(i) ? A(i, i) + rows[i].compliance : 1) : inside.get(i) && inside.get(j) ? A(i, j) : 0))),
            bounded.map((i) => (inside.get(i) ? b(i) - bounded.reduce((sum, j) => sum + (j !== i && !inside.get(j) ? A(i, j) * held.get(j) : 0), 0) : held.get(i)))
        ) : [];
        const solved = new Map(bounded.map((r, k) => [r, values[k]]));
        const fits = bounded.every((r, k) => {
            if (!(rows[r].high - rows[r].low > 1e-300)) return true;
            if (inside.get(r)) return values[k] >= rows[r].low && values[k] <= rows[r].high;
            const residual = residualOf(r, solved);
            return held.get(r) > rows[r].low ? residual >= 0 : residual <= 0;
        });
        return { values, fits, solved };
    };
    const tries = [polish(f)];
    while (tries.length < (inputs.polishes ?? constraintPolishes)) tries.push(polish(tries.at(-1).solved));
    const taken = tries.find(({ fits }) => fits);
    bounded.forEach((r, k) => { forces[r] = taken ? taken.values[k] : f.get(r); });
    if (process.env.DUMPALL) console.log('QDD', inputs.time.toFixed(4), solve(zero.map((__, row) => zero.map((___, column) => M0[row][column] + dampingMatrix(row, column) * timeStep)), net.map((value, k) => value + rows.reduce((sum, row, r) => sum + row.J[k] * forces[r], 0))).join(' '), 'taken', tries.indexOf(taken));
    if (process.env.DUMP && Math.abs(inputs.time - Number(process.env.DUMP)) < 0.0015) console.log('REF t', inputs.time.toFixed(4), 'taken', tries.indexOf(taken), 'q', q.map((v) => v.join(',')).join(' '), 'qd', qd.join(' '), '\n', rows.map((row, r) => `  row ${r} J ${row.J.map((v) => v.toFixed(6)).join(',')} R ${row.compliance} high ${row.high} ref ${row.reference} f ${forces[r]}`).join('\n'));
    held.forEach((e, k) => { forces[e] = held0[k] - bounded.reduce((sum, c) => sum + heldPer.get(c)[k] * forces[c], 0); });
    const matrix = zero.map((__, row) => zero.map((___, column) => M0[row][column] + dampingMatrix(row, column) * timeStep));
    const qdd = solve(matrix, net.map((value, k) => value + rows.reduce((sum, row, r) => sum + row.J[k] * forces[r], 0)));
    return { qdd, warm: new Map(bounded.map((r) => [r, forces[r]])) };
}

// The joints' rows, in the model builder's order: a hinge's or a slider's dry friction (held to ±its friction) and its
// limit (pushing only, while past either end), each with its compliance R, its reference acceleration and its bounds.
function jointRows(bodies, q, qd, invweight, timeStep, n) {
    const rows = [];
    const unit = (k, sign = 1) => Array.from({ length: n }, (_, c) => (c === k ? sign : 0));
    bodies.forEach((body, i) => {
        if (!['hinge', 'slider'].includes(body.kind)) return;
        const { joint } = body;
        const k = body.firstSpeed;
        const v = qd[k];
        const soft = softnessOf(joint);
        if (joint.friction > 0) {
            const { B } = rowReference(soft.friction, timeStep);
            const imp = impedanceAt(soft.friction.solimp, 0);
            rows.push({ type: 'friction', J: unit(k), compliance: ((1 - imp) * invweight[k]) / imp, low: -joint.friction, high: joint.friction, reference: -B * v });
        }
        if (joint.limits) {
            const { B, K } = rowReference(soft.limit, timeStep);
            const lower = q[i][0] - joint.limits.lower - soft.margin;
            const upper = joint.limits.upper - q[i][0] - soft.margin;
            const past = lower < 0 ? lower : upper < 0 ? upper : 0;
            const sign = lower < 0 ? 1 : upper < 0 ? -1 : 1;
            const imp = impedanceAt(soft.limit.solimp, past);
            rows.push({
                type: 'limit', J: unit(k, sign), compliance: ((1 - imp) * invweight[k]) / imp, low: 0, high: lower < 0 || upper < 0 ? 1e30 : 0,
                reference: -B * sign * v - K * imp * past
            });
        }
    });
    return rows;
}

// The couplings' rows: q₁ - p(q₂) held to nothing, unbounded, after the joints' rows.
function couplingRows(couplings, q, qd, timeStep, n) {
    return couplings.map(({ first, second, polycoef: c, softness, w }) => {
        const x = second ? q[second.body][0] : 0;
        const r = q[first.body][0] - (c[0] + c[1] * x + c[2] * x ** 2 + c[3] * x ** 3 + c[4] * x ** 4);
        const J = Array(n).fill(0);
        J[first.speed] = 1;
        if (second) J[second.speed] = -(c[1] + 2 * c[2] * x + 3 * c[3] * x ** 2 + 4 * c[4] * x ** 3);
        const { B, K } = rowReference(softness, timeStep);
        const imp = impedanceAt(softness.solimp, r);
        return { type: 'equality', J, compliance: ((1 - imp) * w) / imp, low: -1e30, high: 1e30, reference: -B * dot3(J, qd) - K * imp * r };
    });
}

// A tendon now: its length and speed, the force it pulls with, and the damping taken at the step's end speed.
function tendonNow(tendon, q, qd) {
    const L = tendon.offset + dot3(tendon.c, tendon.positions(q));
    const speed = dot3(tendon.c, qd);
    const drive = tendon.drive ?? { mode: 'free' };
    const rest = Math.max(tendon.springLength, Math.min(tendon.springLengthUpper ?? tendon.springLength, L));
    let force = -tendon.stiffness * (L - rest) - tendon.damping * speed;
    let implicit = tendon.damping;
    if (drive.mode === 'torque') force += drive.torque ?? 0;
    if (drive.mode === 'servo') {
        const raw = drive.gain * ((tendon.setpoint ?? drive.setpoint) - L) - (drive.damping ?? 0) * speed;
        force += Math.max(-drive.torqueLimit, Math.min(drive.torqueLimit, raw));
        if (Math.abs(raw) <= drive.torqueLimit) implicit += drive.damping ?? 0;
    }
    return { L, speed, force, implicit };
}

// The tendons' rows, after the couplings': each tendon's dry friction, then its limit, as a joint's are, along c.
function tendonRows(tendons, q, qd, timeStep) {
    const rows = [];
    for (const tendon of tendons) {
        const { L, speed } = tendonNow(tendon, q, qd);
        const soft = softnessOf(tendon);
        if (tendon.friction > 0) {
            const { B } = rowReference(soft.friction, timeStep);
            const imp = impedanceAt(soft.friction.solimp, 0);
            rows.push({ type: 'friction', J: tendon.c, compliance: ((1 - imp) * tendon.w) / imp, low: -tendon.friction, high: tendon.friction, reference: -B * speed });
        }
        if (tendon.limits) {
            const { B, K } = rowReference(soft.limit, timeStep);
            const lower = L - tendon.limits.lower - soft.margin;
            const upper = tendon.limits.upper - L - soft.margin;
            const past = lower < 0 ? lower : upper < 0 ? upper : 0;
            const sign = lower < 0 ? 1 : upper < 0 ? -1 : 1;
            const imp = impedanceAt(soft.limit.solimp, past);
            rows.push({
                type: 'limit', J: tendon.c.map((value) => sign * value), compliance: ((1 - imp) * tendon.w) / imp, low: 0, high: lower < 0 || upper < 0 ? 1e30 : 0,
                reference: -B * sign * speed - K * imp * past
            });
        }
    }
    return rows;
}

// The contacts' rows, after the tendons': each pair's contact points as MuJoCo's collision functions find them
// (lib/collision.mjs, shared with the model builder and checked against MuJoCo's own contacts), each point four rows
// along its friction pyramid's edges n ± μ tᵢ, pushing only while it touches. J from each solid's motion per unit of
// each speed at the point; R = 2 μ² (1 - imp) w (1 + μ²) / imp, w the two solids' translational body_invweight0 (from
// this file's mass matrix); aref = -B J q̇ - K imp (dist - margin).
function contactRows(contacts, bodies, poses, qd, timeStep) {
    if (!contacts.length) return [];
    const n = qd.length;
    const unit = Array.from({ length: n }, (_, k) => motionOf(bodies, poses, qd.map((__, j) => (j === k ? 1 : 0)), qd.map(() => 0), 0));
    const pointVelocity = (body, k, point) => {
        if (body === -1) return [0, 0, 0];
        const { omega, velocity, centre } = unit[k][body];
        return add(velocity, cross(omega, subtract(point, centre)));
    };
    const rows = [];
    for (const contact of contacts) {
        const shape = ({ body, collider, ground }) => {
            const pose = body === -1 ? ground : poses[body];
            const rotation = multiply(pose.rotation, collider.rotation);
            const matrix = [0, 1, 2].map((row) => [0, 1, 2].map((column) => rotate(rotation, [0, 1, 2].map((axis) => (axis === column ? 1 : 0)))[row]));
            return { kind: collider.kind, size: collider.size, pos: add(pose.position, rotate(pose.rotation, collider.position)), mat: matrix.flat() };
        };
        const [a, b] = [contact.first, contact.second];
        const slots = collide(numbers, shape(a), shape(b), 0);
        const { B, K } = rowReference({ solref: contact.solref, solimp: contact.solimp }, timeStep);
        for (const slot of slots) {
            const touching = slot.active && slot.dist < contact.margin;
            const normal = slot.frame.slice(0, 3);
            const tangents = [slot.frame.slice(3, 6), slot.frame.slice(6, 9)];
            const past = slot.dist - contact.margin;
            const imp = impedanceAt(contact.solimp, past);
            const compliance = 2 * contact.mu * contact.mu * Math.max(1e-15, ((1 - imp) * contact.w * (1 + contact.mu * contact.mu)) / imp);
            for (const tangent of tangents) {
                for (const side of [1, -1]) {
                    const direction = add(normal, scale(tangent, side * contact.mu));
                    const J = Array.from({ length: n }, (_, k) => dot(direction, subtract(pointVelocity(b.body, k, slot.pos), pointVelocity(a.body, k, slot.pos))));
                    rows.push({ type: 'contact', J, compliance, low: 0, high: touching ? 1e30 : 0, reference: -B * dot3(J, qd) - K * imp * past });
                }
            }
        }
    }
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
// a solid joined to nothing has its joint as `free:<solid id>`. `sweeps`: the constraint rows' sweeps of projected
// Gauss-Seidel, the model builder's constraintSweeps unless given.
export function simulate(drawing, times, timeStep = 0.001, { sweeps, polishes } = {}) {
    const bodies = treeOf(drawing);
    const loops = loopsOf(drawing, bodies);
    let q = restCoordinates(bodies);
    let qd = Array(bodies.reduce((sum, body) => sum + speedCount[body.kind], 0)).fill(0);
    // Whether the joints have rows (a hinge or a slider with a dry friction or a limit), the constraint rows' forces the
    // step before, and each joint's inverse inertia where drawn.
    const couplingList = couplingsOf(drawing);
    const tendonList = tendonsOf(drawing);
    const contactList = contactPairs(drawing).pairs;
    const soft = bodies.some((body) => (body.kind === 'hinge' || body.kind === 'slider') && (body.joint.friction > 0 || body.joint.limits))
        || couplingList.length > 0 || tendonList.some((tendon) => tendon.friction > 0 || tendon.limits) || contactList.length > 0;
    let warm = new Map();
    const inverseMass = (() => {
        const zero = Array(qd.length).fill(0);
        const columns = zero.map((_, column) => inverseDynamics(bodies, q, zero, zero.map((__, k) => (k === column ? 1 : 0)), 0));
        const rotor = (joint) => (joint.drive?.mode === 'motor' ? joint.drive.motor.rotorInertia * joint.drive.motor.gearRatio ** 2 : 0);
        const armature = bodies.flatMap((body) => Array(speedCount[body.kind]).fill(body.kind === 'free' ? 0 : (body.joint.armature ?? 0) + rotor(body.joint)));
        const M0 = zero.map((__, row) => zero.map((___, column) => columns[column][row] + (row === column ? armature[row] : 0)));
        return zero.map((_, k) => solve(M0, zero.map((__, j) => (j === k ? 1 : 0))));
    })();
    const invweight = inverseMass.map((column, k) => column[k]);
    // Whether speeds i and j are of one joint, or of two joints one carrying the other.
    const bodyOfSpeed = bodies.flatMap((body, index) => Array(speedCount[body.kind]).fill(index));
    const carries = (above, below) => { for (let at = below; at !== -1; at = bodies[at].parent) if (at === above) return true; return false; };
    const related = (i, j) => carries(bodyOfSpeed[i], bodyOfSpeed[j]) || carries(bodyOfSpeed[j], bodyOfSpeed[i]);
    // Where each joint's coordinate and speed are, by joint id.
    const at = new Map(bodies.map((body, index) => [body.joint.id, { body: index, speed: body.firstSpeed }]));
    const couplings = couplingList.map((coupling) => {
        const first = at.get(coupling.first);
        const second = coupling.second ? at.get(coupling.second) : null;
        const softness = { solref: coupling.softness?.solref ?? [0.02, 1], solimp: coupling.softness?.solimp ?? [0.9, 0.95, 0.001, 0.5, 2] };
        return { first, second, polycoef: coupling.polycoef, softness, w: invweight[first.speed] + (second ? invweight[second.speed] : 0) };
    });
    const tendons = tendonList.map((tendon) => {
        const c = Array(qd.length).fill(0);
        for (const { joint, coef } of tendon.parts) c[at.get(joint).speed] = coef;
        const positions = (coordinates) => {
            const values = Array(qd.length).fill(0);
            for (const { joint } of tendon.parts) values[at.get(joint).speed] = coordinates[at.get(joint).body][0];
            return values;
        };
        return { ...tendon, c, positions, w: dot3(c, c.map((_, i) => dot3(inverseMass[i], c))) };
    });
    // Each body's translational inverse inertia where drawn (MuJoCo's body_invweight0): its centre's velocity per unit
    // of each speed, through M⁻¹, the mean of the diagonal.
    const restPoses = place(bodies, q);
    const restUnit = Array.from({ length: qd.length }, (_, k) => motionOf(bodies, restPoses, qd.map((__, j) => (j === k ? 1 : 0)), qd.map(() => 0), 0));
    const translationWeight = (body) => {
        if (body === -1) return 0;
        let sum = 0;
        for (let axis = 0; axis < 3; axis += 1) for (let i = 0; i < qd.length; i += 1) for (let j = 0; j < qd.length; j += 1) sum += restUnit[i][body].velocity[axis] * inverseMass[i][j] * restUnit[j][body].velocity[axis];
        return sum / 3;
    };
    const groundPose = poseOf(drawing, 'ground');
    const indexOf = new Map(bodies.map(({ id }, index) => [id, index]));
    const contacts = contactList.map(({ first, second }) => {
        const [a, b] = [first, second].map((entry) => (entry.solid === 'ground' ? floorSurfaceOf(drawing) : surfaceOf(drawing.solids.find(({ id }) => id === entry.solid))));
        const standard = a.solref[0] > 0 && b.solref[0] > 0;
        const end = (entry) => ({ body: entry.solid === 'ground' ? -1 : indexOf.get(entry.solid), collider: entry.collider, ground: groundPose });
        return {
            first: end(first), second: end(second),
            mu: Math.max(leastFriction, a.friction, b.friction), margin: a.margin + b.margin,
            solref: [0, 1].map((k) => (standard ? 0.5 * a.solref[k] + 0.5 * b.solref[k] : Math.min(a.solref[k], b.solref[k]))),
            solimp: a.solimp.map((value, k) => 0.5 * value + 0.5 * b.solimp[k]),
            w: translationWeight(end(first).body) + translationWeight(end(second).body)
        };
    });
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
            const found = forwardDynamics(bodies, q, qd, loops, timeStep, { setpoints, motorTorques, soft, warm, invweight, couplings, tendons, related, sweeps, polishes, contacts, time });
            const { qdd } = found;
            warm = found.warm;
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

