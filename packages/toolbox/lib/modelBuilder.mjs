/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A drawing as a Konjugate project document, in Konjugate's own terms: each solid is a node, each joint is a
// relationship between the two solids it joins, and the robot moves by the equations on those relationships alone.
// Nothing outside the graph solves it.
//
// The equations are Featherstone's articulated-body method, split along the joints. Every spatial quantity is in the
// world frame, about the world's origin, as six numbers: angular then linear (a velocity's linear part is that of the
// body point passing through the origin). With everything about one point, a joint relationship needs no transforms
// between its two solids: it reads its parent solid's and its child solid's states and sets, at each instant:
// - outward, on the child: its pose (position and rotation), its velocity, its own spatial inertia and bias force;
// - inward, on the parent: the articulated inertia and bias force the child passes up (added to what the parent's
//   other children and the parent's own body give);
// - outward again, on the child: its joint's acceleration and the child's spatial acceleration.
// Konjugate evaluates relationships that set values in dependency order within each instant (core's algebraic
// relationships), so the three sweeps settle every step. Each solid's node then integrates its joint's speed and angle
// semi-implicitly (speed first, then the angle from the new speed), which keeps a swinging arm's energy bounded.
// Gravity enters as the Ground's acceleration, upward (the usual trick: the solids' accelerations are then relative
// to free fall).
//
// The Ground is a node too, holding still. Each solid's mass is a shared parameter; a joint's settings are its
// relationship's parameters, the drive's torque or setpoint a live slider.
//
// Where a hinge or a slider has a dry friction or a limit, the joints' rows join the loops' and are found as MuJoCo
// finds them: a first sweep from the solids' inertia alone gives each joint's acceleration q̈₀ and how it moves per
// unit of each row's force (the loops' sensitivities, a column per row); each joint's relationship hands its own rows of
// K = J M⁻¹ Jᵀ and its wanted acceleration inward, and the relationship of each solid hanging from the Ground solves
// them from what the Ground gathered: the loops' rows taken out exactly, the joints' by projected Gauss-Seidel from the
// step before's forces (kept on that solid) and then exactly for the rows it leaves inside their bounds. The forces
// are handed out again, and a second sweep takes the damping at the step's end speed with them on: the acceleration
// the step takes. All of it equations on the relationships, nothing solved outside the graph.
//
// A coupling (MuJoCo's joint equality) and a fixed tendon's dry friction and limit are rows the same way: a coupling a
// relationship between its two joints' solids, a tendon a node of its own with a relationship from each of its joints'
// solids. With a tendon's damping, every damping is a row too, found at the Ground after the others (buildModel's
// `dampingRows`), in place of the second sweep.

import { batteryOf, checkDrawing, couplingsOf, defaultDrive, drivenJoints, frameById, frameInWorld, groundId, keyframesOf, massOf, poseOf, structure, tendonsOf, treeOrder } from './drawing.mjs';
import * as E from './expressions.mjs';
import { conjugate, directionToLocal, multiply as multiplyQuaternions, normalize, rotate, scale, toLocal, toRollPitchYaw } from './math.mjs';
import { shapeMesh } from './shapes.mjs';

export const gravity = 9.81;

// How a joint that closes a loop holds its two points together: their gap and its rate are pulled back to nothing as a
// critically damped spring would pull them (Baumgarte's terms), at a rate of a quarter of the time step's (250 per
// second at a millisecond), as fast as stepping allows without overshoot; and the force between them gives a little,
// by `compliance` (metres per second squared per newton), so a direction the loop's other joints already hold (out of a
// planar linkage's plane) asks for no force, rather than any.
export const closure = Object.freeze({ ratePerStep: 0.25, compliance: 1e-9 });
export const closureRate = (timeStep) => closure.ratePerStep / timeStep;

// A name as a lower camel case key ("left elbow" and "left_elbow" are "leftElbow"), unique among `taken`.
export function camelKey(name, taken = new Set(), fallback = 'part') {
    const words = String(name).split(/[^A-Za-z0-9]+/).filter(Boolean);
    let key = words.map((word, index) => (index ? word[0].toUpperCase() + word.slice(1) : word[0].toLowerCase() + word.slice(1))).join('') || fallback;
    if (!/^[a-z]/.test(key)) key = `${fallback}${key[0].toUpperCase()}${key.slice(1)}`;
    let unique = key;
    for (let suffix = 2; taken.has(unique); suffix += 1) unique = `${key}${suffix}`;
    taken.add(unique);
    return unique;
}

// A solid's inertia per kilogram about its centre, along its own axes: an imported solid's own (its frame its
// principal one), or its shape's.
export const inertiaOf = (solid) => (solid.shape === 'mesh' ? solid.unitInertia : unitInertia(solid.shape, solid.size));

// A primitive's inertia per kilogram about its centre, along its own axes (a cylinder and a capsule along z).
export function unitInertia(shape, size) {
    switch (shape) {
    case 'box': return [(size.y ** 2 + size.z ** 2) / 12, (size.x ** 2 + size.z ** 2) / 12, (size.x ** 2 + size.y ** 2) / 12];
    case 'cylinder': {
        const across = (3 * size.radius ** 2 + size.length ** 2) / 12;
        return [across, across, size.radius ** 2 / 2];
    }
    case 'sphere': return Array(3).fill((2 / 5) * size.radius ** 2);
    case 'capsule': {
        // Its middle a cylinder, its ends two half spheres, each share of the mass as its share of the volume.
        const { radius: r, length: l } = size;
        const middle = Math.PI * r * r * l;
        const ends = (4 / 3) * Math.PI * r ** 3;
        const total = middle + ends;
        const across = (middle * (l * l / 12 + r * r / 4) + ends * ((2 / 5) * r * r + l * l / 4 + (3 / 8) * l * r)) / total;
        return [across, across, (middle * r * r / 2 + ends * (2 / 5) * r * r) / total];
    }
    default: throw new Error(`There is no shape "${shape}".`);
    }
}

// A quaternion as a rotation matrix (rows).
function rotationMatrix(q) {
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((_, row) => [0, 1, 2].map((column) => rotate(q, [0, 1, 2].map((axis) => (axis === column ? 1 : 0)))[row]));
}

const round = (value) => Math.round(value * 1e12) / 1e12;

// A live shared parameter's slider, from `minimum` to `maximum` in 400 steps.
const liveRange = (minimum, maximum) => ({ minimum: round(minimum), maximum: round(maximum), step: round((maximum - minimum) / 400) });

// What a fork can change, as Konjugate's launcher host finds it (the importer's parameter index): a live shared
// parameter, by what it is (`key`) and whose it is (`entity`, a solid's, a motor's joint's or the battery's name).
const indexEntry = (key, entity, shared) => ({
    key, entity, name: shared.name, symbol: shared.symbol, sharedParameterId: shared.id, scope: 'instance', value: shared.value, unit: shared.unit,
    live: shared.mode === 'live', minimum: shared.control?.minimum, maximum: shared.control?.maximum
});

// The states a solid's node holds, by symbol: its joint's speed and angle first (integrated in that order), then what
// the relationships set. Index pairs of the 6 by 6 symmetric articulated inertia are its upper triangle.
const symmetric = [];
for (let row = 1; row <= 6; row += 1) for (let column = row; column <= 6; column += 1) symmetric.push([row, column]);
const inertiaSymbol = (row, column) => `ia${Math.min(row, column)}${Math.max(row, column)}`;
const rotationSymbols = [['r11', 'r12', 'r13'], ['r21', 'r22', 'r23'], ['r31', 'r32', 'r33']];
const velocitySymbols = ['w1', 'w2', 'w3', 'v1', 'v2', 'v3'];
const accelerationSymbols = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6'];
const biasSymbols = ['pa1', 'pa2', 'pa3', 'pa4', 'pa5', 'pa6'];

const stateNames = {
    jointSpeed: 'Joint speed', jointAngle: 'Joint angle', jointPosition: 'Joint position', jointAcceleration: 'Joint acceleration',
    jointTorque: 'Drive torque', motorTorque: 'Motor torque at the joint', supplyVoltage: 'Supply voltage', duty: 'Duty cycle', current: 'Current',
    torque: 'Torque at the joint', windingTemperature: 'Winding temperature', voltage: 'Voltage', stateOfCharge: 'State of charge', x: 'x', y: 'y', z: 'z', roll: 'Roll', pitch: 'Pitch', yaw: 'Yaw',
    w1: 'Angular velocity x', w2: 'Angular velocity y', w3: 'Angular velocity z',
    v1: 'Velocity at origin x', v2: 'Velocity at origin y', v3: 'Velocity at origin z',
    vc1: 'Centre velocity x', vc2: 'Centre velocity y', vc3: 'Centre velocity z',
    jointSpeed1: 'Joint angular velocity x', jointSpeed2: 'Joint angular velocity y', jointSpeed3: 'Joint angular velocity z',
    jointTurnW: 'Joint turn w', jointTurnX: 'Joint turn x', jointTurnY: 'Joint turn y', jointTurnZ: 'Joint turn z',
    turnW: 'Turn w', turnX: 'Turn x', turnY: 'Turn y', turnZ: 'Turn z',
    kineticEnergy: 'Kinetic energy', potentialEnergy: 'Potential energy',
    tendonForce: 'Tendons\' force on the joint', length: 'Length', speed: 'Speed', force: 'Force', limitSide: 'Limit side'
};
const stateName = (symbol) => stateNames[symbol]
    ?? (/^jointAcceleration\d$/.test(symbol) ? `Joint acceleration ${symbol.slice(-1)}`
        : symbol.startsWith('ia') ? `Articulated inertia ${symbol.slice(2)}` : symbol.startsWith('pa') ? `Articulated bias force ${symbol.slice(2)}`
            : symbol.startsWith('lp') ? `Loop bias sensitivity ${symbol.slice(2)}` : symbol.startsWith('la') ? `Loop acceleration sensitivity ${symbol.slice(2)}`
                : symbol.startsWith('dp') ? `Loop bias change ${symbol.slice(2)}` : symbol.startsWith('da') ? `Loop acceleration change ${symbol.slice(2)}`
                    : symbol.startsWith('kr') ? `Loop coupling ${symbol.slice(2)}` : symbol.startsWith('kb') ? `Loop wanted acceleration ${symbol.slice(2)}`
                        : symbol.startsWith('lam') ? `Loop force ${symbol.slice(3)}` : /^rj\d+$/.test(symbol) ? `Row ${symbol.slice(2)} on the joint`
                    : symbol.startsWith('a') ? `Spatial acceleration ${symbol.slice(1)}` : symbol.startsWith('r') ? `Rotation ${symbol.slice(1)}` : symbol);

// The loops' sensitivities: a 6 by R matrix each (how a solid's bias force, or its acceleration, moves with the R
// components of the forces that close the loops), the changes those forces make, and what is gathered to find them: the
// R by R coupling K (how each loop's held motion moves with each component), the accelerations wanted, and the forces.
const sensitivitySymbols = (prefix, count) => [1, 2, 3, 4, 5, 6].map((row) => Array.from({ length: count }, (_, column) => `${prefix}${row}c${column + 1}`));
const couplingSymbols = (count) => Array.from({ length: count }, (_, row) => Array.from({ length: count }, (__, column) => `kr${row + 1}c${column + 1}`));
// K is symmetric: only its lower triangle (column at or before row) is gathered.
const lowerCoupling = (count) => couplingSymbols(count).flatMap((row, r) => row.slice(0, r + 1));
const wantedSymbols = (count) => Array.from({ length: count }, (_, row) => `kb${row + 1}`);
const forceSymbols = (count) => Array.from({ length: count }, (_, row) => `lam${row + 1}`);
// A joint row's settings, gathered into the Ground with its rows of K: its compliance R (1/D) and its force's bounds; a
// damping row's, h b alone (its damping times the step).
const rowSettingSymbols = (types) => types.flatMap((type, index) => (type === 'loop' ? [] : type === 'damping' ? [`kc${index + 1}`] : [`kc${index + 1}`, `kn${index + 1}`, `kx${index + 1}`]));
// The rows projected Gauss-Seidel finds: a friction's, a limit's and a coupling's (an equality, unbounded).
const searched = (type) => type === 'friction' || type === 'limit' || type === 'equality';
// Sweeps of projected Gauss-Seidel over the constraint rows at each instant, starting from the step before's forces;
// then the rows it leaves strictly inside their bounds are solved exactly with the others held at theirs, and that is
// taken where it is consistent (each solved force inside its bounds, each held one pushing against its bound), the
// sweeps' forces where not. Measured on the robots of the tests, two sweeps already find the rows MuJoCo's own solve
// holds, every step.
export const constraintSweeps = 5;
// What a joint closing a loop holds, by kind: its point (three linear rows) or the turn about its axes (angular rows),
// along the first solid's frame axes t1, t2 and n (n the joint's axis). A ball holds its point; a hinge its point and its
// axis; a slider its axis's line and the turn; a fixed joint everything.
const closureRows = {
    ball: [['linear', 0], ['linear', 1], ['linear', 2]],
    hinge: [['linear', 0], ['linear', 1], ['linear', 2], ['angular', 0], ['angular', 1]],
    slider: [['linear', 0], ['linear', 1], ['angular', 0], ['angular', 1], ['angular', 2]],
    fixed: [['linear', 0], ['linear', 1], ['linear', 2], ['angular', 0], ['angular', 1], ['angular', 2]]
};

// Two unit directions square to `n` and to each other, so (t1, t2, n) is a right-handed frame.
export function closureFrame(n) {
    const smallest = [0, 1, 2].reduce((best, axis) => (Math.abs(n[axis]) < Math.abs(n[best]) ? axis : best), 0);
    const t1 = normalize(cross3(n, [0, 1, 2].map((axis) => (axis === smallest ? 1 : 0))));
    return [t1, cross3(n, t1), n];
}
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const biasChangeSymbols = ['dp1', 'dp2', 'dp3', 'dp4', 'dp5', 'dp6'];
// With joint rows (buildModel's `soft`), the second sweep's: each solid's articulated inertia and bias force with the
// damping taken at the step's end speed and the rows' forces on, its acceleration, and the loops' forces on it.
const dampedInertiaSymbol = (row, column) => `ib${Math.min(row, column)}${Math.max(row, column)}`;
const dampedBiasSymbols = ['pq1', 'pq2', 'pq3', 'pq4', 'pq5', 'pq6'];
const dampedAccelerationSymbols = ['aq1', 'aq2', 'aq3', 'aq4', 'aq5', 'aq6'];
const externalSymbols = ['xf1', 'xf2', 'xf3', 'xf4', 'xf5', 'xf6'];
const accelerationChangeSymbols = ['da1', 'da2', 'da3', 'da4', 'da5', 'da6'];
const turnSymbols = { ball: ['jointTurnW', 'jointTurnX', 'jointTurnY', 'jointTurnZ'], free: ['turnW', 'turnX', 'turnY', 'turnZ'] };
const degreesOfFreedom = { hinge: 1, slider: 1, ball: 3, free: 6, fixed: 0 };
const centreVelocitySymbols = ['vc1', 'vc2', 'vc3'];

// The reasons a drawing cannot be simulated yet by this builder (beyond checkDrawing's), each { id, message }: none, now
// that every kind of joint can close a loop and a robot may have any number of them; kept for what comes next.
// ---- joint limits and dry friction, held as MuJoCo holds them --------------------------------------------------------------
// MuJoCo holds a hinge's or a slider's limit, once it is past it, and its dry friction always, by soft constraints:
// each a row of generalized force f on the joint, a dry friction's held to ±its friction, a limit's to pushing only,
// found with every other row from (A + R) f = aref - J q̈₀ within those bounds, A = J M⁻¹ Jᵀ (buildModel gathers it
// into the Ground). R = (1 - imp) w / imp, w the joint's inverse inertia where the robot is drawn (inverseWeights) and
// imp the row's impedance; aref = -B v - K imp r, v the row's speed and r how far past the limit it is (0 for
// friction). B and K from the row's solref, imp from its solimp, as MuJoCo works them out.
export const defaultSoftness = Object.freeze({ solref: Object.freeze([0.02, 1]), solimp: Object.freeze([0.9, 0.95, 0.001, 0.5, 2]) });
const impedanceBound = (value) => Math.min(Math.max(value, 0.0001), 0.9999);
// A joint's softness: its limit's and its friction's { solref, solimp }, and its limit's margin (lib/drawing.mjs's
// joint.softness, from an imported model; MuJoCo's defaults otherwise).
export function softnessOf(joint) {
    const of = (which) => ({ solref: joint.softness?.[which]?.solref ?? defaultSoftness.solref, solimp: joint.softness?.[which]?.solimp ?? defaultSoftness.solimp });
    return { limit: of('limit'), friction: of('friction'), margin: joint.softness?.margin ?? 0 };
}
// B and K for a row (MuJoCo's reference): a time constant (at least two steps) and damping ratio, or, given as
// negatives, a stiffness and damping.
export function rowReference({ solref, solimp }, timeStep) {
    const dmax = impedanceBound(solimp[1]);
    if (solref[0] > 0) {
        const timeconst = Math.max(solref[0], 2 * timeStep);
        return { B: 2 / (dmax * timeconst), K: 1 / (dmax * dmax * timeconst * timeconst * solref[1] * solref[1]) };
    }
    return { B: -solref[1] / dmax, K: -solref[0] / (dmax * dmax) };
}
// The impedance at r, how far past (as a number, for a row's fixed impedance) or as an expression.
export function impedanceAt([low, high, width, mid, power], r) {
    const dmin = impedanceBound(low);
    const dmax = impedanceBound(high);
    const steps = Math.round(power);
    const raise = (value) => E.multiply(...Array(Math.max(steps, 1)).fill(value));
    if (typeof r === 'number') {
        const x = Math.abs(r) / width;
        if (x >= 1 || !(width > 1e-10)) return dmax;
        const y = x <= mid ? x ** steps / mid ** (steps - 1) : 1 - (1 - x) ** steps / (1 - mid) ** (steps - 1);
        return dmin + y * (dmax - dmin);
    }
    if (!(width > 1e-10)) return dmax;
    const x = E.divide(E.max(r, E.negate(r)), width);
    return E.cases([
        [E.greater(x, 1), dmax],
        [E.greater(x, mid), E.add(dmin, E.multiply(dmax - dmin, E.subtract(1, E.divide(raise(E.subtract(1, x)), round((1 - mid) ** (steps - 1))))))]
    ], E.add(dmin, E.multiply(dmax - dmin, E.divide(raise(x), round(mid ** (steps - 1))))));
}

// Each hinge's and slider's inverse inertia where the robot is drawn, (M⁻¹)ᵢᵢ, as MuJoCo's dof_invweight0: M the
// tree's joint-space mass matrix there (each solid through every joint that carries it, each joint's armature and
// motor rotor on its diagonal; loops left out, as MuJoCo leaves its equality constraints out). By joint id; and each
// tendon's, cᵀ M⁻¹ c, by its id.
export function inverseWeights(drawing, built = structure(drawing)) {
    const order = treeOrder(drawing, built);
    const children = new Map();
    for (const [child, { parent }] of built.parentJoint) children.set(parent, [...(children.get(parent) ?? []), child]);
    const carried = (solidId) => { const all = [solidId]; for (let at = 0; at < all.length; at += 1) all.push(...(children.get(all[at]) ?? [])); return all; };
    const dofs = [];
    for (const { id: solidId } of order) {
        const pose = poseOf(drawing, solidId);
        const link = built.parentJoint.get(solidId);
        const units = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
        const at = carried(solidId);
        if (!link) {
            units.forEach((e) => dofs.push({ at, w: [0, 0, 0], v: e, armature: 0 }));
            units.forEach((e) => dofs.push({ at, w: e, v: cross3(pose.position, e), armature: 0 }));
            continue;
        }
        const { joint } = link;
        if (joint.kind === 'fixed') continue;
        const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
        const rotor = joint.drive?.mode === 'motor' ? (joint.drive.motor?.rotorInertia ?? 0) * (joint.drive.motor?.gearRatio ?? 0) ** 2 : 0;
        const armature = (joint.armature ?? 0) + rotor;
        const axis = normalize(anchor.axis);
        if (joint.kind === 'hinge') dofs.push({ at, w: axis, v: cross3(anchor.point, axis), armature, joint: joint.id });
        else if (joint.kind === 'slider') dofs.push({ at, w: [0, 0, 0], v: axis, armature, joint: joint.id });
        else if (joint.kind === 'ball') units.forEach((e) => dofs.push({ at, w: e, v: cross3(anchor.point, e), armature }));
    }
    const inertias = new Map(drawing.solids.map((solid) => {
        const pose = poseOf(drawing, solid.id);
        const R = rotationMatrix(pose.rotation);
        const moments = inertiaOf(solid).map((value) => value * massOf(solid));
        // R diag(moments) Rᵀ w
        const apply = (w) => { const local = [0, 1, 2].map((k) => R[0][k] * w[0] + R[1][k] * w[1] + R[2][k] * w[2]); return [0, 1, 2].map((i) => R[i][0] * moments[0] * local[0] + R[i][1] * moments[1] * local[1] + R[i][2] * moments[2] * local[2]); };
        return [solid.id, { mass: massOf(solid), centre: pose.position, apply }];
    }));
    const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const n = dofs.length;
    const M = Array.from({ length: n }, () => Array(n).fill(0));
    for (let a = 0; a < n; a += 1) {
        for (let b = a; b < n; b += 1) {
            const shared = new Set(dofs[b].at);
            let sum = 0;
            for (const solidId of dofs[a].at) {
                if (!shared.has(solidId)) continue;
                const { mass, centre, apply } = inertias.get(solidId);
                const ua = dofs[a].v.map((value, k) => value + cross3(dofs[a].w, centre)[k]);
                const ub = dofs[b].v.map((value, k) => value + cross3(dofs[b].w, centre)[k]);
                sum += dot3(dofs[a].w, apply(dofs[b].w)) + mass * dot3(ua, ub);
            }
            M[a][b] = sum + (a === b ? dofs[a].armature : 0);
            M[b][a] = M[a][b];
        }
    }
    // M⁻¹'s diagonal, by Gauss-Jordan elimination with partial pivoting.
    const work = M.map((row, r) => [...row, ...Array.from({ length: n }, (_, c) => (c === r ? 1 : 0))]);
    for (let column = 0; column < n; column += 1) {
        let pivot = column;
        for (let r = column + 1; r < n; r += 1) if (Math.abs(work[r][column]) > Math.abs(work[pivot][column])) pivot = r;
        [work[column], work[pivot]] = [work[pivot], work[column]];
        const lead = work[column][column];
        for (let c = 0; c < 2 * n; c += 1) work[column][c] /= lead;
        for (let r = 0; r < n; r += 1) {
            if (r === column || !work[r][column]) continue;
            const factor = work[r][column];
            for (let c = 0; c < 2 * n; c += 1) work[r][c] -= factor * work[column][c];
        }
    }
    const weights = new Map(dofs.map((dof, k) => [dof.joint, work[k][n + k]]).filter(([jointId]) => jointId));
    // A tendon's, as MuJoCo's tendon_invweight0: cᵀ M⁻¹ c, by the tendon's id.
    const dofOf = new Map(dofs.map((dof, k) => [dof.joint, k]).filter(([jointId]) => jointId));
    for (const tendon of tendonsOf(drawing)) {
        if (!tendon.parts.every(({ joint }) => dofOf.has(joint))) continue;
        weights.set(tendon.id, tendon.parts.reduce((sum, a) => sum + tendon.parts.reduce((inner, b) => inner + a.coef * b.coef * work[dofOf.get(a.joint)][n + dofOf.get(b.joint)], 0), 0));
    }
    return weights;
}

export function simulationLimits() {
    return [];
}

// A unit quaternion's rotation matrix (rows), from a quaternion that may have drifted from unit length: each entry over
// its squared length, so the matrix stays a rotation.
function quaternionMatrix(define, prefix, [w, x, y, z]) {
    const size = define(`${prefix}n`, E.add(E.multiply(w, w), E.multiply(x, x), E.multiply(y, y), E.multiply(z, z)));
    const entry = (expression) => E.divide(expression, size);
    const twice = (a, b) => E.multiply(2, a, b);
    return [
        [entry(E.add(E.multiply(w, w), E.multiply(x, x), E.negate(E.multiply(y, y)), E.negate(E.multiply(z, z)))), entry(E.subtract(twice(x, y), twice(w, z))), entry(E.add(twice(x, z), twice(w, y)))],
        [entry(E.add(twice(x, y), twice(w, z))), entry(E.add(E.multiply(w, w), E.negate(E.multiply(x, x)), E.multiply(y, y), E.negate(E.multiply(z, z)))), entry(E.subtract(twice(y, z), twice(w, x)))],
        [entry(E.subtract(twice(x, z), twice(w, y))), entry(E.add(twice(y, z), twice(w, x))), entry(E.add(E.multiply(w, w), E.negate(E.multiply(x, x)), E.negate(E.multiply(y, y)), E.multiply(z, z)))]
    ].map((row, rowIndex) => row.map((value, column) => define(`${prefix}${rowIndex + 1}${column + 1}`, value)));
}

// q̇ = ½ [0, ω] ⊗ q, for a quaternion's four components in turn.
function quaternionRates([w, x, y, z], [w1, w2, w3]) {
    return [
        E.multiply(-0.5, E.add(E.multiply(w1, x), E.multiply(w2, y), E.multiply(w3, z))),
        E.multiply(0.5, E.add(E.multiply(w1, w), E.multiply(w2, z), E.negate(E.multiply(w3, y)))),
        E.multiply(0.5, E.add(E.multiply(w2, w), E.multiply(w3, x), E.negate(E.multiply(w1, z)))),
        E.multiply(0.5, E.add(E.multiply(w3, w), E.multiply(w1, y), E.negate(E.multiply(w2, x))))
    ];
}

// A small symmetric matrix's eigenvalues and unit eigenvectors, [{ value, vector }], by Jacobi's rotations.
export function symmetricEigen(matrix) {
    const n = matrix.length;
    const a = matrix.map((row) => [...row]);
    const v = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (__, j) => (i === j ? 1 : 0)));
    for (let sweep = 0; sweep < 100; sweep += 1) {
        let off = 0;
        for (let p = 0; p < n; p += 1) for (let q = p + 1; q < n; q += 1) off += a[p][q] * a[p][q];
        if (off < 1e-30) break;
        for (let p = 0; p < n; p += 1) {
            for (let q = p + 1; q < n; q += 1) {
                if (Math.abs(a[p][q]) < 1e-300) continue;
                const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
                const c = 1 / Math.sqrt(t * t + 1);
                const s = t * c;
                for (let k = 0; k < n; k += 1) {
                    const [akp, akq] = [a[k][p], a[k][q]];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (let k = 0; k < n; k += 1) {
                    const [apk, aqk] = [a[p][k], a[q][k]];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (let k = 0; k < n; k += 1) {
                    const [vkp, vkq] = [v[k][p], v[k][q]];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    return Array.from({ length: n }, (_, k) => ({ value: a[k][k], vector: v.map((row) => row[k]) }));
}

// Solving D x = b for a joint's symmetric, positive definite D (1, 3 or 6 square): by division for one axis, otherwise
// by D's L D Lᵀ factors, defined once, and each solve's own forward and back substitution. Returns solve(b, name).
function symmetricSolver(define, prefix, D) {
    const n = D.length;
    if (n === 1) return (b) => [E.divide(b[0], D[0][0])];
    const L = Array.from({ length: n }, () => Array(n).fill(0));
    const pivots = [];
    for (let j = 0; j < n; j += 1) {
        pivots[j] = define(`${prefix}d${j + 1}`, E.subtract(D[j][j], E.add(Array.from({ length: j }, (_, k) => E.multiply(L[j][k], L[j][k], pivots[k])))));
        for (let i = j + 1; i < n; i += 1) {
            L[i][j] = define(`${prefix}l${i + 1}${j + 1}`, E.divide(E.subtract(D[i][j], E.add(Array.from({ length: j }, (_, k) => E.multiply(L[i][k], L[j][k], pivots[k])))), pivots[j]));
        }
    }
    return (b, name) => {
        const forward = [];
        for (let i = 0; i < n; i += 1) forward[i] = define(`${name}f${i + 1}`, E.subtract(b[i], E.add(Array.from({ length: i }, (_, k) => E.multiply(L[i][k], forward[k])))));
        const x = [];
        for (let i = n - 1; i >= 0; i -= 1) {
            x[i] = define(`${name}x${i + 1}`, E.subtract(E.divide(forward[i], pivots[i]), E.add(Array.from({ length: n - 1 - i }, (_, offset) => E.multiply(L[i + 1 + offset][i], x[i + 1 + offset])))));
        }
        return x;
    };
}

// Builds the document. Returns { document, data, report } or throws with the drawing's first problem.
// `looks`: an imported solid's look by its key (`solid.look.key`; { mesh, color }, in the solid's frame), for its node in the canvas; a
// solid with none is shown as the box about it.
// `inverseWeights`: each joint's inverse inertia where drawn (inverseWeights), as a model built before had it: a
// constant of the model, as MuJoCo's dof_invweight0 is of its compiled model, which a change made later (a heavier
// solid from a time on) leaves as it was. Worked out from this drawing otherwise.
export function buildModel(drawing, { timeStep = 0.001, runTime = 10, outputInterval = 0.01, looks = {}, inverseWeights: weightsGiven = null } = {}) {
    const check = checkDrawing(drawing);
    const problems = [...check.errors, ...simulationLimits(drawing)];
    if (problems.length) throw new Error(problems[0].message);
    let nextId = 1;
    const id = () => nextId++;
    const document = {
        format: 'konjugate', version: 1, copyright: 'Copyright © 2026 Zenin Easa Panthakkalakath', metadata: { units: 'SI' },
        nodes: [], edges: [], sharedParameters: []
    };
    const built = structure(drawing);
    const order = treeOrder(drawing, built);
    // Each solid's joint to its parent; a solid joined to nothing hangs from the Ground by a free joint.
    const parentOf = new Map(order.map(({ id: solidId }) => {
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        return [solidId, built.parentJoint.get(solidId) ?? { joint: { id: `free:${solidId}`, kind: 'free', name: `${solid.name} floating` }, parent: groundId, child: solidId }];
    }));

    // ---- the loops the drawing closes -------------------------------------------------------------------------------
    // Each closing joint holds some rows of motion (closureRows), R in all. The solids from each loop's two ends back
    // towards the Ground (`path`) carry the sensitivities; every solid below one of them (`affected`) moves with the
    // forces; and from each loop's holder (its second end, or its first when the second is the Ground) back to the
    // Ground (`gather`), what finds the forces is gathered into the Ground and the forces handed back.
    const loopKeys = new Set();
    let offset = 0;
    const loops = built.loops.map((joint) => {
        const ends = [joint.parent, joint.child].map((frameId) => frameById(drawing, frameId));
        const rows = closureRows[joint.kind];
        const loop = { joint, ends, rows, offset, key: camelKey(joint.name, loopKeys, 'loop'), holderId: ends[1].solid === groundId ? ends[0].solid : ends[1].solid };
        offset += rows.length;
        return loop;
    });
    // ---- the joints' rows, where a hinge or a slider has a limit or a dry friction -----------------------------------
    // Then every joint's soft rows (jointRows), as MuJoCo holds them, join the loops' rows, gathered into the Ground
    // and found there with them from the solids' inertia alone; and a second sweep of the articulated-body method
    // takes the damping at the step's end speed with the rows' and loops' forces on, as MuJoCo takes them.
    //
    // A coupling (q₁ = p(q₂), MuJoCo's joint equality) and a tendon's dry friction and limit are rows the same way, each
    // acting on several joints (J = [1, -p'(q₂)] for a coupling, ±the tendon's coefficients for a tendon): each joint
    // hands in its share of the row's K and of J q̈₀, and the coupling's or the tendon's own relationship the row's
    // reference acceleration, compliance and bounds. A tendon's damping, or its servo's, couples its joints' speeds, so
    // the damping taken at the step's end speed is no longer each joint's alone: then every damping is a row too
    // (`dampingRows`), unbounded, with compliance 1 / (h b) and found at the Ground after the other rows, from them, as
    // (M + h Σ b c cᵀ) q̈ = τ + Jᵀ f gives it; and each joint's acceleration is q̈₀ plus each row's force times how the
    // joint moves with it, with no second sweep.
    const tree = order.map(({ id: solidId }) => parentOf.get(solidId).joint);
    const couplings = couplingsOf(drawing);
    const tendons = tendonsOf(drawing);
    const tendonDamped = (tendon) => tendon.damping > 0 || (tendon.drive?.mode === 'servo' && tendon.drive.damping > 0);
    const dampingRows = tendons.some(tendonDamped);
    const soft = dampingRows || couplings.length > 0 || tendons.some((tendon) => tendon.friction > 0 || tendon.limits)
        || tree.some((joint) => (joint.kind === 'hinge' || joint.kind === 'slider') && (joint.friction > 0 || joint.limits) && !built.loops.includes(joint));
    const solidOfJoint = new Map(order.map(({ id: solidId }) => [parentOf.get(solidId).joint.id, solidId]));
    // Each solid's joint's rows: { type, dof, index, J }, J a number, a function of the joint's coordinate, or 'state'
    // (set on the solid by the row's own relationship); `external` when the coupling's or the tendon's relationship gives
    // the row's reference, compliance and bounds.
    const jointRows = new Map(order.map(({ id: solidId }) => [solidId, []]));
    const rowsOn = (jointId) => jointRows.get(solidOfJoint.get(jointId));
    const rowTypes = Array(offset).fill('loop');
    const newRow = (type) => { rowTypes.push(type); offset += 1; return offset - 1; };
    if (soft) {
        for (const { id: solidId } of order) {
            const { joint } = parentOf.get(solidId);
            if (joint.kind !== 'hinge' && joint.kind !== 'slider') continue;
            if (joint.friction > 0) jointRows.get(solidId).push({ type: 'friction', dof: 0, index: newRow('friction'), J: 1 });
            if (joint.limits) jointRows.get(solidId).push({ type: 'limit', dof: 0, index: newRow('limit'), J: 'limit' });
        }
    }
    const couplingRowOf = new Map();
    for (const coupling of couplings) {
        const index = newRow('equality');
        couplingRowOf.set(coupling.id, index);
        rowsOn(coupling.first).push({ type: 'equality', dof: 0, index, J: 1, external: true });
        if (coupling.second) {
            const c = coupling.polycoef;
            rowsOn(coupling.second).push({ type: 'equality', dof: 0, index, external: true,
                J: (q) => E.negate(E.add(c[1], E.multiply(2 * c[2], q), E.multiply(3 * c[3], q, q), E.multiply(4 * c[4], q, q, q))) });
        }
    }
    const tendonRowsOf = new Map();
    for (const tendon of tendons) {
        const own = {};
        if (tendon.friction > 0) {
            own.friction = newRow('friction');
            for (const { joint, coef } of tendon.parts) rowsOn(joint).push({ type: 'friction', dof: 0, index: own.friction, J: coef, external: true });
        }
        if (tendon.limits) {
            own.limit = newRow('limit');
            for (const { joint } of tendon.parts) rowsOn(joint).push({ type: 'limit', dof: 0, index: own.limit, J: 'state', external: true });
        }
        tendonRowsOf.set(tendon.id, own);
    }
    if (dampingRows) {
        for (const { id: solidId } of order) {
            const { joint } = parentOf.get(solidId);
            const servoDamping = (joint.kind === 'hinge' || joint.kind === 'slider') && joint.drive?.mode === 'servo' && joint.drive.damping > 0;
            if (!['hinge', 'slider', 'ball'].includes(joint.kind) || !(joint.damping > 0 || servoDamping)) continue;
            for (let dof = 0; dof < degreesOfFreedom[joint.kind]; dof += 1) jointRows.get(solidId).push({ type: 'damping', dof, index: newRow('damping'), J: 1 });
        }
        // A tendon's damping b c cᵀ, as MuJoCo's implicitfast takes it: only where the mass matrix has an entry (two of
        // its joints one carrying the other), so two joints side by side, a gripper's fingers, are each damped alone.
        // That matrix, G = c cᵀ kept there, is a sum of rows as its eigenvectors give it (numbers of the drawing, as its
        // coefficients are; G may have a negative eigenvalue where it is kept only in part): row k along vₖ, its weight
        // λₖ times h b.
        const carries = (above, below) => { for (let at = below; at !== groundId; at = parentOf.get(at).parent) if (at === above) return true; return false; };
        for (const tendon of tendons.filter(tendonDamped)) {
            const solidsOf = tendon.parts.map(({ joint }) => solidOfJoint.get(joint));
            const G = tendon.parts.map((a, i) => tendon.parts.map((b, j) => (carries(solidsOf[i], solidsOf[j]) || carries(solidsOf[j], solidsOf[i]) ? a.coef * b.coef : 0)));
            const scale = Math.max(...G.map((row, i) => Math.abs(row[i])));
            const rows = [];
            for (const { value, vector } of symmetricEigen(G).sort((a, b) => b.value - a.value)) {
                if (!(Math.abs(value) > 1e-12 * scale)) continue;
                const index = newRow('damping');
                rows.push({ index, weight: round(value) });
                tendon.parts.forEach(({ joint }, i) => { if (Math.abs(vector[i]) > 1e-15) rowsOn(joint).push({ type: 'damping', dof: 0, index, J: round(vector[i]), external: true }); });
            }
            tendonRowsOf.get(tendon.id).damping = rows;
        }
    }
    const tendonsOn = new Map();
    for (const tendon of tendons) for (const { joint } of tendon.parts) tendonsOn.set(solidOfJoint.get(joint), true);
    const R = offset;
    const path = new Set();
    const affected = new Set();
    const gather = new Set();
    for (const loop of loops) {
        for (const { solid } of loop.ends) for (let at = solid; at !== groundId; at = parentOf.get(at).parent) path.add(at);
        for (let at = loop.holderId; at !== groundId; at = parentOf.get(at).parent) gather.add(at);
    }
    if (soft) for (const { id: solidId } of order) { path.add(solidId); gather.add(solidId); }
    // With joint rows and no damping rows, a second sweep takes the damping at the step's end speed.
    const secondSweep = soft && !dampingRows;
    // Without joint rows, the loops' forces change the solids' accelerations by a sweep of their own (`affected`);
    // with them, the second sweep takes them.
    if (!soft) for (const { id: solidId } of order) if (path.has(solidId) || affected.has(parentOf.get(solidId).parent)) affected.add(solidId);

    // The joints a motor drives (each a hinge or a slider in the tree), by the solid each carries.
    const motorDriven = new Map(order.map(({ id: solidId }) => [solidId, parentOf.get(solidId).joint])
        .filter(([, joint]) => (joint.kind === 'hinge' || joint.kind === 'slider') && joint.drive?.mode === 'motor'));

    // ---- nodes ---------------------------------------------------------------------------------------------------
    const nodes = new Map();
    const addState = (node, symbol, initialValue, unit = '') => {
        const created = { id: id(), name: stateName(symbol), symbol, initialValue: round(initialValue), unit };
        node.states.push(created);
        node.bySymbol.set(symbol, created);
        return created;
    };
    // Each update in the node's order: `state` grows at `rate`, which reads the states named in it.
    const integrate = (node, state, rate) => {
        const latex = E.toLatex(rate);
        node.sourceTerms.push({
            id: id(), state: state.symbol, expression: latex, parameters: [],
            expressionModel: {
                latex, mathJson: rate, output: { stateId: state.id },
                bindings: [...E.symbolsOf(rate)].map((symbol) => ({ symbol, kind: 'state', stateId: node.bySymbol.get(symbol).id }))
            }
        });
    };
    const rotationStates = (node, pose, { position = true } = {}) => {
        const matrix = rotationMatrix(pose.rotation);
        const [roll, pitch, yaw] = toRollPitchYaw(pose.rotation);
        if (position) ['x', 'y', 'z'].forEach((symbol, axis) => addState(node, symbol, pose.position[axis], 'm'));
        addState(node, 'roll', roll, 'rad');
        addState(node, 'pitch', pitch, 'rad');
        addState(node, 'yaw', yaw, 'rad');
        rotationSymbols.forEach((row, rowIndex) => row.forEach((symbol, column) => addState(node, symbol, matrix[rowIndex][column])));
    };
    const velocityStates = (node) => velocitySymbols.forEach((symbol) => addState(node, symbol, 0, symbol.startsWith('w') ? 'rad/s' : 'm/s'));

    const ground = { id: id(), name: 'Ground', type: 'robotGround', position: [0, 0, 0], states: [], sourceTerms: [], bySymbol: new Map(),
        appearance: { type: 'primitive', shape: 'box', color: '#7b8794' } };
    rotationStates(ground, poseOf(drawing, groundId));
    velocityStates(ground);
    // Holding still under gravity is, in these terms, accelerating upward at g.
    accelerationSymbols.forEach((symbol, index) => addState(ground, symbol, index === 5 ? gravity : 0, index < 3 ? 'rad/s²' : 'm/s²'));
    if (R) {
        lowerCoupling(R).forEach((symbol) => addState(ground, symbol, 0));
        wantedSymbols(R).forEach((symbol) => addState(ground, symbol, 0));
        rowSettingSymbols(rowTypes).forEach((symbol) => addState(ground, symbol, 0));
    }
    nodes.set(groundId, ground);

    const rows = new Map();
    for (const { id: solidId, depth } of order) {
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const { joint } = parentOf.get(solidId);
        const pose = poseOf(drawing, solid.id);
        const row = rows.get(depth) ?? 0;
        rows.set(depth, row + 1);
        const node = {
            id: id(), name: solid.name, type: 'robotSolid', position: [5 * (depth + 1), -3.6 * row, 0], states: [], sourceTerms: [], bySymbol: new Map(),
            numerics: { substepsPerGlobalStep: 1, integration: 'semiImplicit' },
            appearance: {
                type: 'primitive', shape: solid.shape === 'capsule' ? 'cylinder' : solid.shape === 'mesh' ? 'box' : solid.shape, color: solid.color,
                parts: [{ prefix: '', mesh: solid.shape === 'mesh' ? (looks[solid.look?.key]?.mesh ?? shapeMesh('box', solid.size)) : shapeMesh(solid.shape, solid.size), color: solid.color }]
            }
        };
        // The joint's speeds first, then its coordinates from the new speeds, each from the ones updated before it.
        if (joint.kind === 'hinge' || joint.kind === 'slider') {
            const linear = joint.kind === 'slider';
            const speed = addState(node, 'jointSpeed', 0, linear ? 'm/s' : 'rad/s');
            const position = addState(node, linear ? 'jointPosition' : 'jointAngle', 0, linear ? 'm' : 'rad');
            const acceleration = addState(node, 'jointAcceleration', 0, linear ? 'm/s²' : 'rad/s²');
            addState(node, 'jointTorque', 0, linear ? 'N' : 'N·m');
            integrate(node, speed, acceleration.symbol);
            integrate(node, position, speed.symbol);
            rotationStates(node, pose);
            velocityStates(node);
        } else if (joint.kind === 'ball') {
            // Its angular velocity in its parent's frame, and its turn from where it is drawn, in its parent's frame.
            const speeds = [1, 2, 3].map((axis) => addState(node, `jointSpeed${axis}`, 0, 'rad/s'));
            const turn = turnSymbols.ball.map((symbol, index) => addState(node, symbol, index === 0 ? 1 : 0));
            const accelerations = [1, 2, 3].map((axis) => addState(node, `jointAcceleration${axis}`, 0, 'rad/s²'));
            speeds.forEach((speed, axis) => integrate(node, speed, accelerations[axis].symbol));
            quaternionRates(turnSymbols.ball, speeds.map(({ symbol }) => symbol)).forEach((rate, index) => integrate(node, turn[index], rate));
            rotationStates(node, pose);
            velocityStates(node);
        } else if (joint.kind === 'free') {
            // Joined to nothing: its angular velocity and its centre's velocity, its centre and its rotation are what it
            // integrates (as MuJoCo's free joint does: the centre's velocity, not the velocity at the world's origin, so
            // a solid far from the origin steps as well as one near it). The velocity at the origin is set, as for any
            // solid, for its children's joints to read.
            ['w1', 'w2', 'w3'].forEach((symbol) => addState(node, symbol, 0, 'rad/s'));
            const centreVelocity = centreVelocitySymbols.map((symbol) => addState(node, symbol, 0, 'm/s'));
            ['x', 'y', 'z'].forEach((symbol, axis) => addState(node, symbol, pose.position[axis], 'm'));
            const turn = turnSymbols.free.map((symbol, index) => addState(node, symbol, pose.rotation[index]));
            const accelerations = [1, 2, 3, 4, 5, 6].map((axis) => addState(node, `jointAcceleration${axis}`, 0, axis <= 3 ? 'rad/s²' : 'm/s²'));
            ['w1', 'w2', 'w3', ...centreVelocitySymbols].forEach((symbol, index) => integrate(node, node.bySymbol.get(symbol), accelerations[index].symbol));
            ['x', 'y', 'z'].forEach((symbol, axis) => integrate(node, node.bySymbol.get(symbol), centreVelocity[axis].symbol));
            quaternionRates(turnSymbols.free, ['w1', 'w2', 'w3']).forEach((rate, index) => integrate(node, turn[index], rate));
            ['v1', 'v2', 'v3'].forEach((symbol) => addState(node, symbol, 0, 'm/s'));
            rotationStates(node, pose, { position: false });
        } else {
            rotationStates(node, pose);
            velocityStates(node);
        }
        if (motorDriven.has(solidId)) addState(node, 'motorTorque', 0, joint.kind === 'slider' ? 'N' : 'N·m');
        accelerationSymbols.forEach((symbol) => addState(node, symbol, 0));
        symmetric.forEach(([row, column]) => addState(node, inertiaSymbol(row, column), 0));
        biasSymbols.forEach((symbol) => addState(node, symbol, 0));
        if (path.has(solidId)) {
            sensitivitySymbols('lp', R).flat().forEach((symbol) => addState(node, symbol, 0));
            sensitivitySymbols('la', R).flat().forEach((symbol) => addState(node, symbol, 0));
            if (!soft) biasChangeSymbols.forEach((symbol) => addState(node, symbol, 0));
        }
        if (secondSweep) {
            symmetric.forEach(([row, column]) => addState(node, dampedInertiaSymbol(row, column), 0));
            dampedBiasSymbols.forEach((symbol) => addState(node, symbol, 0));
            dampedAccelerationSymbols.forEach((symbol) => addState(node, symbol, 0));
            externalSymbols.forEach((symbol) => addState(node, symbol, 0));
        }
        if (affected.has(solidId)) accelerationChangeSymbols.forEach((symbol) => addState(node, symbol, 0));
        // The tendons' pull on its joint, and the rows' J on it that another relationship sets.
        if (tendonsOn.has(solidId)) addState(node, 'tendonForce', 0, joint.kind === 'slider' ? 'N' : 'N·m');
        for (const row of jointRows.get(solidId) ?? []) if (row.J === 'state') addState(node, `rj${row.index + 1}`, 0);
        if (gather.has(solidId)) {
            lowerCoupling(R).forEach((symbol) => addState(node, symbol, 0));
            wantedSymbols(R).forEach((symbol) => addState(node, symbol, 0));
            forceSymbols(R).forEach((symbol) => addState(node, symbol, 0, ''));
            rowSettingSymbols(rowTypes).forEach((symbol) => addState(node, symbol, 0));
            // On a solid hanging from the Ground, the constraint rows' forces the step before, where its search for
            // them starts (each the force's value as the state's next).
            if (soft && parentOf.get(solidId).parent === groundId) {
                rowTypes.forEach((type, index) => {
                    if (!searched(type)) return;
                    const warm = addState(node, `warm${index + 1}`, 0);
                    integrate(node, warm, E.multiply(round(1 / timeStep), E.subtract(forceSymbols(R)[index], warm.symbol)));
                });
            }
        }
        addState(node, 'kineticEnergy', 0, 'J');
        addState(node, 'potentialEnergy', 0, 'J');
        nodes.set(solid.id, node);
    }
    for (const loop of loops) {
        // The force and moment the closing joint puts on its holder, and the gap and twist left, on the holder.
        const holder = nodes.get(loop.holderId);
        const angular = loop.rows.some(([type]) => type === 'angular');
        const named = (symbol, name, unit) => { addState(holder, symbol, 0, unit).name = `${loop.joint.name} ${name}`; };
        ['x', 'y', 'z'].forEach((axis, index) => named(`${loop.key}Force${index + 1}`, `force ${axis}`, 'N'));
        if (angular) ['x', 'y', 'z'].forEach((axis, index) => named(`${loop.key}Moment${index + 1}`, `moment ${axis}`, 'N·m'));
        named(`${loop.key}Gap`, 'gap', 'm');
        if (angular) named(`${loop.key}Twist`, 'twist', 'rad');
        loop.holder = holder;
    }
    // Each tendon, a node of its own: its length and speed (each joint's share added by its relationship), the force it
    // pulls with and, with a limit, which end it is past.
    const tendonNodes = new Map();
    tendons.forEach((tendon, index) => {
        const linear = tendon.parts.every(({ joint }) => parentOf.get(solidOfJoint.get(joint)).joint.kind === 'slider');
        const node = {
            id: id(), name: tendon.name, type: 'robotTendon', position: [-5, -3.6 * index, 0], states: [], sourceTerms: [], bySymbol: new Map(),
            numerics: { substepsPerGlobalStep: 1, integration: 'semiImplicit' }, appearance: { type: 'primitive', shape: 'cylinder', color: '#c58b4f' }
        };
        addState(node, 'length', tendon.offset, linear ? 'm' : '');
        addState(node, 'speed', 0, linear ? 'm/s' : '1/s');
        addState(node, 'force', 0, linear ? 'N' : '');
        if (tendon.limits) addState(node, 'limitSide', 1);
        tendonNodes.set(tendon.id, node);
    });
    document.nodes.push(...nodes.values(), ...tendonNodes.values());

    // ---- each solid's mass, a shared parameter -------------------------------------------------------------------------
    // A live slider, so a fork can make a solid heavier from a time on (the part in a gripper, say), from a tenth of it
    // to twenty times it (and at least 5 kg more).
    const symbols = new Set();
    const masses = new Map();
    const parameterIndex = [];
    for (const { id: solidId } of order) {
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const mass = massOf(solid);
        const shared = { id: id(), name: `${solid.name} mass`, symbol: camelKey(`${solid.name} mass`, symbols), value: round(mass), unit: 'kg', mode: 'live', control: liveRange(mass / 10, Math.max(20 * mass, mass + 5)) };
        document.sharedParameters.push(shared);
        masses.set(solidId, shared);
        parameterIndex.push(indexEntry('solidMass', solid.name, shared));
    }

    // ---- each driven joint's setpoint, scheduled by the keyframes ------------------------------------------------------
    // A shared parameter per joint the keyframes set, following them in straight lines from one to the next (and
    // holding the first before it and the last after it), a live slider too.
    const setpoints = new Map();
    const keyframes = keyframesOf(drawing);
    for (const joint of drivenJoints(drawing)) {
        const samples = keyframes.filter(({ setpoints: values }) => Number.isFinite(values[joint.id])).map(({ time, setpoints: values }) => [round(time), round(values[joint.id])]);
        if (!samples.length) continue;
        const travel = joint.limits ?? (joint.kind === 'hinge' ? { lower: -Math.PI, upper: Math.PI } : { lower: -0.5, upper: 0.5 });
        const lower = Math.min(travel.lower, ...samples.map(([, value]) => value));
        const upper = Math.max(travel.upper, ...samples.map(([, value]) => value));
        const shared = {
            id: id(), name: `${joint.name} setpoint`, symbol: camelKey(`${joint.name} setpoint`, symbols), value: samples[0][1],
            unit: joint.kind === 'slider' ? 'm' : 'rad', mode: 'live', control: { minimum: round(lower), maximum: round(upper), step: round((upper - lower) / 400) },
            schedule: { interpolation: 'linear', samples }
        };
        document.sharedParameters.push(shared);
        setpoints.set(joint.id, shared);
    }

    // ---- each joint, a relationship --------------------------------------------------------------------------------------
    const weights = weightsGiven ?? inverseWeights(drawing, built);
    for (const { id: childId } of order) {
        const { joint, parent: parentId } = parentOf.get(childId);
        const roles = {
            onPath: path.has(childId), affected: affected.has(childId), parentAffected: affected.has(parentId), gathers: gather.has(childId), R, timeStep,
            setpoint: setpoints.get(joint.id) ?? null, inverseWeight: weights.get(joint.id) ?? null,
            soft, secondSweep, dampingRows, rows: jointRows.get(childId) ?? [], rowTypes, tendonForce: tendonsOn.has(childId)
        };
        document.edges.push(jointRelationship(drawing, joint, nodes.get(parentId), nodes.get(childId), parentId, childId, masses.get(childId), id, roles));
    }
    for (const loop of loops) document.edges.push(closingRelationship(drawing, loop, nodes, path, R, id, timeStep, secondSweep ? 'second' : soft ? 'rows' : 'plain'));
    // ---- couplings and tendons --------------------------------------------------------------------------------------------
    for (const coupling of couplings) {
        const first = solidOfJoint.get(coupling.first);
        const second = coupling.second ? solidOfJoint.get(coupling.second) : groundId;
        const w = weights.get(coupling.first) + (coupling.second ? weights.get(coupling.second) : 0);
        document.edges.push(couplingRelationship(coupling, nodes.get(second), nodes.get(first), parentOf.get(first).joint, coupling.second ? parentOf.get(second).joint : null, couplingRowOf.get(coupling.id), w, R, id, timeStep));
    }
    tendons.forEach((tendon) => {
        const parts = tendon.parts.map(({ joint, coef }) => ({ coef, node: nodes.get(solidOfJoint.get(joint)), joint: parentOf.get(solidOfJoint.get(joint)).joint }));
        document.edges.push(...tendonRelationships(tendon, parts, tendonNodes.get(tendon.id), tendonRowsOf.get(tendon.id), weights.get(tendon.id), R, id, timeStep, symbols, document));
    });

    // ---- motors and their battery ----------------------------------------------------------------------------------------
    if (motorDriven.size) parameterIndex.push(...motorsAndBattery(drawing, motorDriven, nodes, document, setpoints, symbols, id));
    for (const node of document.nodes) delete node.bySymbol;

    const runConfigurationId = id();
    document.runConfigurations = [{ id: runConfigurationId, name: `${runTime} s`, targetTime: runTime, globalTimeStep: timeStep, outputInterval }];
    document.activeRunConfigurationId = runConfigurationId;
    return {
        document,
        parameterIndex,
        data: {
            solids: drawing.solids.length,
            joints: drawing.joints.length,
            driven: drawing.joints.filter((joint) => joint.drive?.mode && joint.drive.mode !== 'free').length,
            nodes: document.nodes.length,
            edges: document.edges.length,
            equations: document.edges.reduce((sum, edge) => sum + (edge.equationModel.outputs?.length ?? 1), 0)
        },
        report: { warnings: check.warnings.map(({ message }) => message), notes: check.notes.map(({ message }) => message) }
    };
}

// A relationship's bindings, definitions and outputs, as the joint relationships and the closing one write them.
function equations() {
    const bindings = [];
    const definitions = [];
    const outputs = [];
    const bind = (node, role, symbol, prefix) => {
        const state = node.bySymbol.get(symbol);
        if (!state) throw new Error(`${node.name} has no state ${symbol}.`);
        const bound = `${prefix}${symbol[0].toUpperCase()}${symbol.slice(1)}`;
        if (!bindings.some((binding) => binding.symbol === bound)) bindings.push({ symbol: bound, kind: 'state', role, nodeId: node.id, stateId: state.id });
        return bound;
    };
    const define = (symbol, expression) => {
        if (typeof expression === 'number' || typeof expression === 'string') return expression;
        definitions.push({ symbol, mathJson: expression, latex: E.toLatex(expression) });
        return symbol;
    };
    const defineAll = (prefix, values) => values.map((value, index) => define(`${prefix}${index + 1}`, value));
    const defineMatrix = (prefix, matrix) => matrix.map((row, rowIndex) => row.map((value, column) => define(`${prefix}${rowIndex + 1}${column + 1}`, value)));
    const output = (node, role, symbol, expression) => {
        const state = node.bySymbol.get(symbol);
        outputs.push({ role, stateId: state.id, mathJson: expression, latex: E.toLatex(expression), setsValue: true });
    };
    // Only the definitions an equation needs, directly or through others.
    const pruned = () => {
        const needed = new Set();
        const byName = new Map(definitions.map((definition) => [definition.symbol, definition]));
        const visit = (expression) => {
            for (const symbol of E.symbolsOf(expression)) {
                if (!byName.has(symbol) || needed.has(symbol)) continue;
                needed.add(symbol);
                visit(byName.get(symbol).mathJson);
            }
        };
        outputs.forEach(({ mathJson }) => visit(mathJson));
        return definitions.filter(({ symbol }) => needed.has(symbol));
    };
    return { bindings, definitions, outputs, bind, define, defineAll, defineMatrix, output, pruned };
}

// One joint's relationship: from the solid nearer the Ground (`parentNode`) to the one it carries (`childNode`).
// `roles` says what it carries for the loop: its child is on the loop's path back to the Ground (`onPath`), or moves
// with the loop's force (`affected`), as may its parent (`parentAffected`).
function jointRelationship(drawing, joint, parentNode, childNode, parentId, childId, massParameter, id, roles) {
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const kind = joint.kind;
    const hinge = kind === 'hinge';
    const slider = kind === 'slider';
    const ball = kind === 'ball';
    const free = kind === 'free';
    const dof = degreesOfFreedom[kind];
    const moving = dof > 0;
    const parentIsGround = parentId === groundId;
    const child = drawing.solids.find(({ id: solidId }) => solidId === childId);
    const { bindings, outputs, bind, define, defineAll, defineMatrix, output, pruned } = equations();

    // ---- the joint's geometry, in the parent's frame where the drawing has them, as numbers ------------------------------
    const parentPose = poseOf(drawing, parentId);
    const childPose = poseOf(drawing, childId);
    const restRotation = rotationMatrix(multiplyQuaternions(conjugate(parentPose.rotation), childPose.rotation)).map((row) => row.map(round));
    const restOffset = toLocal(parentPose, childPose.position).map(round);
    let a = [0, 0, 0];
    let n = [0, 0, 1];
    if (!free) {
        const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
        // A joint read the other way round (its first frame on the solid further from the Ground) turns the other way.
        const flipped = solidOf.get(joint.child) !== childId;
        a = toLocal(parentPose, anchor.point).map(round);
        n = normalize(scale(directionToLocal(parentPose, anchor.axis), flipped ? -1 : 1)).map(round);
    }
    const unit = inertiaOf(child).map(round);

    // ---- parameters ----------------------------------------------------------------------------------------------------------
    const parameters = [];
    const parameter = (symbol, name, value, unitName, control = null, sharedParameterId = null) => {
        const created = {
            id: id(), name, symbol, value: round(value), unit: unitName, mode: control ? 'live' : 'constant',
            ...(control ? { control } : {}), ...(sharedParameterId ? { sharedParameterId } : {})
        };
        parameters.push(created);
        return created;
    };
    parameter('mass', `${child.name} mass`, massParameter.value, 'kg', null, massParameter.id);
    const linearUnits = { position: 'm', damping: 'N·s/m', friction: 'N', armature: 'kg', effort: 'N', stiffness: 'N/m' };
    const angularUnits = { position: 'rad', damping: 'N·m·s/rad', friction: 'N·m', armature: 'kg·m²', effort: 'N·m', stiffness: 'N·m/rad' };
    const units = slider ? linearUnits : angularUnits;
    const drive = (hinge || slider || ball) ? joint.drive ?? { mode: 'free' } : { mode: 'free' };
    const range = (value, floor, minimum = null) => {
        const reach = Math.max(floor, Math.abs(value) * 2);
        return { minimum: round(Math.min(minimum ?? -reach, value)), maximum: round(Math.max(reach, value)), step: round(reach / 100) };
    };
    if (moving && !free) {
        parameter('damping', 'Damping', joint.damping ?? 0, units.damping, range(joint.damping ?? 0, slider ? 50 : 0.5, 0));
        parameter('friction', 'Friction', joint.friction ?? 0, units.friction, range(joint.friction ?? 0, slider ? 5 : 0.5, 0));
        // A motor's rotor turns with the joint, through its gearbox: its inertia times the ratio squared adds to the armature.
        const rotor = drive.mode === 'motor' ? (drive.motor?.rotorInertia ?? 0) * (drive.motor?.gearRatio ?? 0) ** 2 : 0;
        parameter('armature', rotor ? 'Armature (with the motor\'s rotor)' : 'Armature', (joint.armature ?? 0) + rotor, units.armature);
        // A hinge's or a slider's spring, pulling it to its rest (taken at the step's start, as MuJoCo takes it).
        if ((hinge || slider) && joint.stiffness > 0) {
            parameter('springStiffness', 'Spring stiffness', joint.stiffness, units.stiffness, range(joint.stiffness, slider ? 100 : 1, 0));
            parameter('springRest', 'Spring rest', joint.springRest ?? 0, units.position);
        }
        parameter('step', 'Time step', roles.timeStep, 's');
        if (joint.limits && !ball) {
            parameter('lower', 'Lower limit', joint.limits.lower, units.position);
            parameter('upper', 'Upper limit', joint.limits.upper, units.position);
        }
        if ((hinge || slider) && (joint.limits || joint.friction > 0)) {
            // Its soft rows' settings (defaultSoftness): its inverse inertia where drawn, and each row's B, K and impedance.
            const soft = softnessOf(joint);
            parameter('inverseWeight', 'Inverse inertia where drawn', roles.inverseWeight, slider ? '1/kg' : '1/(kg·m²)');
            if (joint.friction > 0) {
                const { B } = rowReference(soft.friction, roles.timeStep);
                const imp = impedanceAt(soft.friction.solimp, 0);
                parameter('frictionDamping', 'Friction row damping', B, '1/s');
                parameter('frictionInertia', 'Friction row inertia', imp / ((1 - imp) * roles.inverseWeight), units.armature);
            }
            if (joint.limits) {
                const { B, K } = rowReference(soft.limit, roles.timeStep);
                parameter('limitDamping', 'Limit row damping', B, '1/s');
                parameter('limitStiffness', 'Limit row stiffness', K, '1/s²');
                parameter('margin', 'Limit margin', soft.margin, units.position);
            }
        }
        if (ball) {
            // A ball's limit is its most turn from where it is drawn; its drive a torque about each of its parent's axes,
            // or a servo turning it to a setpoint given as a turn vector (its axis, its length the angle).
            if (joint.limits) parameter('mostTurn', 'Most turn', joint.limits.upper, 'rad');
            const vector = (value) => [0, 1, 2].map((axis) => (Array.isArray(value) ? Number(value[axis]) || 0 : 0));
            if (drive.mode === 'torque') vector(drive.torque).forEach((value, axis) => parameter(`torque${axis + 1}`, `Torque ${'xyz'[axis]}`, value, units.effort, range(value, 2)));
            if (drive.mode === 'servo') {
                vector(drive.setpoint).forEach((value, axis) => parameter(`setpoint${axis + 1}`, `Setpoint turn ${'xyz'[axis]}`, value, 'rad', { minimum: round(-Math.PI), maximum: round(Math.PI), step: round(Math.PI / 200) }));
                parameter('stiffness', 'Servo stiffness', drive.gain, units.stiffness);
                const limit = drive.torqueLimit;
                parameter('torqueLimit', 'Servo torque limit', limit, units.effort, { minimum: round(limit / 100), maximum: round(limit * 3), step: round(limit / 100) });
            }
        } else if (drive.mode === 'torque') parameter('torque', slider ? 'Force' : 'Torque', drive.torque ?? 0, units.effort, range(drive.torque ?? 0, hinge ? 2 : 50));
        if (drive.mode === 'servo' && !ball) {
            const travel = joint.limits ?? (hinge ? { lower: -Math.PI, upper: Math.PI } : { lower: -0.5, upper: 0.5 });
            const setpoint = Math.min(Math.max(drive.setpoint ?? 0, travel.lower), travel.upper);
            parameter('setpoint', 'Setpoint', roles.setpoint?.value ?? setpoint, units.position, roles.setpoint?.control ?? { minimum: round(travel.lower), maximum: round(travel.upper), step: round((travel.upper - travel.lower) / 400) }, roles.setpoint?.id ?? null);
            parameter('stiffness', 'Servo stiffness', drive.gain, units.stiffness);
            const limit = drive.torqueLimit;
            parameter('torqueLimit', slider ? 'Servo force limit' : 'Servo torque limit', limit, units.effort, { minimum: round(limit / 100), maximum: round(limit * 3), step: round(limit / 100) });
            // Its own damping (a D term), inside its limit, as a real servo's (and MuJoCo's position actuator's kv).
            if (drive.damping > 0) parameter('servoDamping', 'Servo damping', drive.damping, units.damping, range(drive.damping, slider ? 50 : 0.5, 0));
        }
    }

    // ---- bindings: the parent's and the child's states, and the parameters ----------------------------------------------------
    for (const created of parameters) bindings.push({ symbol: created.symbol, kind: 'parameter', parameterId: created.id });
    const parentRotation = rotationSymbols.map((row) => row.map((symbol) => bind(parentNode, 'source', symbol, 'p')));
    const parentPosition = ['x', 'y', 'z'].map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    const parentVelocity = velocitySymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    const parentAcceleration = accelerationSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    // The joint's coordinates and speeds.
    let q = null;
    let speeds = [];
    if (hinge || slider) {
        q = bind(childNode, 'target', slider ? 'jointPosition' : 'jointAngle', 'c');
        speeds = [bind(childNode, 'target', 'jointSpeed', 'c')];
    } else if (ball) {
        speeds = [1, 2, 3].map((axis) => bind(childNode, 'target', `jointSpeed${axis}`, 'c'));
    } else if (free) {
        speeds = ['w1', 'w2', 'w3', ...centreVelocitySymbols].map((symbol) => bind(childNode, 'target', symbol, 'c'));
    }
    const articulated = (row, column) => `c${inertiaSymbol(row, column)[0].toUpperCase()}${inertiaSymbol(row, column).slice(1)}`;
    symmetric.forEach(([row, column]) => bind(childNode, 'target', inertiaSymbol(row, column), 'c'));
    const childBias = biasSymbols.map((symbol) => bind(childNode, 'target', symbol, 'c'));
    const IA = (row, column) => articulated(row, column);
    const toChild = (symbol, expression) => output(childNode, 'target', symbol, expression);
    const toParent = (symbol, expression) => output(parentNode, 'source', symbol, expression);

    // ---- the child's place: rotation and position relative to the parent, then in the world ------------------------------------
    let rotation;
    let position;
    if (free) {
        rotation = quaternionMatrix(define, 'rot', turnSymbols.free.map((symbol) => bind(childNode, 'target', symbol, 'c')));
        position = ['x', 'y', 'z'].map((symbol) => bind(childNode, 'target', symbol, 'c'));
    } else {
        let relativeRotation = restRotation;
        let relativeOffset = restOffset;
        if (hinge) {
            const cosine = define('cosq', E.cos(q));
            const sine = define('sinq', E.sin(q));
            const versine = define('verq', E.subtract(1, cosine));
            const skew = [[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]];
            const turn = defineMatrix('jq', [0, 1, 2].map((row) => [0, 1, 2].map((column) => E.add(
                E.multiply(row === column ? 1 : 0, cosine), E.multiply(skew[row][column], sine), E.multiply(n[row] * n[column], versine)))));
            relativeRotation = defineMatrix('rel', E.matrixMatrix(turn, restRotation));
            relativeOffset = defineAll('off', E.matrixVector(turn, restOffset.map((value, axis) => value - a[axis])).map((value, axis) => E.add(value, a[axis])));
        } else if (ball) {
            // The turn about the joint's point, in the parent's frame.
            const turn = quaternionMatrix(define, 'jq', turnSymbols.ball.map((symbol) => bind(childNode, 'target', symbol, 'c')));
            relativeRotation = defineMatrix('rel', E.matrixMatrix(turn, restRotation));
            relativeOffset = defineAll('off', E.matrixVector(turn, restOffset.map((value, axis) => value - a[axis])).map((value, axis) => E.add(value, a[axis])));
        } else if (slider) {
            relativeOffset = defineAll('off', restOffset.map((value, axis) => E.add(value, E.multiply(n[axis], q))));
        }
        rotation = defineMatrix('rot', E.matrixMatrix(parentRotation, relativeRotation));
        position = defineAll('pos', E.matrixVector(parentRotation, relativeOffset).map((value, axis) => E.add(parentPosition[axis], value)));
        ['x', 'y', 'z'].forEach((symbol, axis) => toChild(symbol, position[axis]));
    }
    rotationSymbols.forEach((row, rowIndex) => row.forEach((symbol, column) => toChild(symbol, rotation[rowIndex][column])));
    toChild('roll', E.atan2(rotation[2][1], rotation[2][2]));
    toChild('pitch', E.asin(E.min(1, E.max(-1, E.negate(rotation[2][0])))));
    toChild('yaw', E.atan2(rotation[1][0], rotation[0][0]));

    // ---- the joint's motion: one spatial vector per axis it moves along (S's columns) -------------------------------------------
    // A hinge turns about its axis through its point, a slider moves along it, a ball turns about each of its parent's
    // axes through its point, and a free joint moves along each of the world's six.
    let motions = [];
    if (hinge || slider) {
        const axis = defineAll('axis', E.matrixVector(parentRotation, n));
        if (hinge) {
            const point = defineAll('jp', E.matrixVector(parentRotation, a).map((value, index) => E.add(parentPosition[index], value)));
            motions = [[...axis, ...defineAll('jm', E.cross(point, axis))]];
        } else {
            motions = [[0, 0, 0, ...axis]];
        }
    } else if (ball) {
        const point = defineAll('jp', E.matrixVector(parentRotation, a).map((value, index) => E.add(parentPosition[index], value)));
        motions = [0, 1, 2].map((column) => {
            const axis = parentRotation.map((row) => row[column]);
            return [...axis, ...defineAll(`jm${column + 1}`, E.cross(point, axis))];
        });
    } else if (free) {
        // Its speeds are its angular velocity and its centre's velocity: turning about each world axis through its
        // centre c, and moving along each.
        motions = [0, 1, 2].map((column) => {
            const axis = [0, 1, 2].map((row) => (row === column ? 1 : 0));
            return [...axis, ...E.cross(position, axis)];
        }).concat([0, 1, 2].map((column) => [0, 0, 0, ...[0, 1, 2].map((row) => (row === column ? 1 : 0))]));
    }
    // The joint's own velocity, S q̇, and the child's: its parent's plus the joint's (a free joint's parent, the Ground,
    // is still).
    const jointVelocity = [0, 1, 2, 3, 4, 5].map((row) => E.add(motions.map((motion, k) => E.multiply(motion[row], speeds[k]))));
    const velocity = free ? [...speeds.slice(0, 3), ...defineAll('vel', jointVelocity.slice(3))]
        : defineAll('vel', parentVelocity.map((value, index) => E.add(value, jointVelocity[index])));
    if (free) ['v1', 'v2', 'v3'].forEach((symbol, index) => toChild(symbol, velocity[3 + index]));
    else velocitySymbols.forEach((symbol, index) => toChild(symbol, velocity[index]));

    // The child's own spatial inertia about the origin: its rotational inertia about its centre turned into the world,
    // shifted to the origin, and its mass.
    const mass = 'mass';
    const centre = position;
    const rotational = defineMatrix('ic', [0, 1, 2].map((row) => [0, 1, 2].map((column) => E.multiply(mass, E.add(
        [0, 1, 2].map((axis) => E.multiply(unit[axis], rotation[row][axis], rotation[column][axis])))))));
    const centreSquared = define('cc', E.dot(centre, centre));
    const skewCentre = [[0, E.negate(centre[2]), centre[1]], [centre[2], 0, E.negate(centre[0])], [E.negate(centre[1]), centre[0], 0]];
    const own = (row, column) => {
        // 1-based rows and columns of the 6 by 6 inertia.
        if (row <= 3 && column <= 3) {
            const [i, j] = [row - 1, column - 1];
            return E.add(rotational[i][j], E.multiply(mass, E.subtract(i === j ? centreSquared : 0, E.multiply(centre[i], centre[j]))));
        }
        if (row <= 3) return E.multiply(mass, skewCentre[row - 1][column - 4]);
        if (column <= 3) return E.multiply(mass, skewCentre[column - 1][row - 4]);
        return row === column ? mass : 0;
    };
    const ownInertia = new Map(symmetric.map(([row, column]) => [`${row}${column}`, define(`io${row}${column}`, own(row, column))]));
    const I = (row, column) => ownInertia.get(`${Math.min(row, column)}${Math.max(row, column)}`);
    symmetric.forEach(([row, column]) => toChild(inertiaSymbol(row, column), I(row, column)));
    // Its momentum, and its own bias force: velocity cross momentum.
    const momentum = defineAll('mom', [1, 2, 3, 4, 5, 6].map((row) => E.add([1, 2, 3, 4, 5, 6].map((column) => E.multiply(I(row, column), velocity[column - 1])))));
    const angular = velocity.slice(0, 3);
    const linear = velocity.slice(3);
    const forceCross = [
        ...E.cross(angular, momentum.slice(0, 3)).map((value, index) => E.add(value, E.cross(linear, momentum.slice(3))[index])),
        ...E.cross(angular, momentum.slice(3))
    ];
    biasSymbols.forEach((symbol, index) => toChild(symbol, forceCross[index]));
    toChild('kineticEnergy', E.multiply(0.5, E.dot(velocity, momentum)));
    toChild('potentialEnergy', E.multiply(mass, gravity, centre[2]));

    // The loops' states this relationship reads and passes on.
    const loopBiasSymbols = sensitivitySymbols('lp', roles.R);
    const loopAccelerationSymbols = sensitivitySymbols('la', roles.R);
    const loopColumns = Array.from({ length: roles.R }, (_, column) => column);
    const loopBias = roles.onPath ? loopBiasSymbols.map((row) => row.map((symbol) => bind(childNode, 'target', symbol, 'c'))) : null;
    const biasChange = roles.onPath && !roles.soft ? biasChangeSymbols.map((symbol) => bind(childNode, 'target', symbol, 'c')) : [0, 0, 0, 0, 0, 0];
    const parentOnPath = roles.onPath && !parentIsGround;
    const parentLoopAcceleration = parentOnPath ? loopAccelerationSymbols.map((row) => row.map((symbol) => bind(parentNode, 'source', symbol, 'p'))) : null;
    const parentAccelerationChange = roles.parentAffected ? accelerationChangeSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p')) : [0, 0, 0, 0, 0, 0];
    const columnOf = (matrix, column) => matrix.map((row) => row[column]);
    // What finds the loops' forces is passed in towards the Ground and gathered there; on a solid hanging from the
    // Ground, the forces are found from it (K + εI, by its L D Lᵀ factors) and handed out again, one solid to the next.
    // The joint's own rows' entries of K, wanted accelerations and settings, by symbol, added to its child's as they
    // pass inward (filled in once the joint's acceleration and sensitivities are known).
    const ownGathered = new Map();
    let foundForces = null;
    const forcesOf = () => { foundForces ??= gatheredForces(); return foundForces; };
    const passGathered = () => {
        if (!roles.gathers) return;
        const plus = (symbol) => (ownGathered.has(symbol) ? E.add(bind(childNode, 'target', symbol, 'c'), ownGathered.get(symbol)) : bind(childNode, 'target', symbol, 'c'));
        lowerCoupling(roles.R).forEach((symbol) => toParent(symbol, plus(symbol)));
        wantedSymbols(roles.R).forEach((symbol) => toParent(symbol, plus(symbol)));
        rowSettingSymbols(roles.rowTypes).forEach((symbol) => toParent(symbol, plus(symbol)));
        const forces = forcesOf();
        forceSymbols(roles.R).forEach((symbol, index) => toChild(symbol, forces[index]));
    };
    // The rows' forces: handed out from the parent; or, on a solid hanging from the Ground, found from what the Ground
    // gathered. Without joint rows, the loops' forces solve (K + εI) λ = wanted, by its L D Lᵀ factors. With them, as
    // MuJoCo finds them: the loops' rows (held, as before) and the constraint rows (each a friction held to ±its friction,
    // or a limit pushing only, with its compliance R) from the solids' inertia alone, the loops' rows taken out exactly
    // and the constraint rows found by projected Gauss-Seidel, starting from the step before's forces, then polished.
    function gatheredForces() {
        if (!roles.gathers) return Array(roles.R).fill(0);
        if (!parentIsGround) return forceSymbols(roles.R).map((symbol) => bind(parentNode, 'source', symbol, 'p'));
        parameter('compliance', 'Loop compliance', closure.compliance, 'm/(s²·N)');
        bindings.push({ symbol: 'compliance', kind: 'parameter', parameterId: parameters.at(-1).id });
        const coupling = couplingSymbols(roles.R);
        const K = (i, j) => bind(parentNode, 'source', coupling[Math.max(i, j)][Math.min(i, j)], 'p');
        const wanted = wantedSymbols(roles.R).map((symbol) => bind(parentNode, 'source', symbol, 'p'));
        if (!roles.soft) {
            const lower = coupling.map((row, i) => row.map((_, j) => (j > i ? null : E.add(K(i, j), i === j ? 'compliance' : 0))));
            const solveLoops = symmetricSolver(define, 'kl', lower.map((row, i) => row.map((_, j) => lower[Math.max(i, j)][Math.min(i, j)])));
            return solveLoops(wanted, 'lm');
        }
        const types = roles.rowTypes;
        const of = (type) => types.map((value, index) => (value === type ? index : -1)).filter((index) => index >= 0);
        const held = of('loop');
        const rows = types.map((value, index) => (searched(value) ? index : -1)).filter((index) => index >= 0);
        const setting = (prefix, index) => bind(parentNode, 'source', `${prefix}${index + 1}`, 'p');
        // The loops' rows taken out: X = (K_EE + εI)⁻¹ [w_E, K_EC].
        let A = (r, c) => K(r, c);
        let b = (r) => wanted[r];
        let held0 = [];
        let heldPer = () => [];
        if (held.length) {
            const solveHeld = symmetricSolver(define, 'ke', held.map((i) => held.map((j) => E.add(K(Math.max(i, j), Math.min(i, j)), i === j ? 'compliance' : 0))));
            held0 = solveHeld(held.map((i) => wanted[i]), 'kew');
            const columns = new Map(rows.map((c) => [c, solveHeld(held.map((i) => K(i, c)), `kec${c + 1}`)]));
            heldPer = (c) => columns.get(c);
            const reducedA = new Map();
            A = (r, c) => {
                const key = `${r},${c}`;
                if (!reducedA.has(key)) reducedA.set(key, define(`ka${r + 1}x${c + 1}`, E.subtract(K(r, c), E.add(held.map((e, k) => E.multiply(K(r, e), columns.get(c)[k]))))));
                return reducedA.get(key);
            };
            b = (r) => define(`kbr${r + 1}`, E.subtract(wanted[r], E.add(held.map((e, k) => E.multiply(K(r, e), held0[k])))));
        }
        // Projected Gauss-Seidel, from the step before's forces.
        let f = new Map(rows.map((r) => [r, bind(childNode, 'target', `warm${r + 1}`, 'c')]));
        const fixedB = new Map(rows.map((r) => [r, b(r)]));
        for (let sweep = 0; sweep < constraintSweeps; sweep += 1) {
            for (const r of rows) {
                const compliance = setting('kc', r);
                const residual = E.subtract(fixedB.get(r), E.add(...rows.map((c) => E.multiply(A(r, c), f.get(c))), E.multiply(compliance, f.get(r))));
                const next = E.add(f.get(r), E.divide(residual, E.add(A(r, r), compliance)));
                f.set(r, define(`pg${sweep + 1}x${r + 1}`, E.max(setting('kn', r), E.min(setting('kx', r), next))));
                f = new Map(f);
            }
        }
        // The rows inside their bounds solved exactly, the others held at theirs, taken where that is consistent (each
        // solved force inside its bounds, each held one pushing against its bound); where not, once more from that
        // solution held to the bounds; and the sweeps' forces where neither is.
        const low = (r) => setting('kn', r);
        const high = (r) => setting('kx', r);
        const polish = (start, tag) => {
            const inside = new Map(rows.map((r) => [r, define(`${tag}in${r + 1}`, E.cases([[E.greater(start.get(r), low(r)), E.cases([[E.less(start.get(r), high(r)), 1]], 0)]], 0))]));
            const out = (r) => E.subtract(1, inside.get(r));
            const values = symmetricSolver(define, `${tag}o`, rows.map((i) => rows.map((j) => (i === j
                ? E.add(E.multiply(inside.get(i), E.add(A(i, i), setting('kc', i))), out(i))
                : E.multiply(inside.get(i), inside.get(j), A(Math.max(i, j), Math.min(i, j)))))))(rows.map((i) => E.add(
                E.multiply(inside.get(i), E.subtract(fixedB.get(i), E.add(...rows.filter((j) => j !== i).map((j) => E.multiply(out(j), A(i, j), start.get(j)))))),
                E.multiply(out(i), start.get(i)))), `${tag}g`);
            const fits = rows.map((r, k) => {
                const value = values[k];
                const residual = E.subtract(fixedB.get(r), E.add(...rows.map((c, m) => E.multiply(A(r, c), values[m])), E.multiply(setting('kc', r), value)));
                // A row with no room (a limit not reached, a friction of nothing) fits as it is.
                return define(`${tag}fit${r + 1}`, E.cases([
                    [E.less(E.subtract(high(r), low(r)), 1e-300), 1],
                    [E.greater(inside.get(r), 0.5), E.cases([[E.less(value, low(r)), 0], [E.greater(value, high(r)), 0]], 1)],
                    [E.greater(start.get(r), low(r)), E.cases([[E.less(residual, 0), 0]], 1)]
                ], E.cases([[E.greater(residual, 0), 0]], 1)));
            });
            return { values, fits: define(`${tag}fit`, E.multiply(...fits)) };
        };
        const forces = Array(roles.R).fill(0);
        if (rows.length) {
            const first = polish(f, 'p');
            const second = polish(new Map(rows.map((r, k) => [r, define(`pc${r + 1}`, E.max(low(r), E.min(high(r), first.values[k])))])), 'q');
            rows.forEach((r, k) => { forces[r] = define(`pf${r + 1}`, E.cases([[E.greater(first.fits, 0.5), first.values[k]], [E.greater(second.fits, 0.5), second.values[k]]], f.get(r))); });
        }
        // The loops' rows from the constraint rows' forces as found.
        held.forEach((e, k) => { forces[e] = define(`kh${e + 1}`, E.subtract(held0[k], E.add(...rows.map((c) => E.multiply(heldPer(c)[k], forces[c]))))); });
        // The damping rows, from the others' forces: (I + H K) μ = H (wanted - K f), H their h b (a tendon's rows' with
        // the sign of their weight). As (Σ + S K S) ν = S (wanted - K f), μ = S ν, S = √|h b| and Σ its sign.
        const damped = of('damping');
        if (damped.length) {
            const S = new Map(damped.map((d) => [d, define(`ks${d + 1}`, E.sqrt(E.max(setting('kc', d), E.negate(setting('kc', d)))))]));
            const sign = (d) => E.cases([[E.less(setting('kc', d), 0), -1]], 1);
            const others = types.map((_, index) => index).filter((index) => types[index] !== 'damping');
            const right = damped.map((d) => E.multiply(S.get(d), E.subtract(wanted[d], E.add(0, ...others.map((c) => E.multiply(K(d, c), forces[c]))))));
            const solveDamping = symmetricSolver(define, 'kd', damped.map((i) => damped.map((j) => E.add(E.multiply(S.get(i), K(i, j), S.get(j)), i === j ? sign(i) : 0))));
            solveDamping(right, 'kdn').forEach((value, k) => { forces[damped[k]] = define(`kdf${damped[k] + 1}`, E.multiply(S.get(damped[k]), value)); });
        }
        return forces;
    }

    // The articulated inertia and bias force the child has (its own and what its children pass up), as bound.
    const articulatedRow = (row) => [1, 2, 3, 4, 5, 6].map((column) => IA(row, column));
    if (!moving) {
        // Fixed: the child moves with its parent, so it passes everything up, and accelerates as its parent does.
        if (!parentIsGround) {
            symmetric.forEach(([row, column]) => toParent(inertiaSymbol(row, column), IA(row, column)));
            biasSymbols.forEach((symbol, index) => toParent(symbol, childBias[index]));
        }
        accelerationSymbols.forEach((symbol, index) => toChild(symbol, parentAcceleration[index]));
        if (roles.onPath) {
            if (parentOnPath) {
                loopBiasSymbols.forEach((row, r) => row.forEach((symbol, column) => toParent(symbol, loopBias[r][column])));
                if (!roles.soft) biasChangeSymbols.forEach((symbol, index) => toParent(symbol, biasChange[index]));
            }
            loopAccelerationSymbols.forEach((row, r) => row.forEach((symbol, column) => toChild(symbol, parentOnPath ? parentLoopAcceleration[r][column] : 0)));
        }
        if (roles.affected) accelerationChangeSymbols.forEach((symbol, index) => toChild(symbol, parentAccelerationChange[index]));
        if (roles.secondSweep) {
            // The second sweep: everything passed up, the parent's acceleration carried.
            if (!parentIsGround) {
                symmetric.forEach(([row, column]) => toParent(dampedInertiaSymbol(row, column), bind(childNode, 'target', dampedInertiaSymbol(row, column), 'c')));
                dampedBiasSymbols.forEach((symbol) => toParent(symbol, bind(childNode, 'target', symbol, 'c')));
            }
            const parentDamped = parentIsGround ? parentAcceleration : dampedAccelerationSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
            dampedAccelerationSymbols.forEach((symbol, index) => toChild(symbol, parentDamped[index]));
            symmetric.forEach(([row, column]) => toChild(dampedInertiaSymbol(row, column), I(row, column)));
            dampedBiasSymbols.forEach((symbol, index) => toChild(symbol, E.add(forceCross[index], bind(childNode, 'target', externalSymbols[index], 'c'))));
        }
        passGathered();
        return relationshipEdge();
    }

    // U = IA S, and D = Sᵀ U plus the armature on each axis, and the damping times the time step: the damping taken at
    // the step's end speed (implicitly, as MuJoCo takes it), so a heavily damped light link stays stable.
    // A servo's own damping is taken at the end speed too, while the servo is within its limit; at its limit the torque
    // does not change with speed, as MuJoCo takes it.
    const servoDamped = (hinge || slider) && drive.mode === 'servo' && drive.damping > 0;
    const servoRaw = servoDamped ? define('servoRaw', E.subtract(E.multiply('stiffness', E.subtract('setpoint', q)), E.multiply('servoDamping', speeds[0]))) : null;
    const servoStep = servoDamped
        ? E.cases([[E.greater(servoRaw, 'torqueLimit'), 0], [E.less(servoRaw, E.negate('torqueLimit')), 0]], E.multiply('servoDamping', 'step'))
        : 0;
    const projected = motions.map((motion, k) => defineAll(dof === 1 ? 'u' : `u${k + 1}r`, [1, 2, 3, 4, 5, 6].map((row) => E.dot(articulatedRow(row), motion))));
    const D = motions.map((motion, i) => projected.map((column, j) => {
        const entry = E.dot(motion, column);
        // With joint rows, the damping is a row of its own (rowsOf), taken after the rows' forces are found.
        if (i !== j || free) return entry;
        return roles.soft ? E.add(entry, 'armature') : E.add(entry, 'armature', E.multiply('damping', 'step'), servoStep);
    }));
    let solve;
    if (dof === 1) {
        const effective = define('d', D[0][0]);
        solve = (b) => [E.divide(b[0], effective)];
    } else {
        // Each entry of D defined once, as the solver reads several.
        const named = D.map((row, i) => row.map((value, j) => (j < i ? null : define(`dd${i + 1}${j + 1}`, value))));
        solve = symmetricSolver(define, 'ld', D.map((row, i) => row.map((_, j) => named[Math.min(i, j)][Math.max(i, j)])));
    }
    const transposeTimes = (vector) => projected.map((column) => E.dot(column, vector));
    const motionTimes = (vector) => motions.map((motion) => E.dot(motion, vector));
    const along = (amounts) => [0, 1, 2, 3, 4, 5].map((row) => E.add(motions.map((motion, k) => E.multiply(motion[row], amounts[k]))));
    const projectedAlong = (amounts) => [0, 1, 2, 3, 4, 5].map((row) => E.add(projected.map((column, k) => E.multiply(column[row], amounts[k]))));

    // The joint's forces: a hinge's or a slider's drive, damping, friction (smoothed over a hundredth of a radian per
    // second) and limits (a stiff spring and damper beyond them, scaled by the joint's effective inertia); each axis of a
    // ball its own damping and friction; nothing on a free joint.
    let forces;
    if (hinge || slider) {
        const qd = speeds[0];
        let driveTorque = 0;
        if (drive.mode === 'torque') driveTorque = 'torque';
        // A motor's torque at the joint, as its windings' relationship sets it on the child.
        if (drive.mode === 'motor') driveTorque = bind(childNode, 'target', 'motorTorque', 'c');
        if (drive.mode === 'servo') {
            driveTorque = define('servo', E.max(E.negate('torqueLimit'), E.min('torqueLimit', servoRaw ?? E.multiply('stiffness', E.subtract('setpoint', q)))));
        }
        // The tendons' pull on it, as their relationships set it on the child.
        const pulled = roles.tendonForce ? bind(childNode, 'target', 'tendonForce', 'c') : 0;
        const spring = (hinge || slider) && joint.stiffness > 0 ? E.negate(E.multiply('springStiffness', E.subtract(q, 'springRest'))) : 0;
        forces = [define('tau', E.add(driveTorque, E.negate(E.multiply('damping', qd)), pulled, spring))];
        toChild('jointTorque', driveTorque);
    } else if (ball) {
        const [w, x, y, z] = turnSymbols.ball.map((symbol) => bind(childNode, 'target', symbol, 'c'));
        let driveTorque = [0, 0, 0];
        if (drive.mode === 'torque') driveTorque = ['torque1', 'torque2', 'torque3'];
        if (drive.mode === 'servo') {
            // The setpoint's quaternion d, the turn left to make (q ⊗ d*), and a torque of the stiffness times its sine
            // about its axis (the stiffness times the angle, for a small turn), held to the servo's most torque.
            const setpoint = ['setpoint1', 'setpoint2', 'setpoint3'];
            const angle = define('sa', E.sqrt(E.dot(setpoint, setpoint)));
            const factor = define('sf', E.cases([[E.greater(angle, 1e-9), E.divide(E.sin(E.multiply(0.5, angle)), angle)]], 0.5));
            const dw = define('sw', E.cos(E.multiply(0.5, angle)));
            const d = defineAll('sd', setpoint.map((value) => E.multiply(value, factor)));
            const v = [x, y, z];
            const ew = define('ew', E.add(E.multiply(w, dw), E.dot(v, d)));
            const across = E.cross(v, d);
            const ev = defineAll('ev', [0, 1, 2].map((axis) => E.subtract(E.subtract(E.multiply(dw, v[axis]), E.multiply(w, d[axis])), across[axis])));
            const raw = defineAll('sr', ev.map((value) => E.multiply(-2, 'stiffness', ew, value)));
            const held = define('sk', E.min(1, E.divide('torqueLimit', E.max(E.sqrt(E.dot(raw, raw)), 1e-12))));
            driveTorque = defineAll('servo', raw.map((value) => E.multiply(value, held)));
        }
        let limitTorque = [0, 0, 0];
        if (joint.limits) {
            // Beyond its most turn, a stiff spring and damper back towards it along the turn's axis, through the joint's
            // effective inertia D (so every way it turns is held alike, a thin link's spin as its swing).
            const along = define('tv', E.max(E.sqrt(E.add(E.multiply(x, x), E.multiply(y, y), E.multiply(z, z))), 1e-12));
            const scalar = define('tsw', E.max(E.sqrt(E.multiply(w, w)), 1e-12));
            const turned = define('tth', E.multiply(2, E.atan2(along, scalar)));
            const beyond = define('tb', E.cases([[E.greater(turned, 'mostTurn'), 1]], 0));
            const push = defineAll('tu', [x, y, z].map((value, axis) => E.multiply(beyond, E.add(
                E.multiply(10000, E.subtract(turned, 'mostTurn'), E.divide(E.multiply(value, w), E.multiply(along, scalar))), E.multiply(200, speeds[axis])))));
            const entry = (i, j) => `dd${Math.min(i, j) + 1}${Math.max(i, j) + 1}`;
            limitTorque = [0, 1, 2].map((i) => define(`tl${i + 1}`, E.negate(E.add([0, 1, 2].map((j) => E.multiply(entry(i, j), push[j]))))));
        }
        forces = speeds.map((speed, k) => define(`tau${k + 1}`, E.add(driveTorque[k], E.negate(E.multiply('damping', speed)), E.negate(E.multiply('friction', E.tanh(E.multiply(100, speed)))), limitTorque[k])));
    } else {
        forces = speeds.map(() => 0);
    }
    const remaining = dof === 1 ? [define('ur', E.subtract(forces[0], E.dot(motions[0], childBias)))]
        : defineAll('ur', forces.map((force, k) => E.subtract(force, E.dot(motions[k], childBias))));
    // The velocity-product acceleration: the child's velocity cross the joint's.
    const motionCross = [
        ...E.cross(angular, jointVelocity.slice(0, 3)),
        ...E.cross(angular, jointVelocity.slice(3)).map((value, index) => E.add(value, E.cross(linear, jointVelocity.slice(0, 3))[index]))
    ];
    // A free joint's axes move with its centre: Ṡ q̇ = [0; ċ × ω] in place of the parent's velocity cross the joint's.
    const bias = defineAll('c', free ? [0, 0, 0, ...E.cross(speeds.slice(3), speeds.slice(0, 3))] : motionCross);

    // ---- the joint's own rows, where the joints' rows are found at the Ground (roles.soft) ------------------------------
    // Each row's force acts on one of the joint's axes, J times it (J ±1). As a force on the joint, it changes what the
    // joint passes inward and its acceleration outward, column by column (`ownForce`), as a loop's force does through
    // its ends; its row of K is J times the joint's acceleration per unit of each row's force, and its wanted
    // acceleration aref - J q̈₀, q̈₀ the joint's acceleration found from the solids' inertia alone.
    const ownRows = (roles.rows ?? []).map((row) => {
        const qd = speeds[row.dof];
        const tag = row.index + 1;
        // A row another relationship gives the rest of (a coupling's, a tendon's): its J on this joint alone.
        if (row.external) {
            const J = row.J === 'state' ? bind(childNode, 'target', `rj${tag}`, 'c') : typeof row.J === 'function' ? define(`rj${tag}`, row.J(q)) : row.J;
            return { ...row, J };
        }
        // A damping row (roles.dampingRows): its damping times the step, a servo's own within its limit.
        if (row.type === 'damping') return { ...row, J: 1, compliance: E.add(E.multiply('damping', 'step'), servoStep) };
        if (row.type === 'friction') {
            const on = E.greater('friction', 0);
            return {
                ...row, J: 1, compliance: E.divide(1, 'frictionInertia'),
                low: E.cases([[on, E.negate('friction')]], 0), high: E.cases([[on, 'friction']], 0),
                reference: E.multiply(-1, 'frictionDamping', qd)
            };
        }
        // A limit: past its lower end by r = q - lower - margin < 0, or its upper by r = upper - q - margin < 0.
        const { solimp } = softnessOf(joint).limit;
        const lower = define('qlr', E.subtract(E.subtract(q, 'lower'), 'margin'));
        const upper = define('qur', E.subtract(E.subtract('upper', q), 'margin'));
        const past = define('qpr', E.cases([[E.less(lower, 0), lower], [E.less(upper, 0), upper]], 0));
        const J = define('qj', E.cases([[E.less(lower, 0), 1], [E.less(upper, 0), -1]], 1));
        const imp = define('qim', impedanceAt(solimp, past));
        return {
            ...row, J, compliance: E.divide(E.multiply(E.subtract(1, imp), 'inverseWeight'), imp),
            low: 0, high: E.cases([[E.less(lower, 0), 1e30], [E.less(upper, 0), 1e30]], 0),
            reference: E.subtract(E.multiply(-1, 'limitDamping', J, qd), E.multiply('limitStiffness', imp, past))
        };
    });
    // The force on each of the joint's axes per unit of row `column`'s force, or null if it is not one of its own.
    const ownForce = (column) => {
        const row = ownRows.find(({ index }) => index === column);
        return row ? speeds.map((_, k) => (k === row.dof ? row.J : 0)) : null;
    };
    const lessOwn = (values, column) => { const own = ownForce(column); return own ? values.map((value, k) => E.subtract(value, own[k])) : values; };

    // Inward: what the child passes to its parent (the Ground takes nothing): the articulated inertia less what the
    // joint lets go (IA - U D⁻¹ Uᵀ), and the bias force plus IA c plus U D⁻¹ (u - Uᵀ c).
    if (!parentIsGround) {
        if (dof === 1) {
            symmetric.forEach(([row, column]) => toParent(inertiaSymbol(row, column),
                E.subtract(IA(row, column), E.divide(E.multiply(projected[0][row - 1], projected[0][column - 1]), 'd'))));
            const biasAlong = define('uc', E.dot(projected[0], bias));
            biasSymbols.forEach((symbol, index) => toParent(symbol, E.add(childBias[index], E.dot(articulatedRow(index + 1), bias),
                E.divide(E.multiply(projected[0][index], E.subtract(remaining[0], biasAlong)), 'd'))));
        } else {
            // D⁻¹ Uᵀ, a column per row of the inertia.
            const released = [1, 2, 3, 4, 5, 6].map((column) => solve(projected.map((u) => u[column - 1]), `w${column}`));
            symmetric.forEach(([row, column]) => toParent(inertiaSymbol(row, column),
                E.subtract(IA(row, column), E.add(projected.map((u, k) => E.multiply(u[row - 1], released[column - 1][k]))))));
            const kept = solve(remaining.map((value, k) => E.subtract(value, E.dot(projected[k], bias))), 'pb');
            biasSymbols.forEach((symbol, index) => toParent(symbol, E.add(childBias[index], E.dot(articulatedRow(index + 1), bias), projectedAlong(kept)[index])));
        }
    }

    // Outward again: the joint's acceleration and the child's.
    const carried = defineAll('ap', parentAcceleration.map((value, index) => E.add(value, bias[index])));
    const jointAcceleration = dof === 1
        ? [define('qdd', E.divide(E.subtract(remaining[0], E.dot(projected[0], carried)), 'd'))]
        : solve(remaining.map((value, k) => E.subtract(value, E.dot(projected[k], carried))), 'qa');
    accelerationSymbols.forEach((symbol, index) => toChild(symbol, E.add(carried[index], along(jointAcceleration)[index])));

    // The rows' forces, as found at the Ground: from the parent, or found here on a solid hanging from the Ground; the
    // joint's own rows' forces act on its axes.
    const rowForces = forcesOf();
    const ownTotal = speeds.map((_, k) => E.add(ownRows.filter(({ dof: axis }) => axis === k).map((row) => E.multiply(row.J, rowForces[row.index]))));
    const lessOwnForces = (values) => (ownRows.length ? values.map((value, k) => E.subtract(value, ownTotal[k])) : values);
    const movedColumns = [];

    // The loop, where this joint is on its way or below it: how the child's bias force and acceleration move with the
    // force that closes it (P inward: P + (I - U D⁻¹ Sᵀ) P; A outward: A - S D⁻¹ (Sᵀ P + Uᵀ A)), and, once that force
    // is known, the change it makes (inward, then outward), added to the joint's acceleration.
    let accelerationChange = [0];
    if (roles.onPath) {
        const passed = loopColumns.map((column) => solve(lessOwn(motionTimes(columnOf(loopBias, column)), column), `pp${column + 1}`));
        if (parentOnPath) {
            loopBiasSymbols.forEach((row, r) => row.forEach((symbol, column) => toParent(symbol,
                E.subtract(loopBias[r][column], projectedAlong(passed[column])[r]))));
            if (!roles.soft) {
                const passedChange = solve(lessOwnForces(motionTimes(biasChange)), 'pc');
                biasChangeSymbols.forEach((symbol, index) => toParent(symbol, E.subtract(biasChange[index], projectedAlong(passedChange)[index])));
            }
        }
        loopColumns.forEach((column) => {
            const parentColumn = parentOnPath ? columnOf(parentLoopAcceleration, column) : [0, 0, 0, 0, 0, 0];
            const moved = solve(lessOwn(motionTimes(columnOf(loopBias, column)).map((value, k) => E.add(value, E.dot(projected[k], parentColumn))), column), `pa${column + 1}`);
            movedColumns[column] = moved;
            loopAccelerationSymbols.forEach((row, r) => toChild(row[column], E.subtract(parentColumn[r], along(moved)[r])));
        });
    }
    if (roles.affected) {
        accelerationChange = solve(lessOwnForces(motionTimes(biasChange).map((value, k) => E.add(value, E.dot(projected[k], parentAccelerationChange)))).map(E.negate), 'dq');
        accelerationChangeSymbols.forEach((symbol, index) => toChild(symbol, E.add(parentAccelerationChange[index], along(accelerationChange)[index])));
    }
    for (const row of ownRows) {
        const J = row.J;
        loopColumns.filter((column) => column <= row.index).forEach((column) => ownGathered.set(couplingSymbols(roles.R)[row.index][column], E.multiply(-1, J, movedColumns[column][row.dof])));
        ownGathered.set(wantedSymbols(roles.R)[row.index], E.subtract(row.reference ?? 0, E.multiply(J, jointAcceleration[row.dof])));
        if (row.external) continue;
        ownGathered.set(`kc${row.index + 1}`, row.compliance);
        if (row.type === 'damping') continue;
        ownGathered.set(`kn${row.index + 1}`, row.low);
        ownGathered.set(`kx${row.index + 1}`, row.high);
    }
    let total = jointAcceleration.map((value, k) => (roles.affected ? E.add(value, accelerationChange[k]) : value));
    if (roles.dampingRows) {
        // Every row's force, the damping rows' among them, times how the joint moves with it.
        total = jointAcceleration.map((value, k) => define(`qt${k + 1}`, E.subtract(value, E.add(...loopColumns.map((column) => E.multiply(movedColumns[column][k], rowForces[column]))))));
    }
    if (roles.secondSweep) {
        // The second sweep, with the joints' damping taken at the step's end speed (and a servo's within its limit),
        // the rows' forces on the joints and the loops' on the solids: the joint's acceleration, the one the step takes.
        const dampedIA = (row, column) => bind(childNode, 'target', dampedInertiaSymbol(row, column), 'c');
        symmetric.forEach(([row, column]) => toChild(dampedInertiaSymbol(row, column), I(row, column)));
        dampedBiasSymbols.forEach((symbol, index) => toChild(symbol, E.add(forceCross[index], bind(childNode, 'target', externalSymbols[index], 'c'))));
        const dampedBias = dampedBiasSymbols.map((symbol) => bind(childNode, 'target', symbol, 'c'));
        const dampedRow = (row) => [1, 2, 3, 4, 5, 6].map((column) => dampedIA(row, column));
        const projected2 = motions.map((motion, k) => defineAll(dof === 1 ? 'uz' : `u${k + 1}z`, [1, 2, 3, 4, 5, 6].map((row) => E.dot(dampedRow(row), motion))));
        const D2 = motions.map((motion, i) => projected2.map((column, j) => {
            const entry = E.dot(motion, column);
            return i === j && !free ? E.add(entry, 'armature', E.multiply('damping', 'step'), servoStep) : entry;
        }));
        let solve2;
        if (dof === 1) {
            const effective = define('dz', D2[0][0]);
            solve2 = (b) => [E.divide(b[0], effective)];
        } else {
            const named = D2.map((row, i) => row.map((value, j) => (j < i ? null : define(`dz${i + 1}${j + 1}`, value))));
            solve2 = symmetricSolver(define, 'lz', D2.map((row, i) => row.map((_, j) => named[Math.min(i, j)][Math.max(i, j)])));
        }
        const remaining2 = defineAll('uy', forces.map((force, k) => E.subtract(E.add(force, ownTotal[k]), E.dot(motions[k], dampedBias))));
        if (!parentIsGround) {
            const released = [1, 2, 3, 4, 5, 6].map((column) => solve2(projected2.map((u) => u[column - 1]), `wz${column}`));
            symmetric.forEach(([row, column]) => toParent(dampedInertiaSymbol(row, column),
                E.subtract(dampedIA(row, column), E.add(projected2.map((u, k) => E.multiply(u[row - 1], released[column - 1][k]))))));
            const kept = solve2(remaining2.map((value, k) => E.subtract(value, E.dot(projected2[k], bias))), 'kz');
            dampedBiasSymbols.forEach((symbol, index) => toParent(symbol, E.add(dampedBias[index], E.dot(dampedRow(index + 1), bias),
                E.add(projected2.map((u, k) => E.multiply(u[index], kept[k]))))));
        }
        const parentDamped = parentIsGround ? parentAcceleration : dampedAccelerationSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
        const carried2 = defineAll('az', parentDamped.map((value, index) => E.add(value, bias[index])));
        total = solve2(remaining2.map((value, k) => E.subtract(value, E.dot(projected2[k], carried2))), 'qz');
        dampedAccelerationSymbols.forEach((symbol, index) => toChild(symbol, E.add(carried2[index], along(total)[index])));
    }
    if (dof === 1) toChild('jointAcceleration', total[0]);
    else total.forEach((value, k) => toChild(`jointAcceleration${k + 1}`, value));
    passGathered();
    return relationshipEdge();

    function relationshipEdge() {
        return {
            id: id(), name: joint.name, directionality: 'directed',
            source: { nodeId: parentNode.id }, target: { nodeId: childNode.id },
            parameters,
            equation: '',
            equationModel: { bindings, definitions: pruned(), outputs },
            appearance: { color: moving ? '#4fd1c5' : '#7b8794', offset: 0, waypoints: [] }
        };
    }
}

// A joint that closes a loop: a relationship between its two solids (either may be the Ground) holding what its kind
// holds (closureRows), each row a component along the first solid's frame axes a: a linear row the velocity of the
// second solid's point at the first's point p, relative to the first's (a · (Δv + Δω × p)), an angular row the relative
// angular velocity (a · Δω), Δ the second's spatial velocity less the first's. As G Δ. It
// - gives each end's bias force its share of those rows' forces (+Gᵀ on the first end, -Gᵀ on the second, as the
//   forces push the second solid by +Gᵀ λ);
// - reads how each end's acceleration moves with every loop's forces, and hands its holder its rows of K = G (A₂ - A₁)
//   and of the acceleration wanted, -Ġ Δ - 2 ω c - ω² e - G (â₂ - â₁) (c = G Δ its rows' rates, e their gaps and
//   twists), which the tree gathers into the Ground and solves there;
// - reads its own forces back from its holder, and hands each end the change they make to its bias force.
// `mode`: 'plain' (no joint rows: the forces change the bias forces, a sweep of their own carrying the change), 'second'
// (joint rows and a second sweep: the forces on the solids in it) or 'rows' (damping rows: the joints take the forces
// through how they move with them, so nothing more is handed out).
function closingRelationship(drawing, loop, nodes, path, R, id, timeStep, mode = 'plain') {
    const { bindings, outputs, bind, define, defineAll, defineMatrix, output, pruned } = equations();
    const parameters = [{ id: id(), name: 'Closure rate', symbol: 'rate', value: round(closureRate(timeStep)), unit: '1/s', mode: 'constant' }];
    for (const created of parameters) bindings.push({ symbol: created.symbol, kind: 'parameter', parameterId: created.id });
    const loopAccelerationSymbols = sensitivitySymbols('la', R);
    const loopBiasSymbols = sensitivitySymbols('lp', R);
    const ends = loop.ends.map((frame, index) => {
        const node = nodes.get(frame.solid);
        const role = index === 0 ? 'source' : 'target';
        const prefix = index === 0 ? 'p' : 'c';
        const onPath = path.has(frame.solid);
        const rotation = rotationSymbols.map((row) => row.map((symbol) => bind(node, role, symbol, prefix)));
        const centre = ['x', 'y', 'z'].map((symbol) => bind(node, role, symbol, prefix));
        const velocity = velocitySymbols.map((symbol) => bind(node, role, symbol, prefix));
        const acceleration = accelerationSymbols.map((symbol) => bind(node, role, symbol, prefix));
        const sensitivity = onPath ? loopAccelerationSymbols.map((row) => row.map((symbol) => bind(node, role, symbol, prefix))) : null;
        const tag = index === 0 ? 'e' : 'f';
        const point = defineAll(`${tag}p`, E.matrixVector(rotation, frame.point.map(round)).map((value, axis) => E.add(centre[axis], value)));
        return { node, role, prefix, onPath, rotation, velocity, acceleration, sensitivity, point, sign: index === 0 ? 1 : -1, solid: frame.solid };
    });
    const [first, second] = ends;
    // The first solid's frame axes, in the world, and its point.
    const axisLocal = loop.ends[0].axis;
    const frameAxes = closureFrame(normalize(axisLocal)).map((axis, index) => defineAll(`ax${index + 1}`, E.matrixVector(first.rotation, axis.map(round))));
    const p = first.point;
    const relative = (of) => [0, 1, 2, 3, 4, 5].map((index) => E.subtract(of(second)[index], of(first)[index]));
    const dv = defineAll('dv', relative((end) => end.velocity));
    const da = defineAll('dq', relative((end) => end.acceleration));
    const omega = first.velocity.slice(0, 3);
    const pointRate = defineAll('prt', first.velocity.slice(3).map((value, axis) => E.add(value, E.cross(omega, p)[axis])));
    // The second solid's turn from where the drawing has it relative to the first: Q = R₂ (R₁ R₀)ᵀ, and its small-angle
    // vector ½ vee(Q - Qᵀ).
    const angular = loop.rows.some(([type]) => type === 'angular');
    let twist = null;
    if (angular) {
        const firstPose = poseOf(drawing, loop.ends[0].solid);
        const secondPose = poseOf(drawing, loop.ends[1].solid);
        const rest = rotationMatrix(multiplyQuaternions(conjugate(firstPose.rotation), secondPose.rotation)).map((row) => row.map(round));
        const wanted = defineMatrix('rw', E.matrixMatrix(first.rotation, rest));
        const Q = [0, 1, 2].map((i) => [0, 1, 2].map((j) => E.dot(second.rotation[i], wanted[j])));
        twist = defineAll('tw', [E.multiply(0.5, E.subtract(Q[2][1], Q[1][2])), E.multiply(0.5, E.subtract(Q[0][2], Q[2][0])), E.multiply(0.5, E.subtract(Q[1][0], Q[0][1]))]);
    }
    const linearAt = (vector) => [0, 1, 2].map((axis) => E.add(vector[3 + axis], E.cross(vector.slice(0, 3), p)[axis]));
    const relativeAtPoint = defineAll('dvp', linearAt(dv));
    const gap = defineAll('gp', [0, 1, 2].map((axis) => E.subtract(second.point[axis], p[axis])));
    // Each row: its G (6), its rate c, Ġ Δ, G (â₂ - â₁) and its gap or twist e.
    const rows = loop.rows.map(([type, axisIndex], index) => {
        const a = frameAxes[axisIndex];
        const spin = E.cross(omega, a);
        const tag = `r${index + 1}`;
        if (type === 'angular') {
            return {
                type, a,
                G: [...a, 0, 0, 0],
                rate: define(`${tag}c`, E.dot(a, dv.slice(0, 3))),
                drift: define(`${tag}g`, E.dot(spin, dv.slice(0, 3))),
                accel: define(`${tag}a`, E.dot(a, da.slice(0, 3))),
                error: define(`${tag}e`, E.dot(a, twist))
            };
        }
        return {
            type, a,
            G: [...E.cross(p, a), ...a],
            rate: define(`${tag}c`, E.dot(a, relativeAtPoint)),
            drift: define(`${tag}g`, E.add(E.dot(spin, relativeAtPoint), E.dot(a, E.cross(dv.slice(0, 3), pointRate)))),
            accel: define(`${tag}a`, E.dot(a, linearAt(da))),
            error: define(`${tag}e`, E.dot(a, gap))
        };
    }).map((row, k) => ({ ...row, G: row.G.map((value, index) => define(`g${k + 1}x${index + 1}`, value)) }));
    // Each end's own share of how its bias force moves with this loop's forces: +Gᵀ on the first, -Gᵀ on the second.
    for (const end of ends) {
        if (!end.onPath) continue;
        rows.forEach((row, k) => loopBiasSymbols.forEach((line, r) => output(end.node, end.role, line[loop.offset + k], E.multiply(end.sign, row.G[r]))));
    }
    // This loop's rows of K and of the wanted acceleration, to its holder.
    const holder = ends.find((end) => end.solid === loop.holderId);
    const coupling = couplingSymbols(R);
    rows.forEach((row, k) => {
        for (let column = 0; column <= loop.offset + k; column += 1) {
            const moved = E.add(ends.map((end) => (end.onPath ? E.multiply(-end.sign, E.dot(row.G, end.sensitivity.map((line) => line[column]))) : 0)));
            output(holder.node, holder.role, coupling[loop.offset + k][column], moved);
        }
        output(holder.node, holder.role, wantedSymbols(R)[loop.offset + k], E.negate(E.add(row.drift, E.multiply(2, 'rate', row.rate), E.multiply('rate', 'rate', row.error), row.accel)));
    });
    // Its forces, as the Ground found them, back from its holder: each end's bias force changes by ±Gᵀ λ.
    const forces = rows.map((_, k) => bind(holder.node, holder.role, forceSymbols(R)[loop.offset + k], holder.prefix));
    for (const end of ends) {
        if (!end.onPath || mode === 'rows') continue;
        // With joint rows, as the force on the solid in the second sweep.
        (mode === 'second' ? externalSymbols : biasChangeSymbols).forEach((symbol, index) => output(end.node, end.role, symbol, E.multiply(end.sign, E.add(rows.map((row, k) => E.multiply(row.G[index], forces[k]))))));
    }
    // What the second solid is pushed by: the linear rows' force at the point, the angular rows' moment; and the gap
    // and twist left.
    const along = (type) => [0, 1, 2].map((axis) => E.add(rows.map((row, k) => (row.type === type ? E.multiply(row.a[axis], forces[k]) : 0))));
    const report = holder === second ? 1 : -1;
    along('linear').forEach((value, axis) => output(holder.node, holder.role, `${loop.key}Force${axis + 1}`, E.multiply(report, value)));
    if (angular) along('angular').forEach((value, axis) => output(holder.node, holder.role, `${loop.key}Moment${axis + 1}`, E.multiply(report, value)));
    const sizeOf = (type) => E.sqrt(E.add(rows.map((row) => (row.type === type ? E.multiply(row.error, row.error) : 0))));
    output(holder.node, holder.role, `${loop.key}Gap`, sizeOf('linear'));
    if (angular) output(holder.node, holder.role, `${loop.key}Twist`, sizeOf('angular'));
    return {
        id: id(), name: loop.joint.name, directionality: 'directed',
        source: { nodeId: first.node.id }, target: { nodeId: second.node.id },
        parameters,
        equation: '',
        equationModel: { bindings, definitions: pruned(), outputs },
        appearance: { color: '#e0b04f', offset: 0, waypoints: [] }
    };
}

// A coupling: a relationship from its second joint's solid (the Ground when it has none) to its first's, holding the
// first joint's position to a polynomial of the second's, q₁ = c₀ + c₁ q₂ + c₂ q₂² + c₃ q₂³ + c₄ q₂⁴, as MuJoCo's joint
// equality holds it: a row of generalized force, unbounded, its r = q₁ - p(q₂) and v = q̇₁ - p'(q₂) q̇₂, its compliance
// (1 - imp) w / imp (w the two joints' inverse inertias where drawn, summed) and its reference acceleration
// -B v - K imp r, handed to the first joint's solid, where they are gathered with the joints' shares of the row.
function couplingRelationship(coupling, otherNode, firstNode, firstJoint, secondJoint, index, w, R, id, timeStep) {
    const { bindings, outputs, bind, define, output, pruned } = equations();
    const softness = { solref: coupling.softness?.solref ?? defaultSoftness.solref, solimp: coupling.softness?.solimp ?? defaultSoftness.solimp };
    const { B, K } = rowReference(softness, timeStep);
    const parameters = [
        ...coupling.polycoef.map((value, power) => ({ name: `Coefficient ${power}`, symbol: `c${power}`, value })),
        { name: 'Row damping', symbol: 'equalityDamping', value: B, unit: '1/s' },
        { name: 'Row stiffness', symbol: 'equalityStiffness', value: K, unit: '1/s²' },
        { name: 'Inverse inertia where drawn', symbol: 'inverseWeight', value: w }
    ].map((parameter) => ({ id: id(), unit: '', mode: 'constant', ...parameter, value: round(parameter.value) }));
    for (const created of parameters) bindings.push({ symbol: created.symbol, kind: 'parameter', parameterId: created.id });
    const coordinate = (joint) => (joint.kind === 'slider' ? 'jointPosition' : 'jointAngle');
    const q1 = bind(firstNode, 'target', coordinate(firstJoint), 'c');
    const v1 = bind(firstNode, 'target', 'jointSpeed', 'c');
    const q2 = secondJoint ? bind(otherNode, 'source', coordinate(secondJoint), 'p') : 0;
    const v2 = secondJoint ? bind(otherNode, 'source', 'jointSpeed', 'p') : 0;
    const p = E.add('c0', E.multiply('c1', q2), E.multiply('c2', q2, q2), E.multiply('c3', q2, q2, q2), E.multiply('c4', q2, q2, q2, q2));
    const slope = E.add('c1', E.multiply(2, 'c2', q2), E.multiply(3, 'c3', q2, q2), E.multiply(4, 'c4', q2, q2, q2));
    const r = define('er', E.subtract(q1, p));
    const v = define('ev', E.subtract(v1, E.multiply(slope, v2)));
    const imp = define('eim', impedanceAt(softness.solimp, r));
    const tag = index + 1;
    output(firstNode, 'target', wantedSymbols(R)[index], E.subtract(E.multiply(-1, 'equalityDamping', v), E.multiply('equalityStiffness', imp, r)));
    output(firstNode, 'target', `kc${tag}`, E.divide(E.multiply(E.subtract(1, imp), 'inverseWeight'), imp));
    output(firstNode, 'target', `kn${tag}`, -1e30);
    output(firstNode, 'target', `kx${tag}`, 1e30);
    return {
        id: id(), name: coupling.name, directionality: 'directed',
        source: { nodeId: otherNode.id }, target: { nodeId: firstNode.id },
        parameters, equation: '',
        equationModel: { bindings, definitions: pruned(), outputs },
        appearance: { color: '#b48ead', offset: 0, waypoints: [] }
    };
}

// A tendon: a relationship from each of its joints' solids to its node. Each adds its joint's share to the tendon's
// length (offset + Σ c q) and speed (Σ c q̇), and pulls its joint by c F, F the force the tendon pulls with; and, with a
// limit, gives its joint the row's J there, c times the side it is past. The first works out F: its spring
// -k (L - L₀), its damping -b L̇ and its drive (a force, or a servo kp (setpoint - L) - kv L̇ held to its most force);
// and hands its solid its rows' reference accelerations, compliances (w = cᵀ M⁻¹ c where drawn) and bounds, as a
// joint's are, along the tendon: its dry friction, its limit and, with damping rows, its damping h (b + kv) (kv while
// the servo is within its most force).
function tendonRelationships(tendon, parts, tendonNode, rows, w, R, id, timeStep, symbols, document) {
    const soft = softnessOf(tendon);
    const drive = tendon.drive ?? { mode: 'free' };
    return parts.map((part, index) => {
        const { bindings, outputs, bind, define, output, pruned } = equations();
        const parameters = [];
        const parameter = (symbol, name, value, unit = '', extra = {}) => {
            const created = { id: id(), name, symbol, value: round(value), unit, mode: 'constant', ...extra };
            parameters.push(created);
            bindings.push({ symbol, kind: 'parameter', parameterId: created.id });
            return created;
        };
        parameter('coef', 'Share of the length', part.coef);
        const coordinate = part.joint.kind === 'slider' ? 'jointPosition' : 'jointAngle';
        const q = bind(part.node, 'source', coordinate, 'p');
        const qd = bind(part.node, 'source', 'jointSpeed', 'p');
        const toTendon = (symbol, expression) => output(tendonNode, 'target', symbol, expression);
        const toSolid = (symbol, expression) => output(part.node, 'source', symbol, expression);
        if (index === 0) parameter('offset', 'Length where drawn', tendon.offset);
        toTendon('length', E.add(E.multiply('coef', q), index === 0 ? 'offset' : 0));
        toTendon('speed', E.multiply('coef', qd));
        if (index > 0) {
            toSolid('tendonForce', E.multiply('coef', bind(tendonNode, 'target', 'force', 't')));
            if (tendon.limits) toSolid(`rj${rows.limit + 1}`, E.multiply('coef', bind(tendonNode, 'target', 'limitSide', 't')));
            return edge();
        }
        const L = bind(tendonNode, 'target', 'length', 't');
        const Ld = bind(tendonNode, 'target', 'speed', 't');
        parameter('stiffness', 'Stiffness', tendon.stiffness);
        parameter('springLength', 'Spring length', tendon.springLength);
        // A dead band: no pull between its two ends (the same length without one).
        parameter('springLengthUpper', 'Spring length, dead band\'s upper end', tendon.springLengthUpper ?? tendon.springLength);
        parameter('damping', 'Damping', tendon.damping);
        parameter('step', 'Time step', timeStep, 's');
        let pull = 0;
        let servoStep = 0;
        if (drive.mode === 'torque') {
            const reach = Math.max(2, Math.abs(drive.torque ?? 0) * 2);
            parameter('torque', 'Force', drive.torque ?? 0, '', { mode: 'live', control: { minimum: round(-reach), maximum: round(reach), step: round(reach / 100) } });
            pull = 'torque';
        }
        if (drive.mode === 'servo') {
            // Its setpoint a live shared parameter, so a fork can change it from a time on.
            const reach = Math.max(1, Math.abs(drive.setpoint ?? 0) * 2);
            const shared = { id: id(), name: `${tendon.name} setpoint`, symbol: camelKey(`${tendon.name} setpoint`, symbols), value: round(drive.setpoint ?? 0), unit: '', mode: 'live', control: liveRange(-reach, reach) };
            document.sharedParameters.push(shared);
            parameter('setpoint', 'Setpoint', shared.value, '', { mode: 'live', control: shared.control, sharedParameterId: shared.id });
            parameter('gain', 'Servo stiffness', drive.gain);
            parameter('servoDamping', 'Servo damping', drive.damping ?? 0);
            parameter('torqueLimit', 'Servo force limit', drive.torqueLimit);
            const raw = define('sraw', E.subtract(E.multiply('gain', E.subtract('setpoint', L)), E.multiply('servoDamping', Ld)));
            pull = define('servo', E.max(E.negate('torqueLimit'), E.min('torqueLimit', raw)));
            servoStep = E.cases([[E.greater(raw, 'torqueLimit'), 0], [E.less(raw, E.negate('torqueLimit')), 0]], E.multiply('servoDamping', 'step'));
        }
        const stretch = E.subtract(L, E.max('springLength', E.min('springLengthUpper', L)));
        const force = define('tf', E.add(E.multiply(-1, 'stiffness', stretch), E.multiply(-1, 'damping', Ld), pull));
        toTendon('force', force);
        toSolid('tendonForce', E.multiply('coef', force));
        if (rows.friction !== undefined || rows.limit !== undefined) parameter('inverseWeight', 'Inverse inertia where drawn', w);
        if (rows.friction !== undefined) {
            const { B } = rowReference(soft.friction, timeStep);
            const imp = impedanceAt(soft.friction.solimp, 0);
            parameter('friction', 'Friction', tendon.friction);
            parameter('frictionDamping', 'Friction row damping', B, '1/s');
            parameter('frictionImpedance', 'Friction row impedance', imp);
            const tag = rows.friction + 1;
            toSolid(wantedSymbols(R)[rows.friction], E.multiply(-1, 'frictionDamping', Ld));
            toSolid(`kc${tag}`, E.divide(E.multiply(E.subtract(1, 'frictionImpedance'), 'inverseWeight'), 'frictionImpedance'));
            toSolid(`kn${tag}`, E.negate('friction'));
            toSolid(`kx${tag}`, 'friction');
        }
        if (rows.limit !== undefined) {
            const { B, K } = rowReference(soft.limit, timeStep);
            parameter('lower', 'Shortest', tendon.limits.lower);
            parameter('upper', 'Longest', tendon.limits.upper);
            parameter('margin', 'Limit margin', soft.margin);
            parameter('limitDamping', 'Limit row damping', B, '1/s');
            parameter('limitStiffness', 'Limit row stiffness', K, '1/s²');
            const lower = define('qlr', E.subtract(E.subtract(L, 'lower'), 'margin'));
            const upper = define('qur', E.subtract(E.subtract('upper', L), 'margin'));
            const past = define('qpr', E.cases([[E.less(lower, 0), lower], [E.less(upper, 0), upper]], 0));
            const side = define('qj', E.cases([[E.less(lower, 0), 1], [E.less(upper, 0), -1]], 1));
            const imp = define('qim', impedanceAt(soft.limit.solimp, past));
            const tag = rows.limit + 1;
            toTendon('limitSide', side);
            toSolid(`rj${tag}`, E.multiply('coef', side));
            toSolid(wantedSymbols(R)[rows.limit], E.subtract(E.multiply(-1, 'limitDamping', side, Ld), E.multiply('limitStiffness', imp, past)));
            toSolid(`kc${tag}`, E.divide(E.multiply(E.subtract(1, imp), 'inverseWeight'), imp));
            toSolid(`kn${tag}`, 0);
            toSolid(`kx${tag}`, E.cases([[E.less(lower, 0), 1e30], [E.less(upper, 0), 1e30]], 0));
        }
        // Each of its damping rows' h b times its weight (lib/modelBuilder.mjs's dampingRows).
        for (const { index: row, weight } of rows.damping ?? []) toSolid(`kc${row + 1}`, E.multiply(weight, E.add(E.multiply('damping', 'step'), servoStep)));
        return edge();

        function edge() {
            return {
                id: id(), name: `${tendon.name} over ${part.joint.name}`, directionality: 'directed',
                source: { nodeId: part.node.id }, target: { nodeId: tendonNode.id },
                parameters, equation: '',
                equationModel: { bindings, definitions: pruned(), outputs },
                appearance: { color: '#c58b4f', offset: 0, waypoints: [] }
            };
        }
    });
}

// A joint's motor (motor mode): a node of its own, holding its supply voltage, duty cycle, current, torque at the joint
// and winding temperature; and the robot's battery, a node holding its voltage and charge. Each motor's relationships:
// - Supply, from the battery: the motor's supply voltage is the battery's.
// - Controller, from the joint's solid: a duty cycle of the proportional gain times the setpoint left to reach, less the
//   derivative gain times the joint's speed, held to ±1. The setpoint is a live slider, or the keyframes' schedule. A
//   user's own controller may set the duty instead, wired in the canvas with these gains at nothing.
// - Windings, between the joint's solid and the motor: the current, (duty × supply voltage - back-EMF) over the
//   winding's resistance, the back-EMF the torque constant times the motor's speed (the gear ratio times the joint's);
//   the torque at the joint, gear ratio × torque constant × current; and that torque, on the joint's solid, where the
//   joint's own relationship reads it. The winding's inductance is left out: its time constant is far below a step.
// - Drain, to the battery: its charge falls by the current the drive draws from it (duty × current) over its capacity.
// The motor's winding heats with its resistance times the current squared and cools to ambient (a source term of its
// node), and the battery's voltage falls in a straight line from full to empty with its charge (a source term of its).
function motorsAndBattery(drawing, motorDriven, nodes, document, setpoints, symbols, id) {
    const battery = batteryOf(drawing);
    // A shared parameter; `range` makes it a live slider over it, which a fork can change.
    const share = (name, value, unit, range = null) => {
        const created = { id: id(), name, symbol: camelKey(name, symbols), value: round(value), unit, mode: range ? 'live' : 'constant', ...(range ? { control: liveRange(...range) } : {}) };
        document.sharedParameters.push(created);
        return created;
    };
    const parameterIndex = [];
    const nodeOf = (name, type, position, color, shape = 'box') => ({
        id: id(), name, type, position, states: [], sourceTerms: [], bySymbol: new Map(), numerics: { substepsPerGlobalStep: 1, integration: 'semiImplicit' },
        appearance: { type: 'primitive', shape, color }
    });
    const addState = (node, symbol, initialValue, unit) => {
        const created = { id: id(), name: stateName(symbol), symbol, initialValue: round(initialValue), unit };
        node.states.push(created);
        node.bySymbol.set(symbol, created);
        return created;
    };
    // A source term of `node` setting or growing `state` by `latex`/`expression`, reading its own states and `parameters`.
    const sourceTerm = (node, state, expression, parameters, setsValue) => {
        const latex = E.toLatex(expression);
        const bound = parameters.map((parameter) => ({ ...parameter, id: id() }));
        const term = {
            id: id(), state: state.symbol, expression: latex, parameters: bound,
            expressionModel: {
                latex, mathJson: expression, output: { stateId: state.id },
                bindings: [...E.symbolsOf(expression)].map((symbol) => {
                    const parameter = bound.find((candidate) => candidate.symbol === symbol);
                    return parameter ? { symbol, kind: 'parameter', parameterId: parameter.id } : { symbol, kind: 'state', stateId: node.bySymbol.get(symbol).id };
                })
            }
        };
        if (setsValue) term.setsValue = true;
        node.sourceTerms.push(term);
    };
    const linked = (shared, symbol) => ({ name: shared.name, symbol, value: shared.value, unit: shared.unit, mode: 'constant', sharedParameterId: shared.id });

    // Its voltages are live, so a fork can tire the battery: the same charge giving fewer volts.
    const full = share('Battery full voltage', battery.fullVoltage, 'V', [0, 2 * battery.fullVoltage]);
    const empty = share('Battery empty voltage', battery.emptyVoltage, 'V', [0, 2 * battery.fullVoltage]);
    parameterIndex.push(indexEntry('batteryFullVoltage', 'Battery', full), indexEntry('batteryEmptyVoltage', 'Battery', empty));
    const capacity = share('Battery capacity', battery.capacity, 'A·s');
    const store = nodeOf('Battery', 'robotBattery', [0, 4, 0], '#2f6970');
    const voltage = addState(store, 'voltage', battery.fullVoltage, 'V');
    addState(store, 'stateOfCharge', 100, '%');
    sourceTerm(store, voltage, E.add('empty', E.multiply(E.subtract('full', 'empty'), 0.01, 'stateOfCharge')), [linked(full, 'full'), linked(empty, 'empty')], true);
    // A supply with an internal resistance or a most current sags under load. The motors' currents are each linear in
    // the voltage at their terminals V (a motor at duty d with back-EMF e draws d (d V - e) / R from it), so the supply
    // gives G V - J, G the sum of d² / R and J the sum of d e / R over the motors, each motor's load relationship adding
    // its share to the battery's `loadConductance` and `loadCurrent` (values several relationships set add up, as currents
    // at a junction). Through an internal resistance Rs: V = (open voltage + Rs J) / (1 + Rs G). Held to a most current
    // I: when G V - J would be more, V = (I + J) / G. Each supply relationship works V out for its motor.
    const sags = battery.internalResistance > 0 || battery.currentLimit > 0;
    if (sags) {
        addState(store, 'loadConductance', 0, 'S');
        addState(store, 'loadCurrent', 0, 'A');
    }
    document.nodes.push(store);

    for (const [solidId, joint] of motorDriven) {
        const solidNode = nodes.get(solidId);
        const drive = { ...defaultDrive(joint.kind), ...joint.drive, motor: { ...defaultDrive(joint.kind).motor, ...joint.drive.motor } };
        const { motor } = drive;
        const linear = joint.kind === 'slider';
        const motorNode = nodeOf(`${joint.name} motor`, 'robotMotor', [solidNode.position[0] + 2.5, solidNode.position[1] - 1.8, 0], '#397b86', 'cylinder');
        for (const [symbol, initial, unit] of [['supplyVoltage', battery.fullVoltage, 'V'], ['duty', 0, ''], ['current', 0, 'A'], ['torque', 0, linear ? 'N' : 'N·m'], ['windingTemperature', motor.ambientTemperature, 'K'], ...(sags ? [['backEmf', 0, 'V']] : [])]) {
            addState(motorNode, symbol, initial, unit);
        }
        const resistance = share(`${joint.name} winding resistance`, motor.resistance, 'Ω');
        // Live, so a fork can weaken the motor (a worn or a smaller one): less torque for its current, and less back-EMF.
        const torqueConstant = share(`${joint.name} torque constant`, motor.torqueConstant, 'N·m/A', [motor.torqueConstant / 10, 2 * motor.torqueConstant]);
        parameterIndex.push(indexEntry('torqueConstant', joint.name, torqueConstant));
        const gearRatio = share(`${joint.name} gear ratio`, motor.gearRatio, linear ? 'rad/m' : '');
        sourceTerm(motorNode, motorNode.bySymbol.get('windingTemperature'),
            E.divide(E.subtract(E.multiply('resistance', 'current', 'current'), E.divide(E.subtract('windingTemperature', 'ambient'), 'thermalResistance')), 'heatCapacity'), [
                linked(resistance, 'resistance'),
                { name: 'Thermal resistance to ambient', symbol: 'thermalResistance', value: round(motor.thermalResistance), unit: 'K/W', mode: 'constant' },
                { name: 'Winding heat capacity', symbol: 'heatCapacity', value: round(motor.thermalCapacitance), unit: 'J/K', mode: 'constant' },
                { name: 'Ambient temperature', symbol: 'ambient', value: round(motor.ambientTemperature), unit: 'K', mode: 'constant' }
            ], false);
        document.nodes.push(motorNode);

        // Supply: the battery's voltage, on the motor, less what it sags under every motor's load.
        const supplyParameters = sags ? [
            { id: id(), name: 'Internal resistance', symbol: 'internalResistance', value: round(battery.internalResistance), unit: 'Ω', mode: 'constant' },
            ...(battery.currentLimit > 0 ? [{ id: id(), name: 'Most current', symbol: 'currentLimit', value: round(battery.currentLimit), unit: 'A', mode: 'constant' }] : [])
        ] : [];
        document.edges.push(algebraicEdge(`${joint.name} supply`, store, motorNode, id, supplyParameters, (bind, output, define) => {
            const open = bind(store, 'source', 'voltage', 'p');
            if (!sags) { output(motorNode, 'target', 'supplyVoltage', open); return; }
            const conductance = bind(store, 'source', 'loadConductance', 'p');
            const load = bind(store, 'source', 'loadCurrent', 'p');
            const throughResistance = define('throughResistance', E.divide(E.add(open, E.multiply('internalResistance', load)), E.add(1, E.multiply('internalResistance', conductance))));
            const limited = battery.currentLimit > 0
                ? E.cases([[E.greater(E.subtract(E.multiply(conductance, throughResistance), load), 'currentLimit'), E.divide(E.add('currentLimit', load), conductance)]], throughResistance)
                : throughResistance;
            output(motorNode, 'target', 'supplyVoltage', limited);
        }));
        // Controller: the duty cycle, from the setpoint left to reach and the joint's speed.
        const travel = joint.limits ?? (joint.kind === 'hinge' ? { lower: -Math.PI, upper: Math.PI } : { lower: -0.5, upper: 0.5 });
        const scheduled = setpoints.get(joint.id);
        const setpoint = Math.min(Math.max(drive.setpoint ?? 0, travel.lower), travel.upper);
        const controllerParameters = [
            { id: id(), name: 'Setpoint', symbol: 'setpoint', value: round(scheduled?.value ?? setpoint), unit: linear ? 'm' : 'rad', mode: 'live',
                control: scheduled?.control ?? { minimum: round(travel.lower), maximum: round(travel.upper), step: round((travel.upper - travel.lower) / 400) },
                ...(scheduled ? { sharedParameterId: scheduled.id } : {}) },
            { id: id(), name: 'Proportional gain', symbol: 'proportional', value: round(drive.proportional), unit: linear ? '1/m' : '1/rad', mode: 'constant' },
            { id: id(), name: 'Derivative gain', symbol: 'derivative', value: round(drive.derivative), unit: linear ? 's/m' : 's/rad', mode: 'constant' },
            ...(sags ? [{ id: id(), ...linked(torqueConstant, 'torqueConstant') }, { id: id(), ...linked(gearRatio, 'gearRatio') }] : [])
        ];
        const position = linear ? 'jointPosition' : 'jointAngle';
        document.edges.push(algebraicEdge(`${joint.name} controller`, solidNode, motorNode, id, controllerParameters, (bind, output) => {
            const q = bind(solidNode, 'source', position, 'p');
            const qd = bind(solidNode, 'source', 'jointSpeed', 'p');
            output(motorNode, 'target', 'duty', E.max(-1, E.min(1, E.subtract(E.multiply('proportional', E.subtract('setpoint', q)), E.multiply('derivative', qd)))));
            if (sags) output(motorNode, 'target', 'backEmf', E.multiply('torqueConstant', 'gearRatio', qd));
        }));
        if (sags) {
            // Load: the motor's share of what the supply gives, from its duty and back-EMF.
            const loadParameters = [{ id: id(), ...linked(resistance, 'resistance') }];
            document.edges.push(algebraicEdge(`${joint.name} load`, motorNode, store, id, loadParameters, (bind, output) => {
                const duty = bind(motorNode, 'source', 'duty', 'p');
                const backEmf = bind(motorNode, 'source', 'backEmf', 'p');
                output(store, 'target', 'loadConductance', E.divide(E.multiply(duty, duty), 'resistance'));
                output(store, 'target', 'loadCurrent', E.divide(E.multiply(duty, backEmf), 'resistance'));
            }));
        }
        // Windings: the current, and the torque it makes at the joint.
        const windingParameters = [
            { id: id(), ...linked(resistance, 'resistance') },
            { id: id(), ...linked(torqueConstant, 'torqueConstant') },
            { id: id(), ...linked(gearRatio, 'gearRatio') },
            { id: id(), name: 'Gear efficiency', symbol: 'gearEfficiency', value: round(motor.gearEfficiency), unit: '', mode: 'constant' }
        ];
        document.edges.push(algebraicEdge(`${joint.name} windings`, solidNode, motorNode, id, windingParameters, (bind, output, define) => {
            const qd = bind(solidNode, 'source', 'jointSpeed', 'p');
            const duty = bind(motorNode, 'target', 'duty', 'c');
            const supply = bind(motorNode, 'target', 'supplyVoltage', 'c');
            const current = define('amps', E.divide(E.subtract(E.multiply(duty, supply), E.multiply('torqueConstant', 'gearRatio', qd)), 'resistance'));
            const torque = define('tq', E.multiply('gearEfficiency', 'gearRatio', 'torqueConstant', current));
            output(motorNode, 'target', 'current', current);
            output(motorNode, 'target', 'torque', torque);
            output(solidNode, 'source', 'motorTorque', torque);
        }));
        // Drain: the charge the drive draws.
        const drainParameters = [{ id: id(), ...linked(capacity, 'capacity') }];
        const duty = motorNode.bySymbol.get('duty');
        const current = motorNode.bySymbol.get('current');
        const charge = store.bySymbol.get('stateOfCharge');
        const expression = E.negate(E.divide(E.multiply(100, 'sourceDuty', 'sourceCurrent'), 'capacity'));
        const latex = E.toLatex(expression);
        document.edges.push({
            id: id(), name: `${joint.name} drain`, directionality: 'directed',
            source: { nodeId: motorNode.id, stateId: current.id }, target: { nodeId: store.id, stateId: charge.id },
            parameters: drainParameters, equation: latex,
            equationModel: {
                latex, mathJson: expression, output: { role: 'target', stateId: charge.id },
                bindings: [
                    { kind: 'state', role: 'source', nodeId: motorNode.id, stateId: duty.id, symbol: 'sourceDuty' },
                    { kind: 'state', role: 'source', nodeId: motorNode.id, stateId: current.id, symbol: 'sourceCurrent' },
                    { kind: 'parameter', parameterId: drainParameters[0].id, symbol: 'capacity' }
                ]
            },
            appearance: { color: '#c46a6a', offset: 0, waypoints: [] }
        });
    }
    return parameterIndex;
}

// A relationship of several equations between `source` and `target`, written by `write(bind, output, define)`.
function algebraicEdge(name, source, target, id, parameters, write) {
    const { bindings, outputs, bind, define, output, pruned } = equations();
    for (const parameter of parameters) bindings.push({ symbol: parameter.symbol, kind: 'parameter', parameterId: parameter.id });
    write(bind, output, define);
    return {
        id: id(), name, directionality: 'directed', source: { nodeId: source.id }, target: { nodeId: target.id }, parameters, equation: '',
        equationModel: { bindings, definitions: pruned(), outputs }, appearance: { color: '#68a9cc', offset: 0, waypoints: [] }
    };
}
