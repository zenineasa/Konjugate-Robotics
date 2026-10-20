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

import { checkDrawing, frameById, frameInWorld, groundId, massOf, poseOf, structure, treeOrder } from './drawing.mjs';
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
    jointTorque: 'Drive torque', x: 'x', y: 'y', z: 'z', roll: 'Roll', pitch: 'Pitch', yaw: 'Yaw',
    w1: 'Angular velocity x', w2: 'Angular velocity y', w3: 'Angular velocity z',
    v1: 'Velocity at origin x', v2: 'Velocity at origin y', v3: 'Velocity at origin z',
    jointSpeed1: 'Joint angular velocity x', jointSpeed2: 'Joint angular velocity y', jointSpeed3: 'Joint angular velocity z',
    jointTurnW: 'Joint turn w', jointTurnX: 'Joint turn x', jointTurnY: 'Joint turn y', jointTurnZ: 'Joint turn z',
    turnW: 'Turn w', turnX: 'Turn x', turnY: 'Turn y', turnZ: 'Turn z',
    closureForce1: 'Closure force x', closureForce2: 'Closure force y', closureForce3: 'Closure force z', closureGap: 'Closure gap',
    kineticEnergy: 'Kinetic energy', potentialEnergy: 'Potential energy'
};
const stateName = (symbol) => stateNames[symbol]
    ?? (/^jointAcceleration\d$/.test(symbol) ? `Joint acceleration ${symbol.slice(-1)}`
        : symbol.startsWith('ia') ? `Articulated inertia ${symbol.slice(2)}` : symbol.startsWith('pa') ? `Articulated bias force ${symbol.slice(2)}`
            : symbol.startsWith('lp') ? `Loop bias sensitivity ${symbol.slice(2)}` : symbol.startsWith('la') ? `Loop acceleration sensitivity ${symbol.slice(2)}`
                : symbol.startsWith('dp') ? `Loop bias change ${symbol.slice(2)}` : symbol.startsWith('da') ? `Loop acceleration change ${symbol.slice(2)}`
                    : symbol.startsWith('a') ? `Spatial acceleration ${symbol.slice(1)}` : symbol.startsWith('r') ? `Rotation ${symbol.slice(1)}` : symbol);

// The loop's sensitivities: a 6 by 3 matrix each (how a solid's bias force, or its acceleration, moves with the force
// that closes the loop), and the changes that force makes.
const sensitivitySymbols = (prefix) => [1, 2, 3, 4, 5, 6].map((row) => [1, 2, 3].map((column) => `${prefix}${row}${column}`));
const loopBiasSymbols = sensitivitySymbols('lp');
const loopAccelerationSymbols = sensitivitySymbols('la');
const biasChangeSymbols = ['dp1', 'dp2', 'dp3', 'dp4', 'dp5', 'dp6'];
const accelerationChangeSymbols = ['da1', 'da2', 'da3', 'da4', 'da5', 'da6'];
const turnSymbols = { ball: ['jointTurnW', 'jointTurnX', 'jointTurnY', 'jointTurnZ'], free: ['turnW', 'turnX', 'turnY', 'turnZ'] };
const degreesOfFreedom = { hinge: 1, slider: 1, ball: 3, free: 6, fixed: 0 };

// The reasons a drawing cannot be simulated yet by this builder (beyond checkDrawing's), each { id, message }.
export function simulationLimits(drawing) {
    const built = structure(drawing);
    const limits = [];
    const [first, ...others] = built.loops;
    if (first?.kind === 'fixed') limits.push({ id: first.id, message: `${first.name} closes a loop as a fixed joint, which cannot be simulated yet; make it a hinge or a ball, or remove another joint of the loop.` });
    for (const joint of others) {
        limits.push({ id: joint.id, message: `${joint.name} closes a second loop; one loop can be simulated so far. Remove it or another joint of its loop.` });
    }
    return limits;
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
export function buildModel(drawing, { timeStep = 0.001, runTime = 10, outputInterval = 0.01 } = {}) {
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

    // ---- the loop, if the drawing closes one ---------------------------------------------------------------------------
    // The solids from each of its two ends back towards the Ground (`path`) carry its sensitivities; every solid below
    // one of them (`affected`) moves with the force that closes it.
    const [closing] = built.loops;
    const loop = closing ? { joint: closing, ends: [closing.parent, closing.child].map((frameId) => frameById(drawing, frameId)) } : null;
    const path = new Set();
    const affected = new Set();
    if (loop) {
        for (const { solid } of loop.ends) for (let at = solid; at !== groundId; at = parentOf.get(at).parent) path.add(at);
        for (const { id: solidId } of order) if (path.has(solidId) || affected.has(parentOf.get(solidId).parent)) affected.add(solidId);
    }

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
                type: 'primitive', shape: solid.shape === 'capsule' ? 'cylinder' : solid.shape, color: solid.color,
                parts: [{ prefix: '', mesh: shapeMesh(solid.shape, solid.size), color: solid.color }]
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
            // Joined to nothing: its own spatial velocity, centre and rotation are what it integrates.
            velocityStates(node);
            ['x', 'y', 'z'].forEach((symbol, axis) => addState(node, symbol, pose.position[axis], 'm'));
            const turn = turnSymbols.free.map((symbol, index) => addState(node, symbol, pose.rotation[index]));
            const accelerations = [1, 2, 3, 4, 5, 6].map((axis) => addState(node, `jointAcceleration${axis}`, 0, axis <= 3 ? 'rad/s²' : 'm/s²'));
            velocitySymbols.forEach((symbol, index) => integrate(node, node.bySymbol.get(symbol), accelerations[index].symbol));
            // The centre moves as the velocity at the origin plus ω × the centre.
            const spin = E.cross(['w1', 'w2', 'w3'], ['x', 'y', 'z']);
            ['x', 'y', 'z'].forEach((symbol, axis) => integrate(node, node.bySymbol.get(symbol), E.add(`v${axis + 1}`, spin[axis])));
            quaternionRates(turnSymbols.free, ['w1', 'w2', 'w3']).forEach((rate, index) => integrate(node, turn[index], rate));
            rotationStates(node, pose, { position: false });
        } else {
            rotationStates(node, pose);
            velocityStates(node);
        }
        accelerationSymbols.forEach((symbol) => addState(node, symbol, 0));
        symmetric.forEach(([row, column]) => addState(node, inertiaSymbol(row, column), 0));
        biasSymbols.forEach((symbol) => addState(node, symbol, 0));
        if (path.has(solidId)) {
            loopBiasSymbols.flat().forEach((symbol) => addState(node, symbol, 0));
            loopAccelerationSymbols.flat().forEach((symbol) => addState(node, symbol, 0));
            biasChangeSymbols.forEach((symbol) => addState(node, symbol, 0));
        }
        if (affected.has(solidId)) accelerationChangeSymbols.forEach((symbol) => addState(node, symbol, 0));
        addState(node, 'kineticEnergy', 0, 'J');
        addState(node, 'potentialEnergy', 0, 'J');
        nodes.set(solid.id, node);
    }
    if (loop) {
        // The force that closes the loop, and the gap it closes, on the solid at its second end (or its first, when the
        // second is the Ground).
        const holder = nodes.get(loop.ends[1].solid === groundId ? loop.ends[0].solid : loop.ends[1].solid);
        [1, 2, 3].forEach((axis) => addState(holder, `closureForce${axis}`, 0, 'N'));
        addState(holder, 'closureGap', 0, 'm');
        loop.holder = holder;
    }
    document.nodes.push(...nodes.values());

    // ---- each solid's mass, a shared parameter -------------------------------------------------------------------------
    const symbols = new Set();
    const masses = new Map();
    for (const { id: solidId } of order) {
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const shared = { id: id(), name: `${solid.name} mass`, symbol: camelKey(`${solid.name} mass`, symbols), value: round(massOf(solid)), unit: 'kg', mode: 'constant' };
        document.sharedParameters.push(shared);
        masses.set(solidId, shared);
    }

    // ---- each joint, a relationship --------------------------------------------------------------------------------------
    for (const { id: childId } of order) {
        const { joint, parent: parentId } = parentOf.get(childId);
        const roles = { onPath: path.has(childId), affected: affected.has(childId), parentAffected: affected.has(parentId) };
        document.edges.push(jointRelationship(drawing, joint, nodes.get(parentId), nodes.get(childId), parentId, childId, masses.get(childId), id, roles));
    }
    if (loop) document.edges.push(closingRelationship(drawing, loop, nodes, path, id, timeStep));
    for (const node of document.nodes) delete node.bySymbol;

    const runConfigurationId = id();
    document.runConfigurations = [{ id: runConfigurationId, name: `${runTime} s`, targetTime: runTime, globalTimeStep: timeStep, outputInterval }];
    document.activeRunConfigurationId = runConfigurationId;
    return {
        document,
        data: {
            solids: drawing.solids.length,
            joints: drawing.joints.length,
            driven: drawing.joints.filter((joint) => joint.drive?.mode && joint.drive.mode !== 'free').length,
            nodes: document.nodes.length,
            edges: document.edges.length,
            equations: document.edges.reduce((sum, edge) => sum + edge.equationModel.outputs.length, 0)
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
    const unit = unitInertia(child.shape, child.size).map(round);

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
    const drive = (hinge || slider) ? joint.drive ?? { mode: 'free' } : { mode: 'free' };
    const range = (value, floor, minimum = null) => {
        const reach = Math.max(floor, Math.abs(value) * 2);
        return { minimum: round(Math.min(minimum ?? -reach, value)), maximum: round(Math.max(reach, value)), step: round(reach / 100) };
    };
    if (moving && !free) {
        parameter('damping', 'Damping', joint.damping ?? 0, units.damping, range(joint.damping ?? 0, slider ? 50 : 0.5, 0));
        parameter('friction', 'Friction', joint.friction ?? 0, units.friction, range(joint.friction ?? 0, slider ? 5 : 0.5, 0));
        parameter('armature', 'Armature', joint.armature ?? 0, units.armature);
        if (joint.limits && !ball) {
            parameter('lower', 'Lower limit', joint.limits.lower, units.position);
            parameter('upper', 'Upper limit', joint.limits.upper, units.position);
        }
        if (drive.mode === 'torque') parameter('torque', slider ? 'Force' : 'Torque', drive.torque ?? 0, units.effort, range(drive.torque ?? 0, hinge ? 2 : 50));
        if (drive.mode === 'servo') {
            const travel = joint.limits ?? (hinge ? { lower: -Math.PI, upper: Math.PI } : { lower: -0.5, upper: 0.5 });
            const setpoint = Math.min(Math.max(drive.setpoint ?? 0, travel.lower), travel.upper);
            parameter('setpoint', 'Setpoint', setpoint, units.position, { minimum: round(travel.lower), maximum: round(travel.upper), step: round((travel.upper - travel.lower) / 400) });
            parameter('stiffness', 'Servo stiffness', drive.gain, units.stiffness);
            const limit = drive.torqueLimit;
            parameter('torqueLimit', slider ? 'Servo force limit' : 'Servo torque limit', limit, units.effort, { minimum: round(limit / 100), maximum: round(limit * 3), step: round(limit / 100) });
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
        speeds = velocitySymbols.map((symbol) => bind(childNode, 'target', symbol, 'c'));
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
        motions = [0, 1, 2, 3, 4, 5].map((column) => [0, 1, 2, 3, 4, 5].map((row) => (row === column ? 1 : 0)));
    }
    // The joint's own velocity, S q̇, and the child's: its parent's plus the joint's.
    const jointVelocity = [0, 1, 2, 3, 4, 5].map((row) => E.add(motions.map((motion, k) => E.multiply(motion[row], speeds[k]))));
    const velocity = free ? speeds : defineAll('vel', parentVelocity.map((value, index) => E.add(value, jointVelocity[index])));
    if (!free) velocitySymbols.forEach((symbol, index) => toChild(symbol, velocity[index]));

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

    // The loop's states this relationship reads and passes on.
    const loopBias = roles.onPath ? loopBiasSymbols.map((row) => row.map((symbol) => bind(childNode, 'target', symbol, 'c'))) : null;
    const biasChange = roles.onPath ? biasChangeSymbols.map((symbol) => bind(childNode, 'target', symbol, 'c')) : [0, 0, 0, 0, 0, 0];
    const parentOnPath = roles.onPath && !parentIsGround;
    const parentLoopAcceleration = parentOnPath ? loopAccelerationSymbols.map((row) => row.map((symbol) => bind(parentNode, 'source', symbol, 'p'))) : null;
    const parentAccelerationChange = roles.parentAffected ? accelerationChangeSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p')) : [0, 0, 0, 0, 0, 0];
    const columnOf = (matrix, column) => matrix.map((row) => row[column]);

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
                biasChangeSymbols.forEach((symbol, index) => toParent(symbol, biasChange[index]));
            }
            loopAccelerationSymbols.forEach((row, r) => row.forEach((symbol, column) => toChild(symbol, parentOnPath ? parentLoopAcceleration[r][column] : 0)));
        }
        if (roles.affected) accelerationChangeSymbols.forEach((symbol, index) => toChild(symbol, parentAccelerationChange[index]));
        return relationshipEdge();
    }

    // U = IA S, and D = Sᵀ U plus the armature on each axis.
    const projected = motions.map((motion, k) => defineAll(dof === 1 ? 'u' : `u${k + 1}r`, [1, 2, 3, 4, 5, 6].map((row) => E.dot(articulatedRow(row), motion))));
    const D = motions.map((motion, i) => projected.map((column, j) => {
        const entry = E.dot(motion, column);
        return i === j && !free ? E.add(entry, 'armature') : entry;
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
        if (drive.mode === 'servo') {
            driveTorque = define('servo', E.max(E.negate('torqueLimit'), E.min('torqueLimit', E.multiply('stiffness', E.subtract('setpoint', q)))));
        }
        const effective = 'd';
        const limitTorque = joint.limits ? E.cases([
            [E.greater(q, 'upper'), E.negate(E.multiply(effective, E.add(E.multiply(10000, E.subtract(q, 'upper')), E.multiply(200, qd))))],
            [E.less(q, 'lower'), E.negate(E.multiply(effective, E.add(E.multiply(10000, E.subtract(q, 'lower')), E.multiply(200, qd))))]
        ], 0) : 0;
        forces = [define('tau', E.add(driveTorque, E.negate(E.multiply('damping', qd)), E.negate(E.multiply('friction', E.tanh(E.multiply(100, qd)))), limitTorque))];
        toChild('jointTorque', driveTorque);
    } else if (ball) {
        forces = speeds.map((speed, k) => define(`tau${k + 1}`, E.add(E.negate(E.multiply('damping', speed)), E.negate(E.multiply('friction', E.tanh(E.multiply(100, speed)))))));
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
    const bias = defineAll('c', motionCross);

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

    // The loop, where this joint is on its way or below it: how the child's bias force and acceleration move with the
    // force that closes it (P inward: P + (I - U D⁻¹ Sᵀ) P; A outward: A - S D⁻¹ (Sᵀ P + Uᵀ A)), and, once that force
    // is known, the change it makes (inward, then outward), added to the joint's acceleration.
    let accelerationChange = [0];
    if (roles.onPath) {
        const passed = [0, 1, 2].map((column) => solve(motionTimes(columnOf(loopBias, column)), `pp${column + 1}`));
        if (parentOnPath) {
            loopBiasSymbols.forEach((row, r) => row.forEach((symbol, column) => toParent(symbol,
                E.subtract(loopBias[r][column], projectedAlong(passed[column])[r]))));
            const passedChange = solve(motionTimes(biasChange), 'pc');
            biasChangeSymbols.forEach((symbol, index) => toParent(symbol, E.subtract(biasChange[index], projectedAlong(passedChange)[index])));
        }
        [0, 1, 2].forEach((column) => {
            const parentColumn = parentOnPath ? columnOf(parentLoopAcceleration, column) : [0, 0, 0, 0, 0, 0];
            const moved = solve(motionTimes(columnOf(loopBias, column)).map((value, k) => E.add(value, E.dot(projected[k], parentColumn))), `pa${column + 1}`);
            loopAccelerationSymbols.forEach((row, r) => toChild(row[column], E.subtract(parentColumn[r], along(moved)[r])));
        });
    }
    if (roles.affected) {
        accelerationChange = solve(motionTimes(biasChange).map((value, k) => E.negate(E.add(value, E.dot(projected[k], parentAccelerationChange)))), 'dq');
        accelerationChangeSymbols.forEach((symbol, index) => toChild(symbol, E.add(parentAccelerationChange[index], along(accelerationChange)[index])));
    }
    const total = jointAcceleration.map((value, k) => (roles.affected ? E.add(value, accelerationChange[k]) : value));
    if (dof === 1) toChild('jointAcceleration', total[0]);
    else total.forEach((value, k) => toChild(`jointAcceleration${k + 1}`, value));
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

// The joint that closes the loop: a relationship between its two solids (either may be the Ground) holding its two
// frames' points together. It gives each end's bias force its share of the closing force (-Lᵀ on the first end, +Lᵀ on
// the second, where L takes a solid's spatial acceleration to its point's), reads back how each end's acceleration moves
// with that force (K = L₁ A₁ - L₂ A₂), solves (K + εI) λ = -2 ω ė - ω² e - (p̈₁ - p̈₂) for it, and hands each end the
// change it makes to its bias force.
function closingRelationship(drawing, loop, nodes, path, id, timeStep) {
    const { bindings, outputs, bind, define, defineAll, output, pruned } = equations();
    const parameters = [
        { id: id(), name: 'Closure rate', symbol: 'rate', value: round(closureRate(timeStep)), unit: '1/s', mode: 'constant' },
        { id: id(), name: 'Closure compliance', symbol: 'compliance', value: closure.compliance, unit: 'm/(s²·N)', mode: 'constant' }
    ];
    for (const created of parameters) bindings.push({ symbol: created.symbol, kind: 'parameter', parameterId: created.id });
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
        // The point, how fast it moves (v + ω × p) and how fast that changes (a + α × p + ω × its velocity).
        const point = defineAll(`${tag}p`, E.matrixVector(rotation, frame.point.map(round)).map((value, axis) => E.add(centre[axis], value)));
        const pointVelocity = defineAll(`${tag}v`, velocity.slice(3).map((value, axis) => E.add(value, E.cross(velocity.slice(0, 3), point)[axis])));
        const pointAcceleration = defineAll(`${tag}a`, acceleration.slice(3).map((value, axis) => E.add(value, E.cross(acceleration.slice(0, 3), point)[axis], E.cross(velocity.slice(0, 3), pointVelocity)[axis])));
        return { node, role, onPath, point, pointVelocity, pointAcceleration, sensitivity, sign: index === 0 ? -1 : 1 };
    });
    // L's rows for a point p: [-skew(p) | I], so L â = a + α × p.
    const skew = (p) => [[0, E.negate(p[2]), p[1]], [p[2], 0, E.negate(p[0])], [E.negate(p[1]), p[0], 0]];
    const L = (p, row) => [...skew(p)[row].map((value) => E.negate(value)), ...[0, 1, 2].map((axis) => (axis === row ? 1 : 0))];
    // Each end's own share of how its bias force moves with λ: ∓ Lᵀ = ∓ [skew(p); I].
    for (const end of ends) {
        if (!end.onPath) continue;
        loopBiasSymbols.forEach((row, r) => row.forEach((symbol, column) => {
            const entry = r < 3 ? skew(end.point)[r][column] : (r - 3 === column ? 1 : 0);
            output(end.node, end.role, symbol, E.multiply(end.sign, entry));
        }));
    }
    const K = [0, 1, 2].map((row) => [0, 1, 2].map((column) => define(`k${row + 1}${column + 1}`, E.add(ends.map((end) => (end.onPath
        ? E.multiply(-end.sign, E.dot(L(end.point, row), end.sensitivity.map((line) => line[column])))
        : 0)), row === column ? 'compliance' : 0))));
    const gap = defineAll('gap', [0, 1, 2].map((axis) => E.subtract(ends[0].point[axis], ends[1].point[axis])));
    const rate = defineAll('gr', [0, 1, 2].map((axis) => E.subtract(ends[0].pointVelocity[axis], ends[1].pointVelocity[axis])));
    const wanted = defineAll('want', [0, 1, 2].map((axis) => E.subtract(
        E.negate(E.add(E.multiply(2, 'rate', rate[axis]), E.multiply('rate', 'rate', gap[axis]))),
        E.subtract(ends[0].pointAcceleration[axis], ends[1].pointAcceleration[axis]))));
    // λ by Cramer's rule.
    const determinant = define('kdet', E.add(
        E.multiply(K[0][0], E.subtract(E.multiply(K[1][1], K[2][2]), E.multiply(K[1][2], K[2][1]))),
        E.negate(E.multiply(K[0][1], E.subtract(E.multiply(K[1][0], K[2][2]), E.multiply(K[1][2], K[2][0])))),
        E.multiply(K[0][2], E.subtract(E.multiply(K[1][0], K[2][1]), E.multiply(K[1][1], K[2][0])))));
    const replaced = (column) => K.map((row, r) => row.map((value, c) => (c === column ? wanted[r] : value)));
    const det3 = (m) => E.add(
        E.multiply(m[0][0], E.subtract(E.multiply(m[1][1], m[2][2]), E.multiply(m[1][2], m[2][1]))),
        E.negate(E.multiply(m[0][1], E.subtract(E.multiply(m[1][0], m[2][2]), E.multiply(m[1][2], m[2][0])))),
        E.multiply(m[0][2], E.subtract(E.multiply(m[1][0], m[2][1]), E.multiply(m[1][1], m[2][0]))));
    const force = defineAll('lam', [0, 1, 2].map((column) => E.divide(det3(replaced(column)), determinant)));
    // Each end's bias force changes by ∓ Lᵀ λ = ∓ [p × λ; λ].
    for (const end of ends) {
        if (!end.onPath) continue;
        const moment = E.cross(end.point, force);
        biasChangeSymbols.forEach((symbol, index) => output(end.node, end.role, symbol, E.multiply(end.sign, index < 3 ? moment[index] : force[index - 3])));
    }
    const holder = ends.find((end) => end.node === loop.holder);
    [1, 2, 3].forEach((axis) => output(holder.node, holder.role, `closureForce${axis}`, force[axis - 1]));
    output(holder.node, holder.role, 'closureGap', E.sqrt(E.dot(gap, gap)));
    const [first, second] = ends;
    return {
        id: id(), name: loop.joint.name, directionality: 'directed',
        source: { nodeId: first.node.id }, target: { nodeId: second.node.id },
        parameters,
        equation: '',
        equationModel: { bindings, definitions: pruned(), outputs },
        appearance: { color: '#e0b04f', offset: 0, waypoints: [] }
    };
}
