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
    kineticEnergy: 'Kinetic energy', potentialEnergy: 'Potential energy'
};
const stateName = (symbol) => stateNames[symbol]
    ?? (symbol.startsWith('ia') ? `Articulated inertia ${symbol.slice(2)}` : symbol.startsWith('pa') ? `Articulated bias force ${symbol.slice(2)}`
        : symbol.startsWith('a') ? `Spatial acceleration ${symbol.slice(1)}` : symbol.startsWith('r') ? `Rotation ${symbol.slice(1)}` : symbol);

// The reasons a drawing cannot be simulated yet by this builder (beyond checkDrawing's), each { id, message }.
export function simulationLimits(drawing) {
    const built = structure(drawing);
    const limits = [];
    for (const joint of drawing.joints) {
        if (joint.kind === 'ball') limits.push({ id: joint.id, message: `${joint.name}: ball joints cannot be simulated yet; make it a hinge, a slider or fixed.` });
    }
    for (const joint of built.loops) limits.push({ id: joint.id, message: `${joint.name} closes a loop, which cannot be simulated yet; remove it or another joint of the loop.` });
    for (const root of built.roots) {
        const solid = drawing.solids.find(({ id }) => id === root);
        limits.push({ id: root, message: `${solid.name} is not joined to the Ground; a solid that floats freely cannot be simulated yet. Join it, or fix it to the Ground.` });
    }
    return limits;
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

    // ---- nodes ---------------------------------------------------------------------------------------------------
    const nodes = new Map();
    const addState = (node, symbol, initialValue, unit = '') => {
        const created = { id: id(), name: stateName(symbol), symbol, initialValue: round(initialValue), unit };
        node.states.push(created);
        node.bySymbol.set(symbol, created);
        return created;
    };
    const poseStates = (node, pose) => {
        const matrix = rotationMatrix(pose.rotation);
        const [roll, pitch, yaw] = toRollPitchYaw(pose.rotation);
        ['x', 'y', 'z'].forEach((symbol, axis) => addState(node, symbol, pose.position[axis], 'm'));
        addState(node, 'roll', roll, 'rad');
        addState(node, 'pitch', pitch, 'rad');
        addState(node, 'yaw', yaw, 'rad');
        rotationSymbols.forEach((row, rowIndex) => row.forEach((symbol, column) => addState(node, symbol, matrix[rowIndex][column])));
    };

    const ground = { id: id(), name: 'Ground', type: 'robotGround', position: [0, 0, 0], states: [], sourceTerms: [], bySymbol: new Map(),
        appearance: { type: 'primitive', shape: 'box', color: '#7b8794' } };
    poseStates(ground, poseOf(drawing, groundId));
    velocitySymbols.forEach((symbol) => addState(ground, symbol, 0, symbol.startsWith('w') ? 'rad/s' : 'm/s'));
    // Holding still under gravity is, in these terms, accelerating upward at g.
    accelerationSymbols.forEach((symbol, index) => addState(ground, symbol, index === 5 ? gravity : 0, index < 3 ? 'rad/s²' : 'm/s²'));
    nodes.set(groundId, ground);

    const parentOf = new Map([...built.parentJoint].map(([child, link]) => [child, link]));
    const rows = new Map();
    for (const { id: solidId, depth } of order) {
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const joint = parentOf.get(solidId).joint;
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
        const moving = joint.kind === 'hinge' || joint.kind === 'slider';
        if (moving) {
            const linear = joint.kind === 'slider';
            const speed = addState(node, 'jointSpeed', 0, linear ? 'm/s' : 'rad/s');
            const position = addState(node, linear ? 'jointPosition' : 'jointAngle', 0, linear ? 'm' : 'rad');
            const acceleration = addState(node, 'jointAcceleration', 0, linear ? 'm/s²' : 'rad/s²');
            addState(node, 'jointTorque', 0, linear ? 'N' : 'N·m');
            // The speed from the acceleration, then the angle from the new speed.
            const term = (state, from) => {
                const latex = `\\mathrm{${from.symbol}}`;
                node.sourceTerms.push({
                    id: id(), state: state.symbol, expression: latex, parameters: [],
                    expressionModel: { latex, mathJson: from.symbol, bindings: [{ symbol: from.symbol, kind: 'state', stateId: from.id }], output: { stateId: state.id } }
                });
            };
            term(speed, acceleration);
            term(position, speed);
        }
        poseStates(node, poseOf(drawing, solid.id));
        velocitySymbols.forEach((symbol) => addState(node, symbol, 0, symbol.startsWith('w') ? 'rad/s' : 'm/s'));
        accelerationSymbols.forEach((symbol) => addState(node, symbol, 0));
        symmetric.forEach(([row, column]) => addState(node, inertiaSymbol(row, column), 0));
        biasSymbols.forEach((symbol) => addState(node, symbol, 0));
        addState(node, 'kineticEnergy', 0, 'J');
        addState(node, 'potentialEnergy', 0, 'J');
        nodes.set(solid.id, node);
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
        document.edges.push(jointRelationship(drawing, joint, nodes.get(parentId), nodes.get(childId), parentId, childId, masses.get(childId), id));
    }
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

// One joint's relationship: from the solid nearer the Ground (`parentNode`) to the one it carries (`childNode`).
function jointRelationship(drawing, joint, parentNode, childNode, parentId, childId, massParameter, id) {
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const kind = joint.kind;
    const hinge = kind === 'hinge';
    const slider = kind === 'slider';
    const moving = hinge || slider;
    const parentIsGround = parentId === groundId;
    const child = drawing.solids.find(({ id: solidId }) => solidId === childId);

    // ---- the joint's geometry, in the parent's frame where the drawing has them, as numbers ------------------------------
    const parentPose = poseOf(drawing, parentId);
    const childPose = poseOf(drawing, childId);
    const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
    // A joint read the other way round (its first frame on the solid further from the Ground) turns the other way.
    const flipped = solidOf.get(joint.child) !== childId;
    const a = toLocal(parentPose, anchor.point).map(round);
    const n = normalize(scale(directionToLocal(parentPose, anchor.axis), flipped ? -1 : 1)).map(round);
    const restRotation = rotationMatrix(multiplyQuaternions(conjugate(parentPose.rotation), childPose.rotation)).map((row) => row.map(round));
    const restOffset = toLocal(parentPose, childPose.position).map(round);
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
    const drive = joint.drive ?? { mode: 'free' };
    const range = (value, floor, minimum = null) => {
        const reach = Math.max(floor, Math.abs(value) * 2);
        return { minimum: round(Math.min(minimum ?? -reach, value)), maximum: round(Math.max(reach, value)), step: round(reach / 100) };
    };
    if (moving) {
        parameter('damping', 'Damping', joint.damping ?? 0, units.damping, range(joint.damping ?? 0, hinge ? 0.5 : 50, 0));
        parameter('friction', 'Friction', joint.friction ?? 0, units.friction, range(joint.friction ?? 0, hinge ? 0.5 : 5, 0));
        parameter('armature', 'Armature', joint.armature ?? 0, units.armature);
        if (joint.limits) {
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
    const bindings = [];
    const bind = (node, role, symbol, prefix) => {
        const state = node.bySymbol.get(symbol);
        if (!state) throw new Error(`${node.name} has no state ${symbol}.`);
        const bound = `${prefix}${symbol[0].toUpperCase()}${symbol.slice(1)}`;
        bindings.push({ symbol: bound, kind: 'state', role, nodeId: node.id, stateId: state.id });
        return bound;
    };
    for (const created of parameters) bindings.push({ symbol: created.symbol, kind: 'parameter', parameterId: created.id });
    const parentRotation = rotationSymbols.map((row) => row.map((symbol) => bind(parentNode, 'source', symbol, 'p')));
    const parentPosition = ['x', 'y', 'z'].map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    const parentVelocity = velocitySymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    const parentAcceleration = accelerationSymbols.map((symbol) => bind(parentNode, 'source', symbol, 'p'));
    const q = moving ? bind(childNode, 'target', slider ? 'jointPosition' : 'jointAngle', 'c') : null;
    const qd = moving ? bind(childNode, 'target', 'jointSpeed', 'c') : null;
    const articulated = (row, column) => `c${inertiaSymbol(row, column)[0].toUpperCase()}${inertiaSymbol(row, column).slice(1)}`;
    symmetric.forEach(([row, column]) => bind(childNode, 'target', inertiaSymbol(row, column), 'c'));
    const childBias = biasSymbols.map((symbol) => bind(childNode, 'target', symbol, 'c'));
    const IA = (row, column) => articulated(row, column);

    // ---- definitions and equations ----------------------------------------------------------------------------------------------
    const definitions = [];
    const define = (symbol, expression) => {
        if (typeof expression === 'number' || typeof expression === 'string') return expression;
        definitions.push({ symbol, mathJson: expression, latex: E.toLatex(expression) });
        return symbol;
    };
    const defineAll = (prefix, values) => values.map((value, index) => define(`${prefix}${index + 1}`, value));
    const defineMatrix = (prefix, matrix) => matrix.map((row, rowIndex) => row.map((value, column) => define(`${prefix}${rowIndex + 1}${column + 1}`, value)));
    const outputs = [];
    const output = (node, role, symbol, expression) => {
        const state = node.bySymbol.get(symbol);
        outputs.push({ role, stateId: state.id, mathJson: expression, latex: E.toLatex(expression), setsValue: true });
    };
    const toChild = (symbol, expression) => output(childNode, 'target', symbol, expression);
    const toParent = (symbol, expression) => output(parentNode, 'source', symbol, expression);

    // The child's place: rotation and position relative to the parent, then in the world.
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
    } else if (slider) {
        relativeOffset = defineAll('off', restOffset.map((value, axis) => E.add(value, E.multiply(n[axis], q))));
    }
    const rotation = defineMatrix('rot', E.matrixMatrix(parentRotation, relativeRotation));
    const position = defineAll('pos', E.matrixVector(parentRotation, relativeOffset).map((value, axis) => E.add(parentPosition[axis], value)));
    rotationSymbols.forEach((row, rowIndex) => row.forEach((symbol, column) => toChild(symbol, rotation[rowIndex][column])));
    ['x', 'y', 'z'].forEach((symbol, axis) => toChild(symbol, position[axis]));
    toChild('roll', E.atan2(rotation[2][1], rotation[2][2]));
    toChild('pitch', E.asin(E.min(1, E.max(-1, E.negate(rotation[2][0])))));
    toChild('yaw', E.atan2(rotation[1][0], rotation[0][0]));

    // The joint's motion as a spatial vector S: a hinge turns about its axis through its point, a slider moves along it.
    let motion = null;
    if (moving) {
        const axis = defineAll('axis', E.matrixVector(parentRotation, n));
        if (hinge) {
            const point = defineAll('jp', E.matrixVector(parentRotation, a).map((value, index) => E.add(parentPosition[index], value)));
            motion = [...axis, ...defineAll('jm', E.cross(point, axis))];
        } else {
            motion = [0, 0, 0, ...axis];
        }
    }
    // The child's velocity: its parent's, plus the joint's.
    const velocity = defineAll('vel', parentVelocity.map((value, index) => (motion ? E.add(value, E.multiply(motion[index], qd)) : value)));
    velocitySymbols.forEach((symbol, index) => toChild(symbol, velocity[index]));

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

    // The articulated inertia and bias force the child has (its own and what its children pass up), as bound.
    const articulatedRow = (row) => [1, 2, 3, 4, 5, 6].map((column) => IA(row, column));
    if (!moving) {
        // Fixed: the child moves with its parent, so it passes everything up, and accelerates as its parent does.
        if (!parentIsGround) {
            symmetric.forEach(([row, column]) => toParent(inertiaSymbol(row, column), IA(row, column)));
            biasSymbols.forEach((symbol, index) => toParent(symbol, childBias[index]));
        }
        accelerationSymbols.forEach((symbol, index) => toChild(symbol, parentAcceleration[index]));
        return relationshipEdge();
    }

    // The joint's torque: its drive, its damping, its friction (smoothed over a hundredth of a radian per second) and its
    // limits (a stiff spring and damper beyond them, scaled by the joint's effective inertia).
    const projected = defineAll('u', [1, 2, 3, 4, 5, 6].map((row) => E.dot(articulatedRow(row), motion)));
    const effective = define('d', E.add(E.dot(motion, projected), 'armature'));
    let driveTorque = 0;
    if (drive.mode === 'torque') driveTorque = 'torque';
    if (drive.mode === 'servo') {
        driveTorque = define('servo', E.max(E.negate('torqueLimit'), E.min('torqueLimit', E.multiply('stiffness', E.subtract('setpoint', q)))));
    }
    const limitTorque = joint.limits ? E.cases([
        [E.greater(q, 'upper'), E.negate(E.multiply(effective, E.add(E.multiply(10000, E.subtract(q, 'upper')), E.multiply(200, qd))))],
        [E.less(q, 'lower'), E.negate(E.multiply(effective, E.add(E.multiply(10000, E.subtract(q, 'lower')), E.multiply(200, qd))))]
    ], 0) : 0;
    const torque = define('tau', E.add(driveTorque, E.negate(E.multiply('damping', qd)), E.negate(E.multiply('friction', E.tanh(E.multiply(100, qd)))), limitTorque));
    toChild('jointTorque', driveTorque);
    const remaining = define('ur', E.subtract(torque, E.dot(motion, childBias)));
    // The velocity-product acceleration: the child's velocity cross the joint's.
    const jointVelocity = motion.map((value) => E.multiply(value, qd));
    const motionCross = [
        ...E.cross(angular, jointVelocity.slice(0, 3)),
        ...E.cross(angular, jointVelocity.slice(3)).map((value, index) => E.add(value, E.cross(linear, jointVelocity.slice(0, 3))[index]))
    ];
    const bias = defineAll('c', motionCross);

    // Inward: what the child passes to its parent (the Ground takes nothing): the articulated inertia less what the
    // joint lets go (IA - U U'/D), and the bias force plus that reduced inertia times c plus U u/D.
    if (!parentIsGround) {
        symmetric.forEach(([row, column]) => toParent(inertiaSymbol(row, column),
            E.subtract(IA(row, column), E.divide(E.multiply(projected[row - 1], projected[column - 1]), effective))));
        const biasAlong = define('uc', E.dot(projected, bias));
        biasSymbols.forEach((symbol, index) => toParent(symbol, E.add(childBias[index], E.dot(articulatedRow(index + 1), bias),
            E.divide(E.multiply(projected[index], E.subtract(remaining, biasAlong)), effective))));
    }

    // Outward again: the joint's acceleration and the child's.
    const carried = defineAll('ap', parentAcceleration.map((value, index) => E.add(value, bias[index])));
    const jointAcceleration = define('qdd', E.divide(E.subtract(remaining, E.dot(projected, carried)), effective));
    toChild('jointAcceleration', jointAcceleration);
    accelerationSymbols.forEach((symbol, index) => toChild(symbol, E.add(carried[index], E.multiply(motion[index], jointAcceleration))));
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

    // Only the definitions an equation needs, directly or through others.
    function pruned() {
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
    }
}
