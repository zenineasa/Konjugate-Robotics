/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A drawing: what the Robotics window holds, kept with the project, and what Build turns into MJCF and into a
// Konjugate model. Plain JSON, versioned, with no dependencies, so it is tested without a browser and read by scripts.
//
// {
//   format: 'konjugate.robotics.drawing', version: 1, name, nextId,
//   solids: [{ id, name, shape, size, position, rotation, color, mass: { by: 'density', density } | { by: 'mass', value } }],
//   frames: [{ id, solid (a solid's id, or 'ground'), point, axis }],
//   joints: [{ id, name, kind, parent, child, limits, damping, friction, armature, drive }]
// }
//
// Positions and frame points are metres, z up; a solid's `rotation` is a unit quaternion [w, x, y, z]. A frame's point
// and axis are in its solid's own frame (the Ground's frame is the world), so a frame moves with its solid. A joint
// joins two frames, `parent` and `child`, by their ids: its anchor is the parent frame's point and its axis the parent
// frame's axis, both where the drawing has them. `kind` is hinge (turns about the axis), slider (slides along it), ball
// or fixed. `limits` is { lower, upper } (radians for a hinge, metres for a slider; for a ball, lower 0 and upper its
// most turn from where it is drawn) or null. `drive` says what moves a hinge, a slider or a ball: { mode: 'free' },
// { mode: 'torque', torque } (newton metres, or newtons for a slider; for a ball [x, y, z] about its parent's axes) or
// { mode: 'servo', setpoint, gain, torqueLimit } (a position servo: its setpoint, for a ball a turn vector [x, y, z] in
// its parent's frame, its length the angle; its stiffness in N·m per radian; the most torque it gives).
//
// Every operation returns a new drawing and leaves the one given alone, so the window's undo keeps each as it was.

import { defaultSize, shapeKinds, sizeProblems, volume } from './shapes.mjs';
import {
    add, between, conjugate, directionToWorld, identity, length, multiply, normalize, normalizeQuaternion, rotate, scale, subtract, toWorld
} from './math.mjs';

export const drawingFormat = 'konjugate.robotics.drawing';
export const drawingVersion = 1;
export const groundId = 'ground';

export const jointKinds = Object.freeze(['hinge', 'slider', 'ball', 'fixed']);
export const jointLabels = Object.freeze({ hinge: 'Hinge', slider: 'Slider', ball: 'Ball', fixed: 'Fixed' });
export const driveModes = Object.freeze(['free', 'torque', 'servo', 'motor']);
export const driveLabels = Object.freeze({ free: 'Free', torque: 'Torque', servo: 'Servo', motor: 'Motor' });

// A joint's DC motor and gearbox (motor mode), as a hobby 12 V gearmotor: its winding's resistance (ohms) and torque
// constant (N·m per ampere, its back-EMF volts per rad/s too), the gear ratio (motor turns per joint turn; for a slider,
// motor radians per metre, as a lead screw's), its rotor's inertia, and its winding's heating: thermal resistance to
// ambient (K/W) and heat capacity (J/K).
export function defaultMotor(kind) {
    return {
        resistance: 2.5, torqueConstant: 0.02, gearRatio: kind === 'slider' ? 3142 : 100, rotorInertia: 1e-6,
        thermalResistance: 10, thermalCapacitance: 15, ambientTemperature: 293.15, windingLimit: 403.15
    };
}
// `windingLimit`: the hottest its winding may get (130 °C, a class B winding's, by default). Not part of the model: the
// sizing view holds the run's winding temperature against it.
export const motorFields = Object.freeze(['resistance', 'torqueConstant', 'gearRatio', 'rotorInertia', 'thermalResistance', 'thermalCapacitance', 'ambientTemperature', 'windingLimit']);

// The robot's battery, which every motor draws from: its voltage full and empty (falling in a straight line with its
// charge), and its capacity in ampere seconds (a 3-cell LiPo of 2.2 Ah by default).
export const defaultBattery = Object.freeze({ fullVoltage: 12.6, emptyVoltage: 9.9, capacity: 7920 });
export const batteryOf = (drawing) => ({ ...defaultBattery, ...(drawing.battery ?? {}) });
const motorFieldNames = {
    resistance: 'winding resistance', torqueConstant: 'torque constant', gearRatio: 'gear ratio', rotorInertia: 'rotor inertia',
    thermalResistance: 'thermal resistance', thermalCapacitance: 'heat capacity', ambientTemperature: 'ambient temperature',
    windingLimit: 'hottest winding'
};

// Densities a solid may be made of, kilograms per cubic metre, from common tables; "Other" is typed.
export const materials = Object.freeze([
    { id: 'pla', name: 'PLA (3D printed, solid)', density: 1240 },
    { id: 'abs', name: 'ABS', density: 1050 },
    { id: 'aluminium', name: 'Aluminium', density: 2700 },
    { id: 'steel', name: 'Steel', density: 7850 },
    { id: 'wood', name: 'Wood (pine)', density: 500 },
    { id: 'water', name: 'Water', density: 1000 }
]);

// Colours a new solid takes in turn.
const palette = ['#5b8db8', '#d08b4a', '#6aa86a', '#b86a9e', '#c9b44a', '#5ab3b0', '#a07ac8', '#c86a6a'];

export function createDrawing(name = 'Robot') {
    return { format: drawingFormat, version: drawingVersion, name, nextId: 1, solids: [], frames: [], joints: [], keyframes: [] };
}

// A drawing read from a session, checked for its shape, or an error saying what is wrong.
export function readDrawing(value) {
    if (!value || value.format !== drawingFormat) throw new Error('This is not a Robotics drawing.');
    if (value.version !== drawingVersion) throw new Error(`This drawing is of version ${value.version}; this toolbox reads version ${drawingVersion}.`);
    for (const field of ['solids', 'frames', 'joints']) if (!Array.isArray(value[field])) throw new Error(`The drawing has no list of ${field}.`);
    if (value.keyframes !== undefined && !Array.isArray(value.keyframes)) throw new Error('The drawing\'s keyframes are not a list.');
    return structuredClone(value);
}

function take(drawing, prefix) {
    const id = `${prefix}${drawing.nextId}`;
    drawing.nextId += 1;
    return id;
}

// A name not yet used by a solid or joint: `base`, or `base 2`, `base 3` and so on.
export function uniqueName(drawing, base) {
    const taken = new Set([...drawing.solids, ...drawing.joints].map(({ name }) => name.toLowerCase()));
    if (!taken.has(base.toLowerCase())) return base;
    for (let suffix = 2; ; suffix += 1) if (!taken.has(`${base} ${suffix}`.toLowerCase())) return `${base} ${suffix}`;
}

export const solidById = (drawing, id) => drawing.solids.find((solid) => solid.id === id) ?? null;
export const frameById = (drawing, id) => drawing.frames.find((frame) => frame.id === id) ?? null;
export const jointById = (drawing, id) => drawing.joints.find((joint) => joint.id === id) ?? null;

// A solid's pose, or the Ground's (the world itself).
export function poseOf(drawing, solidId) {
    if (solidId === groundId) return { position: [0, 0, 0], rotation: [...identity] };
    const solid = solidById(drawing, solidId);
    if (!solid) throw new Error(`There is no solid ${solidId}.`);
    return { position: solid.position, rotation: solid.rotation };
}

// A frame's point and axis in the world, where the drawing has its solid.
export function frameInWorld(drawing, frame) {
    const pose = poseOf(drawing, frame.solid);
    return { point: toWorld(pose, frame.point), axis: normalize(directionToWorld(pose, frame.axis)) };
}

export function massOf(solid) {
    if (solid.mass?.by === 'mass') return solid.mass.value;
    return (solid.mass?.density ?? 0) * volume(solid.shape, solid.size);
}

// ---- solids -----------------------------------------------------------------------------------------------------

// A new solid of `shape`, resting on the floor at `at` (x, y) unless a position is given. Returns { drawing, id }.
export function addSolid(drawing, shape, { position, name, size, color } = {}) {
    if (!shapeKinds.includes(shape)) throw new Error(`There is no shape "${shape}".`);
    const next = structuredClone(drawing);
    const id = take(next, 's');
    const chosen = size ?? defaultSize(shape);
    const lift = shape === 'box' ? chosen.z / 2 : shape === 'sphere' ? chosen.radius : shape === 'cylinder' ? chosen.length / 2 : chosen.length / 2 + chosen.radius;
    next.solids.push({
        id, name: name ?? uniqueName(next, labelFor(shape)), shape, size: { ...chosen },
        position: position ? [...position] : [0, 0, lift], rotation: [...identity],
        color: color ?? palette[(next.solids.length) % palette.length],
        mass: { by: 'density', density: materials[0].density, material: materials[0].id }
    });
    return { drawing: next, id };
}

const labelFor = (shape) => shape[0].toUpperCase() + shape.slice(1);

// Changes a solid's name, size, colour, mass, position or rotation. Moving or turning a solid moves its frames with it;
// joints keep joining the same frames, so a joint whose two frames no longer meet says so (see checkDrawing).
export function updateSolid(drawing, id, changes) {
    const next = structuredClone(drawing);
    const solid = solidById(next, id);
    if (!solid) throw new Error(`There is no solid ${id}.`);
    for (const field of ['name', 'color']) if (changes[field] !== undefined) solid[field] = String(changes[field]);
    if (changes.size) solid.size = { ...solid.size, ...changes.size };
    if (changes.shape && changes.shape !== solid.shape) {
        if (!shapeKinds.includes(changes.shape)) throw new Error(`There is no shape "${changes.shape}".`);
        solid.shape = changes.shape;
        solid.size = defaultSize(changes.shape);
    }
    if (changes.mass) solid.mass = { ...changes.mass };
    if (changes.position) solid.position = [...changes.position];
    if (changes.rotation) solid.rotation = normalizeQuaternion(changes.rotation);
    return next;
}

// Moves solids by `offset` (metres), together.
export function moveSolids(drawing, ids, offset) {
    const next = structuredClone(drawing);
    for (const solid of next.solids) if (ids.includes(solid.id)) solid.position = add(solid.position, offset);
    return next;
}

// Turns solids by `turn` (a quaternion) about `pivot`, together, as one rigid piece.
export function turnSolids(drawing, ids, turn, pivot) {
    const next = structuredClone(drawing);
    for (const solid of next.solids) {
        if (!ids.includes(solid.id)) continue;
        solid.position = add(pivot, rotate(turn, subtract(solid.position, pivot)));
        solid.rotation = normalizeQuaternion(multiply(turn, solid.rotation));
    }
    return next;
}

// Removes solids, their frames and the joints on them.
export function removeSolids(drawing, ids) {
    const next = structuredClone(drawing);
    const removedFrames = new Set(next.frames.filter((frame) => ids.includes(frame.solid)).map(({ id }) => id));
    next.solids = next.solids.filter((solid) => !ids.includes(solid.id));
    next.frames = next.frames.filter((frame) => !removedFrames.has(frame.id));
    next.joints = next.joints.filter((joint) => !removedFrames.has(joint.parent) && !removedFrames.has(joint.child));
    return next;
}

// Copies of solids, a little to the side, with their frames (not their joints). Returns { drawing, ids }.
export function duplicateSolids(drawing, ids, offset = [0.05, 0.05, 0]) {
    let next = structuredClone(drawing);
    const created = [];
    for (const id of ids) {
        const solid = solidById(next, id);
        if (!solid) continue;
        const copyId = take(next, 's');
        next.solids.push({ ...structuredClone(solid), id: copyId, name: uniqueName(next, `${solid.name} copy`), position: add(solid.position, offset) });
        for (const frame of next.frames.filter((candidate) => candidate.solid === id)) {
            next.frames.push({ ...structuredClone(frame), id: take(next, 'f'), solid: copyId });
        }
        created.push(copyId);
    }
    return { drawing: next, ids: created };
}

// ---- frames -----------------------------------------------------------------------------------------------------

// A frame on a solid (or the Ground) at `point`, with `axis`, both given in the world, kept in the solid's own frame.
// Returns { drawing, id }.
export function addFrameAt(drawing, solidId, worldPoint, worldAxis) {
    const next = structuredClone(drawing);
    const pose = poseOf(next, solidId);
    const inverse = conjugate(pose.rotation);
    const id = take(next, 'f');
    next.frames.push({
        id, solid: solidId,
        point: rotate(inverse, subtract(worldPoint, pose.position)),
        axis: normalize(rotate(inverse, worldAxis))
    });
    return { drawing: next, id };
}

export function updateFrame(drawing, id, { point, axis }) {
    const next = structuredClone(drawing);
    const frame = frameById(next, id);
    if (!frame) throw new Error(`There is no frame ${id}.`);
    if (point) frame.point = [...point];
    if (axis) frame.axis = normalize(axis);
    return next;
}

// ---- joints -----------------------------------------------------------------------------------------------------

export function defaultDrive(kind) {
    if (kind === 'ball') return { mode: 'free', torque: [0, 0, 0], setpoint: [0, 0, 0], gain: 2, torqueLimit: 2 };
    if (kind !== 'hinge' && kind !== 'slider') return { mode: 'free' };
    // A motor's controller: a duty cycle of `proportional` per unit of the setpoint left to reach, less `derivative`
    // times the joint's speed, held to ±1.
    const hinge = kind === 'hinge';
    return {
        mode: 'free', torque: 0, setpoint: 0, gain: hinge ? 20 : 2000, torqueLimit: hinge ? 2 : 50,
        proportional: hinge ? 5 : 100, derivative: hinge ? 0.2 : 5, motor: defaultMotor(kind)
    };
}

// The solids joined to `start` through joints, not crossing `barrier`: the piece that moves with it.
export function connectedSolids(drawing, start, barrier = null) {
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const seen = new Set([start]);
    const queue = [start];
    while (queue.length) {
        const solid = queue.shift();
        for (const joint of drawing.joints) {
            const ends = [solidOf.get(joint.parent), solidOf.get(joint.child)];
            if (!ends.includes(solid)) continue;
            const other = ends[0] === solid ? ends[1] : ends[0];
            if (other === undefined || other === barrier || other === groundId || seen.has(other)) continue;
            seen.add(other);
            queue.push(other);
        }
    }
    return [...seen].filter((id) => id !== groundId);
}

// Joins two solids at two frames: `parentFrame` (its point is the joint's anchor, its axis the joint's axis) and
// `childFrame`. With `mate` (the default), the child's piece (the child and every solid joined to it, not through the
// parent) is turned so the child frame's axis points against the parent frame's, as two faces meet, and moved so the
// two points meet; the child frame's own point then lies on the parent's. Without it, nothing moves. Returns
// { drawing, id }, or throws when the two frames are on one solid, or the child's piece holds the parent.
export function addJoint(drawing, parentFrameId, childFrameId, { kind = 'hinge', mate = true, name } = {}) {
    if (!jointKinds.includes(kind)) throw new Error(`There is no joint kind "${kind}".`);
    let next = structuredClone(drawing);
    const parentFrame = frameById(next, parentFrameId);
    const childFrame = frameById(next, childFrameId);
    if (!parentFrame || !childFrame) throw new Error('A joint joins two frames that exist.');
    if (parentFrame.solid === childFrame.solid) throw new Error('A joint joins two different solids.');
    if (childFrame.solid === groundId) throw new Error('The Ground is where a robot starts, so it is the parent of a joint, never its child.');
    if (mate) {
        const piece = connectedSolids(next, childFrame.solid, parentFrame.solid);
        if (piece.includes(parentFrame.solid)) throw new Error('These solids are already joined another way, so neither can move to meet the other; join them without moving, or remove a joint first.');
        const target = frameInWorld(next, parentFrame);
        const current = frameInWorld(next, childFrame);
        const turn = between(current.axis, scale(target.axis, -1));
        next = turnSolids(next, piece, turn, current.point);
        const turned = frameInWorld(next, frameById(next, childFrameId));
        next = moveSolids(next, piece, subtract(target.point, turned.point));
    }
    const id = take(next, 'j');
    next.joints.push({
        id, name: name ?? uniqueName(next, jointLabels[kind]), kind, parent: parentFrameId, child: childFrameId,
        limits: null, damping: kind === 'hinge' || kind === 'slider' ? 0.01 : 0, friction: 0, armature: 0, drive: defaultDrive(kind)
    });
    return { drawing: next, id };
}

export function updateJoint(drawing, id, changes) {
    const next = structuredClone(drawing);
    const joint = jointById(next, id);
    if (!joint) throw new Error(`There is no joint ${id}.`);
    if (changes.kind && changes.kind !== joint.kind) {
        if (!jointKinds.includes(changes.kind)) throw new Error(`There is no joint kind "${changes.kind}".`);
        joint.kind = changes.kind;
        joint.limits = null;
        joint.drive = defaultDrive(changes.kind);
    }
    if (changes.name !== undefined) joint.name = String(changes.name);
    for (const field of ['damping', 'friction', 'armature']) if (changes[field] !== undefined) joint[field] = Number(changes[field]);
    if (changes.limits !== undefined) joint.limits = changes.limits ? { lower: Number(changes.limits.lower), upper: Number(changes.limits.upper) } : null;
    // A ball's drive carries its vectors whichever mode it is in, so switching modes keeps them.
    if (changes.drive) {
        const defaults = defaultDrive(joint.kind);
        const motor = changes.drive.motor ? { ...defaults.motor, ...joint.drive?.motor, ...changes.drive.motor } : joint.drive?.motor;
        joint.drive = { ...(joint.kind === 'ball' || changes.drive.mode === 'motor' ? defaults : {}), ...joint.drive, ...changes.drive, ...(motor ? { motor } : {}) };
    }
    return next;
}

// ---- the battery and the keyframes ----------------------------------------------------------------------------------

export function updateBattery(drawing, changes) {
    const next = structuredClone(drawing);
    next.battery = { ...batteryOf(drawing), ...Object.fromEntries(Object.entries(changes).map(([key, value]) => [key, Number(value)])) };
    return next;
}

// The joints keyframes set: those a servo or a motor drives, in the tree (a joint closing a loop is not driven), each a
// hinge or a slider.
export function drivenJoints(drawing) {
    const loops = new Set(structure(drawing).loops.map(({ id }) => id));
    return drawing.joints.filter((joint) => (joint.kind === 'hinge' || joint.kind === 'slider') && !loops.has(joint.id) && ['servo', 'motor'].includes(joint.drive?.mode));
}

// Keyframes are poses in time: { id, time, setpoints: { [jointId]: setpoint } }, kept in time order. Build turns them
// into each driven joint's setpoint schedule, the setpoint moving in a straight line from one keyframe to the next.
export const keyframesOf = (drawing) => [...(drawing.keyframes ?? [])].sort((a, b) => a.time - b.time);

export function addKeyframe(drawing, { time, setpoints }) {
    const next = structuredClone(drawing);
    next.keyframes ??= [];
    const id = take(next, 'k');
    next.keyframes.push({ id, time: Number(time), setpoints: { ...setpoints } });
    next.keyframes = keyframesOf(next);
    return { drawing: next, id };
}

export function updateKeyframe(drawing, id, changes) {
    const next = structuredClone(drawing);
    const keyframe = (next.keyframes ?? []).find((candidate) => candidate.id === id);
    if (!keyframe) throw new Error(`There is no keyframe ${id}.`);
    if (changes.time !== undefined) keyframe.time = Number(changes.time);
    if (changes.setpoints) keyframe.setpoints = { ...changes.setpoints };
    next.keyframes = keyframesOf(next);
    return next;
}

export function removeKeyframe(drawing, id) {
    const next = structuredClone(drawing);
    next.keyframes = (next.keyframes ?? []).filter((keyframe) => keyframe.id !== id);
    return next;
}

// The time a new keyframe takes: a second after the last, or 0 for the first.
export const nextKeyframeTime = (drawing) => {
    const last = keyframesOf(drawing).at(-1);
    return last ? Math.round((last.time + 1) * 1000) / 1000 : 0;
};

// Swaps a joint's parent and child frames: the other solid's frame becomes the anchor and axis.
export function reverseJoint(drawing, id) {
    const next = structuredClone(drawing);
    const joint = jointById(next, id);
    if (!joint) throw new Error(`There is no joint ${id}.`);
    if (frameById(next, joint.parent)?.solid === groundId) throw new Error('A joint on the Ground keeps the Ground as its parent.');
    [joint.parent, joint.child] = [joint.child, joint.parent];
    return next;
}

// Removes joints, and the frames no other joint uses.
export function removeJoints(drawing, ids) {
    const next = structuredClone(drawing);
    next.joints = next.joints.filter((joint) => !ids.includes(joint.id));
    const used = new Set(next.joints.flatMap((joint) => [joint.parent, joint.child]));
    next.frames = next.frames.filter((frame) => used.has(frame.id));
    return next;
}

// ---- structure --------------------------------------------------------------------------------------------------

// How the drawing is held together, from the Ground out: each solid's joint to its parent (the solid nearer the
// Ground, or the Ground), the joints that close a loop, and the solids joined to nothing on the Ground, which float
// freely. Joints are walked from the Ground breadth first, in the drawing's order, so the same drawing always gives
// the same tree. Returns { parentJoint: Map(solid → { joint, parent, child }), loops: [joint], roots: [solid] }:
// `roots` are the solids that float, each the first of its piece in the drawing's order.
export function structure(drawing) {
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const parentJoint = new Map();
    const loops = [];
    const roots = [];
    const reached = new Set([groundId]);
    const used = new Set();
    const walk = (start) => {
        const queue = [start];
        while (queue.length) {
            const solid = queue.shift();
            for (const joint of drawing.joints) {
                if (used.has(joint.id)) continue;
                const ends = [solidOf.get(joint.parent), solidOf.get(joint.child)];
                if (!ends.includes(solid) || ends.includes(undefined)) continue;
                const other = ends[0] === solid ? ends[1] : ends[0];
                used.add(joint.id);
                if (reached.has(other)) { loops.push(joint); continue; }
                reached.add(other);
                parentJoint.set(other, { joint, parent: solid, child: other });
                queue.push(other);
            }
        }
    };
    walk(groundId);
    for (const solid of drawing.solids) {
        if (reached.has(solid.id)) continue;
        reached.add(solid.id);
        roots.push(solid.id);
        walk(solid.id);
    }
    return { parentJoint, loops, roots };
}

// The solids in the order a tree lists them: each after its parent, children in the drawing's order.
export function treeOrder(drawing, built = structure(drawing)) {
    const children = new Map();
    for (const [child, { parent }] of built.parentJoint) {
        if (!children.has(parent)) children.set(parent, []);
        children.get(parent).push(child);
    }
    const order = [];
    const visit = (id, depth) => {
        if (id !== groundId) order.push({ id, depth });
        const kids = (children.get(id) ?? []).sort((a, b) => drawing.solids.findIndex((solid) => solid.id === a) - drawing.solids.findIndex((solid) => solid.id === b));
        for (const child of kids) visit(child, depth + 1);
    };
    visit(groundId, -1);
    for (const root of built.roots) visit(root, 0);
    return order;
}

// ---- checks -----------------------------------------------------------------------------------------------------

// What stops the drawing being built (errors), what it should know (warnings) and how it will be read (notes), each
// { message, id } with the id of the solid or joint it is about, if any.
export function checkDrawing(drawing) {
    const errors = [];
    const warnings = [];
    const notes = [];
    if (!drawing.solids.length) errors.push({ message: 'Add a solid to start: a box, a cylinder, a sphere or a capsule.' });
    const names = new Map();
    for (const item of [...drawing.solids, ...drawing.joints]) {
        const key = item.name.trim().toLowerCase();
        if (!key) errors.push({ id: item.id, message: 'Everything needs a name.' });
        else if (names.has(key)) errors.push({ id: item.id, message: `Two things are called "${item.name}"; give each its own name.` });
        names.set(key, item.id);
    }
    for (const solid of drawing.solids) {
        for (const problem of sizeProblems(solid.shape, solid.size)) errors.push({ id: solid.id, message: `${solid.name}: ${problem}` });
        if (!sizeProblems(solid.shape, solid.size).length && !(massOf(solid) > 0)) errors.push({ id: solid.id, message: `${solid.name}: its mass must be more than zero.` });
    }
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const pairs = new Set();
    for (const joint of drawing.joints) {
        const ends = [solidOf.get(joint.parent), solidOf.get(joint.child)];
        if (ends.includes(undefined)) { errors.push({ id: joint.id, message: `${joint.name}: one of its frames is gone; join the solids again.` }); continue; }
        if (ends[0] === ends[1]) errors.push({ id: joint.id, message: `${joint.name}: a joint joins two different solids.` });
        const pair = [...ends].sort().join('|');
        if (pairs.has(pair)) errors.push({ id: joint.id, message: `${joint.name}: these two solids are already joined.` });
        pairs.add(pair);
        if (joint.limits && !(joint.limits.lower < joint.limits.upper)) errors.push({ id: joint.id, message: `${joint.name}: its lower limit must be below its upper limit.` });
        for (const field of ['damping', 'friction', 'armature']) {
            if (!(Number.isFinite(joint[field]) && joint[field] >= 0)) errors.push({ id: joint.id, message: `${joint.name}: its ${field} must be zero or more.` });
        }
        if (joint.kind === 'ball' && joint.drive?.mode && joint.drive.mode !== 'free') {
            const vector = joint.drive.mode === 'torque' ? joint.drive.torque : joint.drive.setpoint;
            if (!(Array.isArray(vector) && vector.length === 3 && vector.every(Number.isFinite))) errors.push({ id: joint.id, message: `${joint.name}: a ball's ${joint.drive.mode === 'torque' ? 'torque' : 'setpoint'} is three numbers, about x, y and z.` });
        }
        if (joint.drive?.mode === 'motor') {
            if (joint.kind !== 'hinge' && joint.kind !== 'slider') errors.push({ id: joint.id, message: `${joint.name}: a motor drives a hinge or a slider.` });
            // A drawing kept before a field was added has its default.
            const motor = { ...defaultMotor(joint.kind), ...joint.drive.motor };
            for (const field of motorFields) {
                const value = motor[field];
                const allowZero = field === 'rotorInertia';
                if (!(Number.isFinite(value) && (allowZero ? value >= 0 : value > 0))) errors.push({ id: joint.id, message: `${joint.name}: its motor's ${motorFieldNames[field]} must be ${allowZero ? 'zero or more' : 'more than zero'}.` });
            }
            if (motor.windingLimit > 0 && motor.ambientTemperature > 0 && !(motor.windingLimit > motor.ambientTemperature)) {
                errors.push({ id: joint.id, message: `${joint.name}: its motor's hottest winding must be above the ambient temperature.` });
            }
            for (const field of ['proportional', 'derivative']) {
                if (!(Number.isFinite(joint.drive[field]) && joint.drive[field] >= 0)) errors.push({ id: joint.id, message: `${joint.name}: its motor controller's ${field} gain must be zero or more.` });
            }
        }
        if (joint.drive?.mode === 'servo') {
            if (!(joint.drive.gain > 0)) errors.push({ id: joint.id, message: `${joint.name}: a servo's stiffness must be more than zero.` });
            if (!(joint.drive.torqueLimit > 0)) errors.push({ id: joint.id, message: `${joint.name}: a servo's most torque must be more than zero.` });
        }
        // The two frames should meet, unless the user chose to keep them apart.
        const parentWorld = frameInWorld(drawing, frameById(drawing, joint.parent));
        const childWorld = frameInWorld(drawing, frameById(drawing, joint.child));
        const gap = length(subtract(parentWorld.point, childWorld.point));
        if (gap > 1e-4) notes.push({ id: joint.id, message: `${joint.name}: its two frames are ${(gap * 1000).toFixed(1)} mm apart; the joint turns about the first one's point.` });
    }
    const built = structure(drawing);
    for (const joint of built.loops) {
        const held = {
            ball: 'together at its point, free to turn there',
            hinge: 'together at its point, free to turn only about its axis',
            slider: 'on its axis\'s line, free only to slide along it',
            fixed: 'fixed to each other'
        }[joint.kind];
        notes.push({ id: joint.id, message: `${joint.name} closes a loop: Build holds its two solids ${held}, by a force of its own.` });
        if (joint.drive?.mode && joint.drive.mode !== 'free') errors.push({ id: joint.id, message: `${joint.name} closes a loop, so it cannot be driven; drive another joint of the loop.` });
    }
    const battery = batteryOf(drawing);
    if (drawing.joints.some((joint) => joint.drive?.mode === 'motor')) {
        if (!(battery.capacity > 0)) errors.push({ message: 'The battery\'s capacity must be more than zero.' });
        if (!(battery.emptyVoltage > 0 && battery.fullVoltage > battery.emptyVoltage)) errors.push({ message: 'The battery\'s full voltage must be above its empty voltage, and both above zero.' });
    }
    const driven = new Set(drivenJoints(drawing).map(({ id }) => id));
    const times = new Set();
    for (const keyframe of keyframesOf(drawing)) {
        if (!(Number.isFinite(keyframe.time) && keyframe.time >= 0)) errors.push({ id: keyframe.id, message: `A keyframe's time must be zero or more, not ${keyframe.time}.` });
        if (times.has(keyframe.time)) errors.push({ id: keyframe.id, message: `Two keyframes are at ${keyframe.time} s; move one.` });
        times.add(keyframe.time);
        const ignored = Object.keys(keyframe.setpoints).filter((jointId) => !driven.has(jointId));
        if (ignored.length) warnings.push({ id: keyframe.id, message: `The keyframe at ${keyframe.time} s sets ${ignored.map((jointId) => jointById(drawing, jointId)?.name ?? 'a joint that is gone').join(', ')}, which no servo or motor drives now; Build leaves ${ignored.length === 1 ? 'it' : 'them'} out.` });
    }
    for (const root of built.roots) {
        const solid = solidById(drawing, root);
        warnings.push({ id: root, message: `${solid.name} is not joined to the Ground, so it falls freely${connectedSolids(drawing, root).length > 1 ? ' with the solids joined to it' : ''}.` });
    }
    return { errors, warnings, notes };
}
