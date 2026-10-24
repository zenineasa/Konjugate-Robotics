/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A MuJoCo model (as lib/mjcf.mjs reads it) as a drawing: each body a solid, each joint a joint, each actuator the
// drive of its joint, with a report of what came in, what was left out and what was taken approximately.
//
// - A body with mass is a solid of shape `mesh`: its own frame is the body's principal frame of inertia (at its centre
//   of mass, along its principal axes, as MuJoCo's compiler puts it), so its inertia is three numbers, kept per
//   kilogram so that a change of mass (a slider, a fork) scales it as the body's would. Its look (`looks`, by the key
//   in its `look`, which a duplicate shares) is its visible geoms in that frame; its `size` the box around them, or
//   with nothing visible the box of the same inertia.
// - A body with no mass and no joint is part of its parent: its look joins the parent's and its children hang from the
//   parent. One with no mass that moves is refused, as MuJoCo refuses it.
// - A hinge, slide or ball joint is the drawing's hinge, slider or ball, at the joint's anchor along its axis with the
//   body where the model puts it; its range, damping, frictionloss and armature come with it. A joint's position in
//   the drawing is counted from where the solid is drawn, which is the model's ref, so a range is moved by the ref.
// - A body with no joint is welded to its parent (a fixed joint); a body on the world with a free joint is joined to
//   nothing, and falls; a mocap body is fixed to the Ground.
// - A position actuator on a joint is its servo: stiffness kp gear², setpoint its control (0) over its gear, counted
//   from the ref, most torque its force range times its gear; its kv joins the joint's damping. A motor actuator is a
//   torque drive of 0, a slider in the canvas. Another kind is left out, and said so.
//
// Pure, no dependencies but frames.mjs, inertia.mjs and drawing.mjs.

import { compose, rotate } from './frames.mjs';
import { principal } from './inertia.mjs';
import { createDrawing, defaultDrive, groundId } from './drawing.mjs';

const conjugate = ([w, x, y, z]) => [w, -x, -y, -z];
const subtract = (a, b) => a.map((value, axis) => value - b[axis]);
const normalize = (vector) => { const length = Math.hypot(...vector); return vector.map((value) => value / length); };
const round = (value) => Math.round(value * 1e12) / 1e12 + 0;
const roundVector = (vector) => vector.map(round);

// A box with this inertia per kilogram (its principal moments): its full sides.
function boxOfInertia([a, b, c]) {
    const side = (i, j, k) => Math.sqrt(Math.max(6 * (j + k - i), 0));
    return { x: side(a, b, c), y: side(b, a, c), z: side(c, a, b) };
}

// The drawing a model makes, as { drawing, looks, report }. `model` is lib/mjcf.mjs's readModel; `looks` maps each
// body's index to its look ({ mesh, color } in the body's frame, from bodyLook) or null; `name` names the drawing.
// The report: { solids, joints, drives, leftOut } with leftOut in words, the model's own and the import's.
export function importModel(model, looks, { name = 'Robot' } = {}) {
    const drawing = createDrawing(name);
    const leftOut = [...model.leftOut];
    const take = (prefix) => `${prefix}${drawing.nextId++}`;
    const names = new Set();
    const unique = (base) => {
        let candidate = base || 'Part';
        for (let suffix = 2; names.has(candidate.toLowerCase()); suffix += 1) candidate = `${base} ${suffix}`;
        names.add(candidate.toLowerCase());
        return candidate;
    };
    // Each body's solid: { solidId, frame } where frame is the solid's frame in the body's (for a body merged into its
    // parent, the parent's solid and the body's place in it).
    const solidOf = new Map();
    const lookParts = new Map();
    const solidLooks = {};
    const jointsOfBody = new Map();

    model.bodies.forEach((body, index) => {
        const mass = body.inertia?.mass ?? 0;
        const moving = body.joints.length > 0;
        if (!(mass > 0)) {
            if (moving) throw new Error(`Body ${body.name} moves but has no mass, which MuJoCo refuses too.`);
            // Part of its parent: welded to it with nothing of its own but its look.
            const parent = body.parent >= 0 ? solidOf.get(body.parent) : null;
            if (!parent) {
                solidOf.set(index, null);
                if (looks[index]) leftOut.push(`${body.name}, fixed to the world with no mass: its look is left out`);
                return;
            }
            solidOf.set(index, { solidId: parent.solidId, inBody: compose(parent.inBody, body.local), merged: true });
            if (looks[index]) lookParts.get(parent.solidId).push({ look: looks[index], frame: compose(parent.inBody, body.local) });
            return;
        }
        // The solid at the body's centre of mass, along its principal axes.
        const { moments, quaternion } = principal(body.inertia.tensor);
        const inBody = { position: body.inertia.centre, quaternion };
        const world = compose(body.world, inBody);
        const id = take('s');
        drawing.solids.push({
            id, name: unique(body.name), shape: 'mesh', size: null,
            position: roundVector(world.position), rotation: roundVector(world.quaternion),
            color: looks[index]?.color ?? '#8a96a3', mass: { by: 'mass', value: round(mass) },
            unitInertia: roundVector(moments.map((moment) => moment / mass)),
            source: { body: body.name, inertia: body.inertiaFrom }
        });
        // The body's frame relative to its solid's.
        const bodyInSolid = { position: rotate(conjugate(quaternion), inBody.position.map((value) => -value)), quaternion: conjugate(quaternion) };
        solidOf.set(index, { solidId: id, inBody: { position: [0, 0, 0], quaternion: [1, 0, 0, 0] }, bodyInSolid, merged: false });
        lookParts.set(id, looks[index] ? [{ look: looks[index], frame: { position: [0, 0, 0], quaternion: [1, 0, 0, 0] } }] : []);
        if (body.inertiaFrom === 'meshes') leftOut.push(`${body.name}'s inertia is worked out from its meshes' volumes, which may differ a little from MuJoCo's`);
        jointsOfBody.set(index, body);
    });

    // Each solid's look in its own frame, and its size.
    for (const solid of drawing.solids) {
        const index = [...solidOf].find(([, entry]) => entry?.solidId === solid.id && !entry.merged)[0];
        const { bodyInSolid } = solidOf.get(index);
        const position = [];
        const indexList = [];
        for (const { look, frame } of lookParts.get(solid.id)) {
            const offset = position.length / 3;
            const toSolid = compose(bodyInSolid, frame);
            for (let at = 0; at < look.mesh.position.length; at += 3) {
                position.push(...compose(toSolid, { position: look.mesh.position.slice(at, at + 3), quaternion: [1, 0, 0, 0] }).position.map(round));
            }
            indexList.push(...look.mesh.index.map((value) => value + offset));
        }
        if (position.length) {
            solidLooks[solid.id] = { mesh: { position, normal: [], index: indexList }, color: solid.color };
            const low = [Infinity, Infinity, Infinity];
            const high = [-Infinity, -Infinity, -Infinity];
            for (let at = 0; at < position.length; at += 3) for (let axis = 0; axis < 3; axis += 1) { low[axis] = Math.min(low[axis], position[at + axis]); high[axis] = Math.max(high[axis], position[at + axis]); }
            // About the solid's centre, the box reaches as far as the look does either way.
            solid.size = { x: round(2 * Math.max(-low[0], high[0])), y: round(2 * Math.max(-low[1], high[1])), z: round(2 * Math.max(-low[2], high[2])) };
        } else {
            solid.size = boxOfInertia(solid.unitInertia);
        }
        solid.look = solidLooks[solid.id] ? { key: solid.id } : null;
    }

    // Frames on solids, in each solid's own frame, from a point and axis in the world.
    const addFrame = (solidId, worldPoint, worldAxis) => {
        const id = take('f');
        if (solidId === groundId) {
            drawing.frames.push({ id, solid: groundId, point: roundVector(worldPoint), axis: roundVector(normalize(worldAxis)) });
            return id;
        }
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const inverse = conjugate(solid.rotation);
        drawing.frames.push({ id, solid: solidId, point: roundVector(rotate(inverse, subtract(worldPoint, solid.position))), axis: roundVector(normalize(rotate(inverse, worldAxis))) });
        return id;
    };
    // The solid a body hangs from: its parent's (through bodies merged into theirs), or the Ground.
    const parentSolid = (body) => (body.parent >= 0 ? solidOf.get(body.parent)?.solidId ?? groundId : groundId);

    const jointOf = new Map();
    let joints = 0;
    for (const [index, body] of jointsOfBody) {
        const solidId = solidOf.get(index).solidId;
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const parent = parentSolid(body);
        const moving = body.joints.filter(({ kind }) => kind !== 'free');
        const free = body.joints.some(({ kind }) => kind === 'free');
        if (free) {
            if (body.parent >= 0) throw new Error(`Body ${body.name} has a free joint but is not on the world, which MuJoCo refuses too.`);
            continue;
        }
        let kind = 'fixed';
        let joint = null;
        if (moving.length && !body.mocap) {
            joint = moving[0];
            kind = joint.kind === 'slide' ? 'slider' : joint.kind;
        }
        if (body.mocap) leftOut.push(`${body.name} is moved by MuJoCo's mocap; here it is fixed to the Ground`);
        const anchorWorld = joint ? compose(body.world, { position: joint.anchor, quaternion: [1, 0, 0, 0] }).position : solid.position;
        const axisWorld = joint && joint.kind !== 'ball' ? rotate(body.world.quaternion, joint.axis) : [0, 0, 1];
        const parentFrame = addFrame(parent, anchorWorld, axisWorld);
        const childFrame = addFrame(solidId, anchorWorld, axisWorld.map((value) => -value));
        const id = take('j');
        const jointName = unique(joint?.name ?? `${solid.name} weld`);
        const entry = {
            id, name: jointName, kind, parent: parentFrame, child: childFrame,
            limits: null, damping: round(joint?.damping ?? 0), friction: round(joint?.friction ?? 0), armature: round(joint?.armature ?? 0),
            drive: defaultDrive(kind)
        };
        if (joint?.range) {
            entry.limits = kind === 'ball' ? { lower: 0, upper: round(joint.range[1]) } : { lower: round(joint.range[0] - joint.reference), upper: round(joint.range[1] - joint.reference) };
        }
        if (joint && joint.reference) leftOut.push(`joint ${joint.name}'s ref: its position here is counted from ${round(joint.reference)} ${kind === 'slider' ? 'm' : 'rad'}, where the model puts its body`);
        drawing.joints.push(entry);
        if (joint) jointOf.set(joint.name, { entry, joint });
        joints += 1;
    }

    // Actuators: each joint's first that the drawing can take, as its drive.
    let drives = 0;
    const driven = new Set();
    for (const actuator of model.actuators) {
        const label = actuator.name || `a ${actuator.element} actuator`;
        if (actuator.transmission !== 'joint') { leftOut.push(`${label}, which acts through a ${actuator.transmission} rather than a joint`); continue; }
        const target = jointOf.get(actuator.joint);
        if (!target || target.entry.kind === 'ball') { leftOut.push(`${label}, on joint ${actuator.joint}, which ${target ? 'is a ball' : 'did not come in'}`); continue; }
        if (driven.has(actuator.joint)) { leftOut.push(`${label}: joint ${actuator.joint} has a drive already`); continue; }
        const { entry, joint } = target;
        const gear = actuator.gear || 1;
        if (actuator.kind === 'position') {
            const force = actuator.forceRange ? Math.max(Math.abs(actuator.forceRange[0]), Math.abs(actuator.forceRange[1])) : null;
            entry.drive = {
                ...defaultDrive(entry.kind), mode: 'servo', gain: round(actuator.gain * gear * gear),
                setpoint: round(-joint.reference),
                torqueLimit: round(force === null ? 1e6 : force * Math.abs(gear))
            };
            if (actuator.forceRange && Math.abs(actuator.forceRange[0] + actuator.forceRange[1]) > 1e-12) leftOut.push(`${label}'s force range is not even about zero: the servo gives up to ${round(force)} either way`);
            if (force === null) leftOut.push(`${label} has no force range: the servo is given a most torque of 10⁶`);
            if (actuator.damping) { entry.damping = round(entry.damping + actuator.damping * gear * gear); leftOut.push(`${label}'s kv is taken as its joint's damping`); }
        } else if (actuator.kind === 'motor') {
            entry.drive = { ...defaultDrive(entry.kind), mode: 'torque', torque: 0 };
        } else {
            leftOut.push(`${label}, a ${actuator.element} actuator the drawing cannot take yet`);
            continue;
        }
        driven.add(actuator.joint);
        drives += 1;
    }
    return { drawing, looks: solidLooks, report: { solids: drawing.solids.length, joints, drives, leftOut } };
}

