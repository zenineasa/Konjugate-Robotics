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
// - A body with several joints moves by each in turn, each joint's anchor and axis carried by those before it, as
//   MuJoCo moves it: a chain of links with no mass, one after each joint but the last, each where the body is, the
//   body's solid on the last.
// - A body with no joint is welded to its parent (a fixed joint); a body on the world with a free joint is joined to
//   nothing, and falls; a mocap body is fixed to the Ground.
// - A position actuator on a joint is its servo: stiffness kp gear², setpoint its control (0, held to its control
//   range) over its gear, counted from the ref, most torque its force range times its gear, and its own damping its kv
//   gear² (or, for a dampratio, as MuJoCo works it out from the inertia it moves). A motor actuator is a torque drive
//   of 0, a slider in the canvas. Another kind is left out, and said so.
//
// Pure, no dependencies but frames.mjs, inertia.mjs and drawing.mjs.

import { compose, rotate } from './frames.mjs';
import { principal } from './inertia.mjs';
import { createDrawing, defaultDrive, defaultTendonDrive, groundId } from './drawing.mjs';

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

// The inertia an actuator on a joint's degree of freedom moves at the model's reference pose, as MuJoCo works it out
// for a dampratio: 1 / (M⁻¹)ᵢᵢ, M the model's joint-space mass matrix there (every body's inertia through every joint
// that moves it, each degree of freedom's armature on its diagonal), so the other joints are free to move with it.
// `dofOf(jointName)` gives a hinge's or a slide's degree of freedom; its `inertia(index)` the inertia at that one.
function massMatrix(model) {
    const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const add = (a, b) => a.map((value, axis) => value + b[axis]);
    const scale = (a, factor) => a.map((value) => value * factor);
    // Each degree of freedom: the body it moves (with all beyond it), its motion at the world's origin (ω, v), its armature.
    const dofs = [];
    const named = new Map();
    model.bodies.forEach((body, index) => {
        for (const joint of body.joints) {
            const at = (local) => compose(body.world, { position: local, quaternion: [1, 0, 0, 0] }).position;
            const turn = (local) => rotate(body.world.quaternion, local);
            const units = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
            const armature = joint.armature ?? 0;
            if (joint.kind === 'hinge') {
                const n = normalize(turn(joint.axis));
                named.set(joint.name, dofs.length);
                dofs.push({ index, w: n, v: cross(at(joint.anchor), n), armature });
            } else if (joint.kind === 'slide') {
                named.set(joint.name, dofs.length);
                dofs.push({ index, w: [0, 0, 0], v: normalize(turn(joint.axis)), armature });
            } else if (joint.kind === 'ball') {
                for (const unit of units) { const e = turn(unit); dofs.push({ index, w: e, v: cross(at(joint.anchor), e), armature }); }
            } else {
                for (const unit of units) dofs.push({ index, w: [0, 0, 0], v: unit, armature });
                for (const unit of units) { const e = turn(unit); dofs.push({ index, w: e, v: cross(body.world.position, e), armature }); }
            }
        }
    });
    const beyond = model.bodies.map(() => null);
    const carried = (index) => {
        if (beyond[index]) return beyond[index];
        const set = new Set([index]);
        model.bodies.forEach((body, at) => { if (set.has(body.parent)) set.add(at); });
        beyond[index] = set;
        return set;
    };
    // Each body's inertia in the world: its mass, centre and tensor.
    const world = model.bodies.map(({ inertia, world: frame }) => {
        if (!(inertia?.mass > 0)) return null;
        const centre = compose(frame, { position: inertia.centre, quaternion: [1, 0, 0, 0] }).position;
        const columns = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((axis) => rotate(frame.quaternion, axis));
        const apply = (w) => {
            // R T Rᵀ w, R's columns the body's axes in the world.
            const local = columns.map((column) => dot(column, w));
            const t = inertia.tensor;
            const turned = [0, 1, 2].map((row) => t[3 * row] * local[0] + t[3 * row + 1] * local[1] + t[3 * row + 2] * local[2]);
            return add(add(scale(columns[0], turned[0]), scale(columns[1], turned[1])), scale(columns[2], turned[2]));
        };
        return { mass: inertia.mass, centre, apply };
    });
    const n = dofs.length;
    const M = Array.from({ length: n }, () => Array(n).fill(0));
    for (let a = 0; a < n; a += 1) {
        for (let b = a; b < n; b += 1) {
            let sum = 0;
            for (const k of carried(dofs[a].index)) {
                if (!carried(dofs[b].index).has(k) || !world[k]) continue;
                const { mass, centre, apply } = world[k];
                const ua = add(dofs[a].v, cross(dofs[a].w, centre));
                const ub = add(dofs[b].v, cross(dofs[b].w, centre));
                sum += dot(dofs[a].w, apply(dofs[b].w)) + mass * dot(ua, ub);
            }
            M[a][b] = sum + (a === b ? dofs[a].armature : 0);
            M[b][a] = M[a][b];
        }
    }
    // M⁻¹'s diagonal, by Gauss-Jordan elimination with partial pivoting.
    const inverse = M.map((row, r) => [...row, ...Array.from({ length: n }, (_, c) => (c === r ? 1 : 0))]);
    for (let column = 0; column < n; column += 1) {
        let pivot = column;
        for (let r = column + 1; r < n; r += 1) if (Math.abs(inverse[r][column]) > Math.abs(inverse[pivot][column])) pivot = r;
        [inverse[column], inverse[pivot]] = [inverse[pivot], inverse[column]];
        const lead = inverse[column][column];
        if (!(Math.abs(lead) > 1e-300)) return { dofOf: () => null, inertia: () => null };
        for (let c = 0; c < 2 * n; c += 1) inverse[column][c] /= lead;
        for (let r = 0; r < n; r += 1) {
            if (r === column || !inverse[r][column]) continue;
            const factor = inverse[r][column];
            for (let c = 0; c < 2 * n; c += 1) inverse[r][c] -= factor * inverse[column][c];
        }
    }
    return {
        dofOf: (name) => named.get(name) ?? null,
        inertia: (dof) => 1 / inverse[dof][n + dof],
        // Along a tendon, its coefficients by degree of freedom: 1 / (cᵀ M⁻¹ c).
        inertiaAlong: (coefs) => 1 / coefs.reduce((sum, [a, ca]) => sum + coefs.reduce((inner, [b, cb]) => inner + ca * inverse[a][n + b] * cb, 0), 0)
    };
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
        // Links with no mass for each joint but its last, where the body is.
        const steps = body.mocap ? [] : body.joints.filter(({ kind }) => kind !== 'free');
        const links = steps.slice(0, -1).map((joint) => {
            const linkId = take('s');
            drawing.solids.push({
                id: linkId, name: unique(`${joint.name} link`), shape: 'mesh', size: { x: 0.01, y: 0.01, z: 0.01 },
                position: roundVector(body.world.position), rotation: roundVector(body.world.quaternion),
                color: '#8a96a3', mass: { by: 'mass', value: 0 }, unitInertia: [0, 0, 0],
                source: { body: body.name, link: joint.name }, look: null
            });
            return linkId;
        });
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
        solidOf.set(index, { solidId: id, inBody: { position: [0, 0, 0], quaternion: [1, 0, 0, 0] }, bodyInSolid, merged: false, links });
        lookParts.set(id, looks[index] ? [{ look: looks[index], frame: { position: [0, 0, 0], quaternion: [1, 0, 0, 0] } }] : []);
        if (body.inertiaFrom === 'meshes') leftOut.push(`${body.name}'s inertia is worked out from its meshes' volumes, which may differ a little from MuJoCo's`);
        jointsOfBody.set(index, body);
    });

    // Each solid's look in its own frame, and its size.
    for (const solid of drawing.solids) {
        if (solid.source?.link) continue;
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
        const { solidId, links } = solidOf.get(index);
        const solid = drawing.solids.find((candidate) => candidate.id === solidId);
        const free = body.joints.some(({ kind }) => kind === 'free');
        if (free) {
            if (body.parent >= 0) throw new Error(`Body ${body.name} has a free joint but is not on the world, which MuJoCo refuses too.`);
            continue;
        }
        if (body.mocap) leftOut.push(`${body.name} is moved by MuJoCo's mocap; here it is fixed to the Ground`);
        const steps = body.mocap ? [] : body.joints;
        // The solids each joint joins: the parent's, the links in turn, the body's.
        const chain = [parentSolid(body), ...links, solidId];
        for (let at = 0; at < Math.max(steps.length, 1); at += 1) {
            const joint = steps[at] ?? null;
            const kind = joint ? (joint.kind === 'slide' ? 'slider' : joint.kind) : 'fixed';
            const anchorWorld = joint ? compose(body.world, { position: joint.anchor, quaternion: [1, 0, 0, 0] }).position : solid.position;
            const axisWorld = joint && joint.kind !== 'ball' ? rotate(body.world.quaternion, joint.axis) : [0, 0, 1];
            const parentFrame = addFrame(chain[at], anchorWorld, axisWorld);
            const childFrame = addFrame(chain[at + 1], anchorWorld, axisWorld.map((value) => -value));
            const id = take('j');
            const jointName = unique(joint?.name ?? `${solid.name} weld`);
            const entry = {
                id, name: jointName, kind, parent: parentFrame, child: childFrame,
                limits: null, damping: round(joint?.damping ?? 0), friction: round(joint?.friction ?? 0), armature: round(joint?.armature ?? 0),
                drive: defaultDrive(kind)
            };
            // Its spring: MuJoCo's stiffness pulls it to its springref, counted here from its ref, where it is drawn.
            if (joint?.stiffness && (kind === 'hinge' || kind === 'slider')) Object.assign(entry, { stiffness: round(joint.stiffness), springRest: round(joint.springref - joint.reference) });
            // How softly the model holds its limit and dry friction, where it says other than MuJoCo's defaults.
            const softness = joint?.soft ? Object.fromEntries([
                ...['limit', 'friction'].filter((which) => joint.soft[which].solref || joint.soft[which].solimp)
                    .map((which) => [which, { ...(joint.soft[which].solref ? { solref: joint.soft[which].solref.map(round) } : {}), ...(joint.soft[which].solimp ? { solimp: joint.soft[which].solimp.map(round) } : {}) }]),
                ...(joint.soft.margin !== undefined && joint.soft.margin !== 0 ? [['margin', round(joint.soft.margin)]] : [])
            ]) : {};
            if (Object.keys(softness).length && (kind === 'hinge' || kind === 'slider')) entry.softness = softness;
            if (joint?.range) {
                entry.limits = kind === 'ball' ? { lower: 0, upper: round(joint.range[1]) } : { lower: round(joint.range[0] - joint.reference), upper: round(joint.range[1] - joint.reference) };
            }
            if (joint?.range && kind !== 'ball' && !(joint.range[0] <= joint.reference && joint.reference <= joint.range[1])) {
                leftOut.push(`joint ${joint.name} is drawn outside its range (the model's reference pose is), so its limit pushes it back from the start, which MuJoCo does a little differently`);
            }
            if (joint && joint.reference) leftOut.push(`joint ${joint.name}'s ref: its position here is counted from ${round(joint.reference)} ${kind === 'slider' ? 'm' : 'rad'}, where the model puts its body`);
            drawing.joints.push(entry);
            if (joint) jointOf.set(joint.name, { entry, joint });
            joints += 1;
        }
    }

    // Couplings (joint equalities) and fixed tendons, over the joints that came in.
    const couplingIds = new Map();
    for (const coupling of model.couplings ?? []) {
        const first = jointOf.get(coupling.joint1);
        const second = coupling.joint2 ? jointOf.get(coupling.joint2) : null;
        if (!first || (coupling.joint2 && !second) || [first, second].some((target) => target && target.entry.kind !== 'hinge' && target.entry.kind !== 'slider')) {
            leftOut.push(`the joint constraint ${coupling.name}, on joints that did not come in as hinges or sliders`);
            continue;
        }
        // MuJoCo counts both from their qpos0, the ref, as the drawing counts its joints from where they are drawn.
        const entry = { id: take('c'), name: unique(coupling.name), first: first.entry.id, second: second?.entry.id ?? null, polycoef: coupling.polycoef.map(round) };
        const softness = Object.fromEntries(['solref', 'solimp'].filter((key) => coupling.soft?.[key]).map((key) => [key, coupling.soft[key].map(round)]));
        if (Object.keys(softness).length) entry.softness = softness;
        (drawing.couplings ??= []).push(entry);
        couplingIds.set(coupling.name, entry.id);
    }
    const tendonOf = new Map();
    for (const tendon of model.tendons ?? []) {
        const parts = tendon.parts.map(({ joint, coef }) => ({ target: jointOf.get(joint), coef }));
        if (!parts.length || parts.some(({ target }) => !target || (target.entry.kind !== 'hinge' && target.entry.kind !== 'slider'))) {
            leftOut.push(`tendon ${tendon.name}, over joints that did not come in as hinges or sliders`);
            continue;
        }
        // Its length as MuJoCo has it (over each joint's qpos, the ref where the robot is drawn) where it is drawn.
        const offset = parts.reduce((sum, { target, coef }) => sum + coef * target.joint.reference, 0);
        const entry = {
            id: take('t'), name: unique(tendon.name), parts: parts.map(({ target, coef }) => ({ joint: target.entry.id, coef: round(coef) })), offset: round(offset),
            // Its spring's length (or a dead band's two ends), by default its length at the joints' springref.
            stiffness: round(tendon.stiffness), damping: round(tendon.damping), friction: round(tendon.friction),
            springLength: round(tendon.springLength?.[0] ?? parts.reduce((sum, { target, coef }) => sum + coef * (target.joint.springref ?? 0), 0)),
            ...(tendon.springLength && tendon.springLength[1] !== tendon.springLength[0] ? { springLengthUpper: round(tendon.springLength[1]) } : {}),
            limits: tendon.range ? { lower: round(tendon.range[0]), upper: round(tendon.range[1]) } : null,
            drive: defaultTendonDrive()
        };
        const softness = Object.fromEntries([
            ...['limit', 'friction'].filter((which) => tendon.soft[which].solref || tendon.soft[which].solimp)
                .map((which) => [which, Object.fromEntries(['solref', 'solimp'].filter((key) => tendon.soft[which][key]).map((key) => [key, tendon.soft[which][key].map(round)]))]),
            ...(tendon.soft.margin ? [['margin', round(tendon.soft.margin)]] : [])
        ]);
        if (Object.keys(softness).length) entry.softness = softness;
        (drawing.tendons ??= []).push(entry);
        tendonOf.set(tendon.name, { entry, parts });
    }

    // Actuators: each joint's or tendon's first that the drawing can take, as its drive.
    let drives = 0;
    const driven = new Set();
    let matrix = null;
    for (const actuator of model.actuators) {
        const label = actuator.name || `a ${actuator.element} actuator`;
        if (actuator.transmission !== 'joint' && actuator.transmission !== 'tendon') { leftOut.push(`${label}, which acts through a ${actuator.transmission} rather than a joint or a tendon`); continue; }
        const onTendon = actuator.transmission === 'tendon';
        const target = onTendon ? tendonOf.get(actuator.tendon) : jointOf.get(actuator.joint);
        const what = onTendon ? `tendon ${actuator.tendon}` : `joint ${actuator.joint}`;
        if (!target || (!onTendon && target.entry.kind === 'ball')) { leftOut.push(`${label}, on ${what}, which ${target ? 'is a ball' : 'did not come in'}`); continue; }
        if (driven.has(what)) { leftOut.push(`${label}: ${what} has a drive already`); continue; }
        const { entry } = target;
        const reference = onTendon ? 0 : target.joint.reference;
        const gear = actuator.gear || 1;
        const base = onTendon ? defaultTendonDrive() : defaultDrive(entry.kind);
        if (actuator.kind === 'position') {
            const force = actuator.forceRange ? Math.max(Math.abs(actuator.forceRange[0]), Math.abs(actuator.forceRange[1])) : null;
            // Its control of 0, held to its control range as MuJoCo holds it, gives its target (the control itself for a
            // position actuator; for a general one, its gain times it plus its first bias, over its stiffness), over
            // its gear, counted from the ref.
            const control = actuator.controlRange ? Math.min(Math.max(0, actuator.controlRange[0]), actuator.controlRange[1]) : 0;
            const aim = (actuator.controlScale ?? 1) * control + (actuator.controlOffset ?? 0);
            entry.drive = {
                ...base, mode: 'servo', gain: round(actuator.gain * gear * gear),
                setpoint: round(aim / gear - reference),
                torqueLimit: round(force === null ? 1e6 : force * Math.abs(gear))
            };
            if (actuator.forceRange && Math.abs(actuator.forceRange[0] + actuator.forceRange[1]) > 1e-12) leftOut.push(`${label}'s force range is not even about zero: the servo gives up to ${round(force)} either way`);
            if (force === null) leftOut.push(`${label} has no force range: the servo is given a most torque of 10⁶`);
            // Its kv is the servo's own damping, inside its most torque as in MuJoCo.
            if (actuator.damping) entry.drive.damping = round(actuator.damping * gear * gear);
            if (actuator.dampratio) {
                // As MuJoCo works it out: dampratio times twice the root of kp times the inertia the actuator moves at
                // the model's reference pose (massMatrix), over gear²; times gear² at the joint or tendon.
                matrix ??= massMatrix(model);
                const inertia = onTendon
                    ? matrix.inertiaAlong(target.parts.map(({ target: part, coef }) => [matrix.dofOf(part.joint.name), coef]))
                    : matrix.inertia(matrix.dofOf(target.joint.name));
                entry.drive.damping = round(actuator.dampratio * 2 * Math.sqrt(actuator.gain * gear * gear * inertia));
            }
        } else if (actuator.kind === 'motor') {
            entry.drive = { ...base, mode: 'torque', torque: 0 };
        } else {
            leftOut.push(`${label}, a ${actuator.element} actuator the drawing cannot take yet`);
            continue;
        }
        driven.add(what);
        drives += 1;
    }
    return { drawing, looks: solidLooks, report: { solids: drawing.solids.length, joints, drives, leftOut } };
}

