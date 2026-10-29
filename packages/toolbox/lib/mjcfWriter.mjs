/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A drawing as an MJCF model, MuJoCo's own format, to run in MuJoCo on its own as a check on the built model (and later
// for export). Pure: the same drawing always gives the same text.
//
// - Each solid is a body with one geom of its shape (an imported solid: its inertia, and the box about it). Bodies
//   nest as the drawing's structure has them (lib/drawing.mjs): a solid sits in the body of its parent joint's other solid, at its place in the drawing relative to it, and the
//   Ground is MuJoCo's world.
// - A joint in the tree is a joint element in the child's body: a hinge or a slide at the parent frame's point, along
//   the parent frame's axis (turned round when the tree reads the joint the other way, so its position still measures
//   the child frame's solid against the parent frame's); a ball; or nothing, for a fixed joint, which welds the child to
//   its parent.
// - A joint that closes a loop is an equality constraint: `connect` (the two solids held together at its point, free to
//   turn, which with the loop's other joints parallel makes a planar linkage exact) for a hinge or a ball, `weld` for a
//   fixed joint. MuJoCo has no equality for a slider, so a loop a slider closes is left open here.
// - A solid joined to nothing on the Ground has a free joint, and falls.
// - An imported link with no mass is not a body: its joint is the first of the body of the solid it carries, as the
//   model it came from had it (MuJoCo refuses a moving body with no mass).
// - A servo is a position actuator on its joint: its stiffness, and its torque limited.
// - A coupling is a joint equality, its polynomial as it is (both joints counted from where they are drawn, as MuJoCo
//   counts them from qpos0). A tendon is a fixed tendon; MuJoCo counts its length from nothing where the robot is
//   drawn, so its spring length and range are written less its offset. Its drive is a general actuator on it: a force,
//   or a servo kp (setpoint - L) - kv L̇ as gain 0 and an affine bias, its force limited.
// - Contact is off: every geom has contype and conaffinity 0. Solids drawn touching at a joint would otherwise push each
//   other apart. Contact comes with its own milestone (docs/direction.md).
//
// Names: each body is named by its solid's id and each joint by its joint's id (`s3`, `j2`), so names in the drawing
// can be anything; the model builder knows them by the same ids.

import { couplingsOf, structure, solidById, frameById, frameInWorld, groundId, massOf, poseOf, tendonsOf } from './drawing.mjs';
import { directionToLocal, formatNumber, formatVector, normalize, relativePose, scale, toLocal } from './math.mjs';
import { mjcfGeometry } from './shapes.mjs';

const escape = (text) => String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

// MuJoCo's own timestep: a millisecond, the built model's default step, so the two step alike.
export const mujocoTimestep = 0.001;

// The joints the mechanism drives or reports, in the drawing's order: { joint, child (the solid it carries), type
// ('hinge' | 'slide' | 'ball'), flipped }. Loop joints and fixed ones are not among them.
export function treeJoints(drawing, built = structure(drawing)) {
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const result = [];
    for (const [child, { joint }] of built.parentJoint) {
        if (joint.kind === 'fixed') continue;
        const flipped = solidOf.get(joint.child) !== child;
        result.push({ joint, child, type: joint.kind === 'slider' ? 'slide' : joint.kind, flipped });
    }
    return result.sort((a, b) => drawing.joints.indexOf(a.joint) - drawing.joints.indexOf(b.joint));
}

export function writeMjcf(drawing, { timestep = mujocoTimestep } = {}) {
    const built = structure(drawing);
    const children = new Map();
    for (const [child, { parent }] of built.parentJoint) {
        if (!children.has(parent)) children.set(parent, []);
        children.get(parent).push(child);
    }
    const order = (ids) => [...ids].sort((a, b) => drawing.solids.findIndex(({ id }) => id === a) - drawing.solids.findIndex(({ id }) => id === b));
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const lines = [];
    const write = (depth, text) => lines.push(`${'  '.repeat(depth)}${text}`);

    // A joint element in body `bodyId`'s frame, for `joint`, which carries solid `carried` (the body's own solid, or a
    // link with no mass before it).
    const writeJoint = (joint, carried, bodyId, depth) => {
        if (joint.kind === 'fixed') return;
        const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
        const pose = poseOf(drawing, bodyId);
        const flipped = solidOf.get(joint.child) !== carried;
        const axis = normalize(scale(directionToLocal(pose, anchor.axis), flipped ? -1 : 1));
        const attributes = [`name="${joint.id}"`, `type="${joint.kind === 'slider' ? 'slide' : joint.kind}"`, `pos="${formatVector(toLocal(pose, anchor.point))}"`];
        if (joint.kind !== 'ball') attributes.push(`axis="${formatVector(axis)}"`);
        // A ball's range is its most turn, from 0.
        if (joint.limits) attributes.push('limited="true"', `range="${formatNumber(joint.kind === 'ball' ? 0 : joint.limits.lower)} ${formatNumber(joint.limits.upper)}"`);
        attributes.push(`damping="${formatNumber(joint.damping ?? 0)}"`, `frictionloss="${formatNumber(joint.friction ?? 0)}"`, `armature="${formatNumber(joint.armature ?? 0)}"`);
        // Its spring, pulling it to its rest from where it is drawn (MuJoCo's qpos0 here).
        if (joint.stiffness > 0 && (joint.kind === 'hinge' || joint.kind === 'slider')) attributes.push(`stiffness="${formatNumber(joint.stiffness)}"`, `springref="${formatNumber(joint.springRest ?? 0)}"`);
        // How softly its limit and dry friction are held, where the drawing says other than MuJoCo's defaults.
        for (const which of ['limit', 'friction']) {
            if (joint.softness?.[which]?.solref) attributes.push(`solref${which}="${formatVector(joint.softness[which].solref)}"`);
            if (joint.softness?.[which]?.solimp) attributes.push(`solimp${which}="${formatVector(joint.softness[which].solimp)}"`);
        }
        if (joint.softness?.margin) attributes.push(`margin="${formatNumber(joint.softness.margin)}"`);
        write(depth, `<joint ${attributes.join(' ')}/>`);
    };
    // A link with no mass (an imported body's joints but its last) is not a body of its own, which MuJoCo refuses: its
    // joint is written first in the body of the one solid it carries, as MuJoCo moves a body by its joints in turn.
    // `before` are those joints, each with the link it carries.
    const massless = (id) => { const solid = solidById(drawing, id); return solid.shape === 'mesh' && massOf(solid) === 0; };
    const body = (id, parent, depth, floating, before = []) => {
        const solid = solidById(drawing, id);
        const link = built.parentJoint.get(id);
        if (massless(id) && link && !floating) {
            const carried = children.get(id) ?? [];
            if (carried.length !== 1) throw new Error(`${solid.name} has no mass and carries ${carried.length} solids; MuJoCo needs a mass for it.`);
            if (built.loops.some((joint) => [solidOf.get(joint.parent), solidOf.get(joint.child)].includes(id))) throw new Error(`${solid.name} has no mass and a loop closes on it; MuJoCo needs a mass for it.`);
            body(carried[0], parent, depth, false, [...before, { joint: link.joint, carried: id }]);
            return;
        }
        const local = relativePose(poseOf(drawing, parent), poseOf(drawing, id));
        write(depth, `<body name="${id}" pos="${formatVector(local.position)}" quat="${formatVector(local.rotation)}">`);
        if (floating) write(depth + 1, '<freejoint/>');
        for (const step of before) writeJoint(step.joint, step.carried, id, depth + 1);
        if (link && !floating) writeJoint(link.joint, id, id, depth + 1);
        const geometry = mjcfGeometry(solid.shape, solid.size);
        if (solid.shape === 'mesh') {
            // An imported solid: its inertia as it is, along its own (principal) axes, and the box about it to show.
            write(depth + 1, `<inertial pos="0 0 0" mass="${formatNumber(massOf(solid))}" diaginertia="${formatVector(solid.unitInertia.map((value) => value * massOf(solid)))}"/>`);
            write(depth + 1, `<geom name="${id}" type="box" size="${formatVector(geometry.size.map((value) => Math.max(value, 1e-4)))}" mass="0" rgba="${rgba(solid.color)}" contype="0" conaffinity="0"/>`);
        } else {
            write(depth + 1, `<geom name="${id}" type="${geometry.type}" size="${formatVector(geometry.size)}" mass="${formatNumber(massOf(solid))}" rgba="${rgba(solid.color)}" contype="0" conaffinity="0"/>`);
        }
        for (const child of order(children.get(id) ?? [])) body(child, id, depth + 1, false);
        write(depth, '</body>');
    };

    write(0, `<mujoco model="${escape(drawing.name || 'Robot')}">`);
    // Angles in radians, as the drawing has them: MuJoCo reads a joint's range in degrees otherwise.
    write(1, '<compiler angle="radian"/>');
    write(1, `<option timestep="${formatNumber(timestep)}" gravity="0 0 -9.81" integrator="implicitfast">`);
    write(2, '<flag energy="enable"/>');
    write(1, '</option>');
    write(1, '<worldbody>');
    write(2, '<geom name="floor" type="plane" size="0 0 0.05" rgba="0.5 0.55 0.6 1" contype="0" conaffinity="0"/>');
    for (const child of order(children.get(groundId) ?? [])) body(child, groundId, 2, false);
    for (const root of built.roots) {
        body(root, groundId, 2, true);
    }
    write(1, '</worldbody>');

    const loops = built.loops.filter((joint) => joint.kind !== 'slider');
    const softAttributes = (softness, suffix = '') => ['solref', 'solimp'].filter((key) => softness?.[key]).map((key) => `${key}${suffix}="${formatVector(softness[key])}"`);
    const tendons = tendonsOf(drawing);
    if (tendons.length) {
        write(1, '<tendon>');
        for (const tendon of tendons) {
            const attributes = [`name="${tendon.id}"`, `stiffness="${formatNumber(tendon.stiffness)}"`, `springlength="${formatNumber(tendon.springLength - tendon.offset)} ${formatNumber((tendon.springLengthUpper ?? tendon.springLength) - tendon.offset)}"`,
                `damping="${formatNumber(tendon.damping)}"`, `frictionloss="${formatNumber(tendon.friction)}"`];
            if (tendon.limits) attributes.push('limited="true"', `range="${formatNumber(tendon.limits.lower - tendon.offset)} ${formatNumber(tendon.limits.upper - tendon.offset)}"`);
            for (const which of ['limit', 'friction']) attributes.push(...softAttributes(tendon.softness?.[which], which));
            if (tendon.softness?.margin) attributes.push(`margin="${formatNumber(tendon.softness.margin)}"`);
            write(2, `<fixed ${attributes.join(' ')}>`);
            for (const { joint, coef } of tendon.parts) write(3, `<joint joint="${joint}" coef="${formatNumber(coef)}"/>`);
            write(2, '</fixed>');
        }
        write(1, '</tendon>');
    }
    const couplings = couplingsOf(drawing);
    if (loops.length || couplings.length) {
        write(1, '<equality>');
        for (const coupling of couplings) {
            write(2, `<joint ${[`name="${coupling.id}"`, `joint1="${coupling.first}"`, ...(coupling.second ? [`joint2="${coupling.second}"`] : []), `polycoef="${formatVector(coupling.polycoef)}"`, ...softAttributes(coupling.softness)].join(' ')}/>`);
        }
        for (const joint of loops) {
            const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
            const first = solidOf.get(joint.parent);
            const second = solidOf.get(joint.child);
            const body1 = first === groundId ? 'world' : first;
            const body2 = second === groundId ? 'world' : second;
            if (joint.kind === 'fixed') {
                write(2, `<weld name="${joint.id}" body1="${body1}" body2="${body2}"/>`);
            } else {
                // The anchor in body1's frame.
                write(2, `<connect name="${joint.id}" body1="${body1}" body2="${body2}" anchor="${formatVector(toLocal(poseOf(drawing, first), anchor.point))}"/>`);
            }
        }
        write(1, '</equality>');
    }

    const servos = treeJoints(drawing, built).filter(({ joint, type }) => type !== 'ball' && joint.drive?.mode === 'servo');
    const pulled = tendons.filter(({ drive }) => drive?.mode === 'servo' || drive?.mode === 'torque');
    if (servos.length || pulled.length) {
        write(1, '<actuator>');
        for (const { joint } of servos) {
            const limit = joint.drive.torqueLimit;
            write(2, `<position name="${joint.id}" joint="${joint.id}" kp="${formatNumber(joint.drive.gain)}"${joint.drive.damping > 0 ? ` kv="${formatNumber(joint.drive.damping)}"` : ''} forcelimited="true" forcerange="${formatNumber(-limit)} ${formatNumber(limit)}"/>`);
        }
        for (const { id, drive, offset } of pulled) {
            if (drive.mode === 'torque') {
                write(2, `<general name="${id}" tendon="${id}" gainprm="0" biastype="affine" biasprm="${formatNumber(drive.torque ?? 0)} 0 0"/>`);
                continue;
            }
            const bias = [drive.gain * (drive.setpoint - offset), -drive.gain, -(drive.damping ?? 0)];
            write(2, `<general name="${id}" tendon="${id}" gainprm="0" biastype="affine" biasprm="${formatVector(bias)}" forcelimited="true" forcerange="${formatNumber(-drive.torqueLimit)} ${formatNumber(drive.torqueLimit)}"/>`);
        }
        write(1, '</actuator>');
    }
    write(0, '</mujoco>');
    return `${lines.join('\n')}\n`;
}

// A colour "#rrggbb" as MJCF's rgba.
function rgba(color) {
    const match = /^#([0-9a-f]{6})$/i.exec(color ?? '');
    if (!match) return '0.6 0.6 0.6 1';
    const value = Number.parseInt(match[1], 16);
    return [(value >> 16) & 255, (value >> 8) & 255, value & 255].map((part) => formatNumber(Math.round((part / 255) * 1000) / 1000)).concat('1').join(' ');
}
