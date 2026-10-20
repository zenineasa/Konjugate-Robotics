/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A drawing as an MJCF model, MuJoCo's own format, to run in MuJoCo on its own as a check on the built model (and later
// for export). Pure: the same drawing always gives the same text.
//
// - Each solid is a body with one geom of its shape. Bodies nest as the drawing's structure has them (lib/drawing.mjs):
//   a solid sits in the body of its parent joint's other solid, at its place in the drawing relative to it, and the
//   Ground is MuJoCo's world.
// - A joint in the tree is a joint element in the child's body: a hinge or a slide at the parent frame's point, along
//   the parent frame's axis (turned round when the tree reads the joint the other way, so its position still measures
//   the child frame's solid against the parent frame's); a ball; or nothing, for a fixed joint, which welds the child to
//   its parent.
// - A joint that closes a loop is an equality constraint: `connect` (the two solids held together at its point, free to
//   turn, which with the loop's other joints parallel makes a planar linkage exact) for a hinge or a ball, `weld` for a
//   fixed joint. MuJoCo has no equality for a slider, so a loop a slider closes is left open here.
// - A solid joined to nothing on the Ground has a free joint, and falls.
// - A servo is a position actuator on its joint: its stiffness, and its torque limited.
// - Contact is off: every geom has contype and conaffinity 0. Solids drawn touching at a joint would otherwise push each
//   other apart. Contact comes with its own milestone (docs/direction.md).
//
// Names: each body is named by its solid's id and each joint by its joint's id (`s3`, `j2`), so names in the drawing
// can be anything; the model builder knows them by the same ids.

import { structure, solidById, frameById, frameInWorld, groundId, massOf, poseOf } from './drawing.mjs';
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

    const body = (id, parent, depth, floating) => {
        const solid = solidById(drawing, id);
        const local = relativePose(poseOf(drawing, parent), poseOf(drawing, id));
        write(depth, `<body name="${id}" pos="${formatVector(local.position)}" quat="${formatVector(local.rotation)}">`);
        if (floating) write(depth + 1, '<freejoint/>');
        const link = built.parentJoint.get(id);
        if (link && !floating) {
            const { joint } = link;
            if (joint.kind !== 'fixed') {
                const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
                const pose = poseOf(drawing, id);
                const flipped = solidOf.get(joint.child) !== id;
                const axis = normalize(scale(directionToLocal(pose, anchor.axis), flipped ? -1 : 1));
                const attributes = [`name="${joint.id}"`, `type="${joint.kind === 'slider' ? 'slide' : joint.kind}"`, `pos="${formatVector(toLocal(pose, anchor.point))}"`];
                if (joint.kind !== 'ball') attributes.push(`axis="${formatVector(axis)}"`);
                // A ball's range is its most turn, from 0.
                if (joint.limits) attributes.push('limited="true"', `range="${formatNumber(joint.kind === 'ball' ? 0 : joint.limits.lower)} ${formatNumber(joint.limits.upper)}"`);
                attributes.push(`damping="${formatNumber(joint.damping ?? 0)}"`, `frictionloss="${formatNumber(joint.friction ?? 0)}"`, `armature="${formatNumber(joint.armature ?? 0)}"`);
                write(depth + 1, `<joint ${attributes.join(' ')}/>`);
            }
        }
        const geometry = mjcfGeometry(solid.shape, solid.size);
        write(depth + 1, `<geom name="${id}" type="${geometry.type}" size="${formatVector(geometry.size)}" mass="${formatNumber(massOf(solid))}" rgba="${rgba(solid.color)}" contype="0" conaffinity="0"/>`);
        for (const child of order(children.get(id) ?? [])) body(child, id, depth + 1, false);
        write(depth, '</body>');
    };

    write(0, `<mujoco model="${escape(drawing.name || 'Robot')}">`);
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
    if (loops.length) {
        write(1, '<equality>');
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
    if (servos.length) {
        write(1, '<actuator>');
        for (const { joint } of servos) {
            const limit = joint.drive.torqueLimit;
            write(2, `<position name="${joint.id}" joint="${joint.id}" kp="${formatNumber(joint.drive.gain)}" forcelimited="true" forcerange="${formatNumber(-limit)} ${formatNumber(limit)}"/>`);
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
