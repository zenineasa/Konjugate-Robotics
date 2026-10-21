/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The drawing posed: each hinge turned and each slider moved by a value (radians or metres from where it is drawn),
// and everything it carries with it, as the built model would place them with those joint positions. For the jog
// panel, which poses the robot to set its setpoints and record keyframes. A joint that closes a loop is left open:
// posing follows the tree, so a loop's two ends may part while posing, and Build's loop holds them together again.
// Pure, no dependencies.

import { frameById, frameInWorld, groundId, poseOf, structure, treeOrder } from './drawing.mjs';
import { add, conjugate, fromAxisAngle, multiply, normalize, normalizeQuaternion, rotate, scale, subtract } from './math.mjs';

// `values`: { [jointId]: value }, for hinges and sliders; others stay as drawn. Returns a new drawing.
export function posedDrawing(drawing, values) {
    const built = structure(drawing);
    const posed = structuredClone(drawing);
    const placed = new Map([[groundId, poseOf(drawing, groundId)]]);
    for (const { id } of treeOrder(drawing, built)) {
        const link = built.parentJoint.get(id);
        const rest = poseOf(drawing, id);
        if (!link) { placed.set(id, rest); continue; }
        const { joint, parent } = link;
        const parentRest = poseOf(drawing, parent);
        const parentNow = placed.get(parent);
        // Where the child sits relative to its parent as drawn, then the joint's own motion in the parent's frame.
        let offset = rotate(conjugate(parentRest.rotation), subtract(rest.position, parentRest.position));
        let turn = normalizeQuaternion(multiply(conjugate(parentRest.rotation), rest.rotation));
        const value = Number(values?.[joint.id]) || 0;
        if (value && (joint.kind === 'hinge' || joint.kind === 'slider')) {
            const anchor = frameInWorld(drawing, frameById(drawing, joint.parent));
            // Read the other way round (its first frame on the solid further from the Ground), it turns the other way.
            const flipped = frameById(drawing, joint.child).solid !== id;
            const point = rotate(conjugate(parentRest.rotation), subtract(anchor.point, parentRest.position));
            const axis = scale(normalize(rotate(conjugate(parentRest.rotation), anchor.axis)), flipped ? -1 : 1);
            if (joint.kind === 'hinge') {
                const spin = fromAxisAngle(axis, value);
                offset = add(point, rotate(spin, subtract(offset, point)));
                turn = normalizeQuaternion(multiply(spin, turn));
            } else {
                offset = add(offset, scale(axis, value));
            }
        }
        const pose = { position: add(parentNow.position, rotate(parentNow.rotation, offset)), rotation: normalizeQuaternion(multiply(parentNow.rotation, turn)) };
        placed.set(id, pose);
        const solid = posed.solids.find((candidate) => candidate.id === id);
        solid.position = pose.position;
        solid.rotation = pose.rotation;
    }
    return posed;
}

// A driven joint's jog range: its limits, or a turn either way for a hinge and 10 cm either way for a slider.
export function jogRange(joint) {
    if (joint.limits) return { lower: joint.limits.lower, upper: joint.limits.upper };
    return joint.kind === 'slider' ? { lower: -0.1, upper: 0.1 } : { lower: -Math.PI, upper: Math.PI };
}
