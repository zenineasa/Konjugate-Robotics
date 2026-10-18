/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Small vector and quaternion arithmetic for the drawing, its MJCF and its model, with no dependencies, so the same
// code runs in the window, in the importer (a worker of the host) and in tests. Vectors are [x, y, z] arrays in metres;
// quaternions are [w, x, y, z] arrays, unit length, as MuJoCo writes them. Space is z up, as in MuJoCo and Konjugate.

export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const subtract = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, factor) => [a[0] * factor, a[1] * factor, a[2] * factor];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const length = (a) => Math.hypot(a[0], a[1], a[2]);

export function normalize(a) {
    const size = length(a);
    if (!(size > 1e-12)) throw new Error('A direction cannot be of zero length.');
    return scale(a, 1 / size);
}

export const identity = Object.freeze([1, 0, 0, 0]);

export function multiply(a, b) {
    return [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    ];
}

export const conjugate = (q) => [q[0], -q[1], -q[2], -q[3]];

export function normalizeQuaternion(q) {
    const size = Math.hypot(q[0], q[1], q[2], q[3]);
    if (!(size > 1e-12)) return [...identity];
    // One sign for one rotation, so the same turn is always written the same way.
    const sign = q[0] < 0 ? -1 : 1;
    return q.map((value) => (sign * value) / size);
}

// `vector` turned by `q`.
export function rotate(q, vector) {
    const turned = multiply(multiply(q, [0, ...vector]), conjugate(q));
    return [turned[1], turned[2], turned[3]];
}

export function fromAxisAngle(axis, angle) {
    const unit = normalize(axis);
    const half = angle / 2;
    return normalizeQuaternion([Math.cos(half), ...scale(unit, Math.sin(half))]);
}

// The shortest turn taking direction `from` onto direction `to`.
export function between(from, to) {
    const a = normalize(from);
    const b = normalize(to);
    const cosine = dot(a, b);
    if (cosine > 1 - 1e-12) return [...identity];
    if (cosine < -1 + 1e-12) {
        // Half a turn about any axis square to `a`.
        const helper = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
        return fromAxisAngle(cross(a, helper), Math.PI);
    }
    return normalizeQuaternion([1 + cosine, ...cross(a, b)]);
}

// Roll, pitch and yaw (radians) of a turn, as Konjugate's pose convention reads them: yaw about z, then pitch about y,
// then roll about x (intrinsic z-y-x).
export function toRollPitchYaw(q) {
    const [w, x, y, z] = q;
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return [roll, pitch, yaw];
}

export function fromRollPitchYaw([roll, pitch, yaw]) {
    return normalizeQuaternion(multiply(multiply(fromAxisAngle([0, 0, 1], yaw), fromAxisAngle([0, 1, 0], pitch)), fromAxisAngle([1, 0, 0], roll)));
}

// A pose is { position, rotation }: where a solid's own frame is, and how it is turned, in the world.
// A point given in the pose's own frame, in the world.
export const toWorld = (pose, point) => add(pose.position, rotate(pose.rotation, point));
// A world point in the pose's own frame.
export const toLocal = (pose, point) => rotate(conjugate(pose.rotation), subtract(point, pose.position));
export const directionToWorld = (pose, direction) => rotate(pose.rotation, direction);
export const directionToLocal = (pose, direction) => rotate(conjugate(pose.rotation), direction);

// `child`'s pose in `parent`'s frame.
export function relativePose(parent, child) {
    return {
        position: toLocal(parent, child.position),
        rotation: normalizeQuaternion(multiply(conjugate(parent.rotation), child.rotation))
    };
}

// Numbers as MJCF writes them: short, with no exponent for ordinary sizes and no "-0".
export function formatNumber(value) {
    if (!Number.isFinite(value)) throw new Error(`${value} is not a finite number.`);
    const rounded = Math.abs(value) < 1e-12 ? 0 : Number(value.toPrecision(12));
    return String(rounded);
}

export const formatVector = (values) => values.map(formatNumber).join(' ');
