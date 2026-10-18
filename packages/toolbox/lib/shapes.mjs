/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The solid shapes a drawing offers, each one of MuJoCo's own geometric primitives, so its mass and inertia are exact
// and no mesh is needed to simulate it. Each is centred on its solid's own frame; a cylinder and a capsule lie along
// their own z. Sizes are in metres:
// - box: { x, y, z }, its full lengths along each axis;
// - cylinder: { radius, length };
// - sphere: { radius };
// - capsule: { radius, length }, the length of its straight middle, without the two half spheres.
// Also: how each is written in MJCF, its volume, its mesh for Pose Visualizer and the canvas, and the points on its
// surface a click snaps to.

import { add, length, normalize, scale, subtract } from './math.mjs';

export const shapeKinds = Object.freeze(['box', 'cylinder', 'sphere', 'capsule']);

export const shapeLabels = Object.freeze({ box: 'Box', cylinder: 'Cylinder', sphere: 'Sphere', capsule: 'Capsule' });

// A new solid's size, a little smaller than a hand.
export function defaultSize(kind) {
    switch (kind) {
    case 'box': return { x: 0.1, y: 0.1, z: 0.1 };
    case 'cylinder': return { radius: 0.03, length: 0.2 };
    case 'sphere': return { radius: 0.05 };
    case 'capsule': return { radius: 0.02, length: 0.2 };
    default: throw new Error(`There is no shape "${kind}".`);
    }
}

// The size fields of each shape, in the order the window shows them.
export const sizeFields = Object.freeze({
    box: Object.freeze([['x', 'Length (x)'], ['y', 'Width (y)'], ['z', 'Height (z)']]),
    cylinder: Object.freeze([['radius', 'Radius'], ['length', 'Length']]),
    sphere: Object.freeze([['radius', 'Radius']]),
    capsule: Object.freeze([['radius', 'Radius'], ['length', 'Length of its middle']])
});

// The reasons a size is not one a solid can have, or none.
export function sizeProblems(kind, size) {
    const fields = sizeFields[kind];
    if (!fields) return [`There is no shape "${kind}".`];
    return fields.filter(([field]) => !(Number.isFinite(size?.[field]) && size[field] > 0)).map(([, label]) => `${label} must be more than zero.`);
}

export function volume(kind, size) {
    switch (kind) {
    case 'box': return size.x * size.y * size.z;
    case 'cylinder': return Math.PI * size.radius ** 2 * size.length;
    case 'sphere': return (4 / 3) * Math.PI * size.radius ** 3;
    case 'capsule': return Math.PI * size.radius ** 2 * size.length + (4 / 3) * Math.PI * size.radius ** 3;
    default: throw new Error(`There is no shape "${kind}".`);
    }
}

// MuJoCo's geom type and size attribute: half lengths for a box, radius and half length for a cylinder or capsule.
export function mjcfGeometry(kind, size) {
    switch (kind) {
    case 'box': return { type: 'box', size: [size.x / 2, size.y / 2, size.z / 2] };
    case 'cylinder': return { type: 'cylinder', size: [size.radius, size.length / 2] };
    case 'sphere': return { type: 'sphere', size: [size.radius] };
    case 'capsule': return { type: 'capsule', size: [size.radius, size.length / 2] };
    default: throw new Error(`There is no shape "${kind}".`);
    }
}

// The half extents of the box that holds the shape, in its own frame.
export function halfExtents(kind, size) {
    switch (kind) {
    case 'box': return [size.x / 2, size.y / 2, size.z / 2];
    case 'cylinder': return [size.radius, size.radius, size.length / 2];
    case 'sphere': return [size.radius, size.radius, size.radius];
    case 'capsule': return [size.radius, size.radius, size.length / 2 + size.radius];
    default: throw new Error(`There is no shape "${kind}".`);
    }
}

// The direction straight out of the shape's surface at a point on it (or near it), in its own frame.
export function surfaceNormal(kind, size, point) {
    switch (kind) {
    case 'box': {
        // The face the point is nearest, counted relative to the face's distance from the centre.
        const half = halfExtents(kind, size);
        let best = 0;
        let bestRatio = -Infinity;
        for (let axis = 0; axis < 3; axis += 1) {
            const ratio = Math.abs(point[axis]) / half[axis];
            if (ratio > bestRatio) { bestRatio = ratio; best = axis; }
        }
        const normal = [0, 0, 0];
        normal[best] = point[best] < 0 ? -1 : 1;
        return normal;
    }
    case 'cylinder': {
        const radial = Math.hypot(point[0], point[1]);
        const end = Math.abs(point[2]) / (size.length / 2);
        if (end >= radial / size.radius || radial < 1e-9) return [0, 0, point[2] < 0 ? -1 : 1];
        return [point[0] / radial, point[1] / radial, 0];
    }
    case 'sphere':
        return length(point) < 1e-9 ? [0, 0, 1] : normalize(point);
    case 'capsule': {
        const half = size.length / 2;
        const centre = [0, 0, Math.max(-half, Math.min(half, point[2]))];
        const outward = subtract(point, centre);
        return length(outward) < 1e-9 ? [1, 0, 0] : normalize(outward);
    }
    default: throw new Error(`There is no shape "${kind}".`);
    }
}

// The points on the shape a click near them snaps to, each with its normal: face centres and edge middles of a box,
// end centres and the rim's quarter points of a cylinder, the six poles of a sphere, the ends and the middle's four
// sides of a capsule.
export function snapPoints(kind, size) {
    const points = [];
    const push = (point, normal) => points.push({ point, normal });
    const half = halfExtents(kind, size);
    switch (kind) {
    case 'box':
        for (let axis = 0; axis < 3; axis += 1) {
            for (const sign of [-1, 1]) {
                const centre = [0, 0, 0];
                centre[axis] = sign * half[axis];
                const normal = [0, 0, 0];
                normal[axis] = sign;
                push(centre, normal);
                // The middles of the face's four edges.
                for (let other = 0; other < 3; other += 1) {
                    if (other === axis) continue;
                    for (const edgeSign of [-1, 1]) {
                        const middle = [...centre];
                        middle[other] = edgeSign * half[other];
                        push(middle, normal);
                    }
                }
            }
        }
        break;
    case 'cylinder':
        for (const sign of [-1, 1]) push([0, 0, sign * half[2]], [0, 0, sign]);
        for (const [x, y] of [[1, 0], [0, 1], [-1, 0], [0, -1]]) push([x * size.radius, y * size.radius, 0], [x, y, 0]);
        break;
    case 'sphere':
        for (let axis = 0; axis < 3; axis += 1) {
            for (const sign of [-1, 1]) {
                const normal = [0, 0, 0];
                normal[axis] = sign;
                push(scale(normal, size.radius), normal);
            }
        }
        break;
    case 'capsule':
        for (const sign of [-1, 1]) push([0, 0, sign * half[2]], [0, 0, sign]);
        for (const [x, y] of [[1, 0], [0, 1], [-1, 0], [0, -1]]) push([x * size.radius, y * size.radius, 0], [x, y, 0]);
        break;
    default: throw new Error(`There is no shape "${kind}".`);
    }
    return points;
}

// The snap point nearest `point` within `tolerance` metres, or null.
export function nearestSnap(kind, size, point, tolerance) {
    let best = null;
    let bestDistance = tolerance;
    for (const candidate of snapPoints(kind, size)) {
        const distance = length(subtract(candidate.point, point));
        if (distance <= bestDistance) { best = candidate; bestDistance = distance; }
    }
    return best;
}

// A triangle mesh of the shape in its own frame, as Konjugate keeps a node's appearance part: { position, normal,
// index }, flat arrays, metres. Flat shaded for a box; smooth elsewhere.
export function shapeMesh(kind, size, segments = 24) {
    const position = [];
    const normal = [];
    const index = [];
    const vertex = (point, direction) => {
        position.push(...point);
        normal.push(...direction);
        return position.length / 3 - 1;
    };
    const quad = (a, b, c, d) => index.push(a, b, c, a, c, d);
    if (kind === 'box') {
        const half = halfExtents(kind, size);
        const faces = [
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [[-1, 0, 0], [0, 0, 1], [0, 1, 0]],
            [[0, 1, 0], [0, 0, 1], [1, 0, 0]], [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
            [[0, 0, 1], [1, 0, 0], [0, 1, 0]], [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
        ];
        for (const [outward, u, v] of faces) {
            const centre = outward.map((value, axis) => value * half[axis]);
            const corner = (su, sv) => add(centre, add(u.map((value, axis) => su * value * half[axis]), v.map((value, axis) => sv * value * half[axis])));
            quad(vertex(corner(-1, -1), outward), vertex(corner(1, -1), outward), vertex(corner(1, 1), outward), vertex(corner(-1, 1), outward));
        }
        return { position, normal, index };
    }
    if (kind === 'sphere') {
        const rings = Math.max(6, Math.round(segments / 2));
        for (let ring = 0; ring <= rings; ring += 1) {
            const polar = (Math.PI * ring) / rings;
            for (let step = 0; step <= segments; step += 1) {
                const azimuth = (2 * Math.PI * step) / segments;
                const direction = [Math.sin(polar) * Math.cos(azimuth), Math.sin(polar) * Math.sin(azimuth), Math.cos(polar)];
                vertex(scale(direction, size.radius), direction);
            }
        }
        for (let ring = 0; ring < rings; ring += 1) {
            for (let step = 0; step < segments; step += 1) {
                const a = ring * (segments + 1) + step;
                const b = a + segments + 1;
                index.push(a, b, a + 1, a + 1, b, b + 1);
            }
        }
        return { position, normal, index };
    }
    if (kind === 'cylinder' || kind === 'capsule') {
        const half = size.length / 2;
        // The side, from bottom to top.
        const side = [];
        for (const z of [-half, half]) {
            const row = [];
            for (let step = 0; step <= segments; step += 1) {
                const azimuth = (2 * Math.PI * step) / segments;
                const direction = [Math.cos(azimuth), Math.sin(azimuth), 0];
                row.push(vertex([direction[0] * size.radius, direction[1] * size.radius, z], direction));
            }
            side.push(row);
        }
        for (let step = 0; step < segments; step += 1) quad(side[0][step], side[0][step + 1], side[1][step + 1], side[1][step]);
        for (const sign of [-1, 1]) {
            if (kind === 'cylinder') {
                // A flat end.
                const outward = [0, 0, sign];
                const centre = vertex([0, 0, sign * half], outward);
                const rim = [];
                for (let step = 0; step <= segments; step += 1) {
                    const azimuth = (2 * Math.PI * step) / segments;
                    rim.push(vertex([Math.cos(azimuth) * size.radius, Math.sin(azimuth) * size.radius, sign * half], outward));
                }
                for (let step = 0; step < segments; step += 1) {
                    if (sign > 0) index.push(centre, rim[step], rim[step + 1]);
                    else index.push(centre, rim[step + 1], rim[step]);
                }
            } else {
                // A half sphere.
                const rings = Math.max(3, Math.round(segments / 4));
                const rows = [];
                for (let ring = 0; ring <= rings; ring += 1) {
                    const polar = (Math.PI / 2) * (ring / rings);
                    const row = [];
                    for (let step = 0; step <= segments; step += 1) {
                        const azimuth = (2 * Math.PI * step) / segments;
                        const direction = [Math.cos(polar) * Math.cos(azimuth), Math.cos(polar) * Math.sin(azimuth), sign * Math.sin(polar)];
                        row.push(vertex(add([0, 0, sign * half], scale(direction, size.radius)), direction));
                    }
                    rows.push(row);
                }
                for (let ring = 0; ring < rings; ring += 1) {
                    for (let step = 0; step < segments; step += 1) {
                        const [a, b, c, d] = [rows[ring][step], rows[ring][step + 1], rows[ring + 1][step + 1], rows[ring + 1][step]];
                        if (sign > 0) quad(a, b, c, d); else quad(a, d, c, b);
                    }
                }
            }
        }
        return { position, normal, index };
    }
    throw new Error(`There is no shape "${kind}".`);
}
