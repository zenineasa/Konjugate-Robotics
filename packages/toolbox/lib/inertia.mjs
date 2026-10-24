/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Mass and inertia as MuJoCo's compiler works them out, for a body whose model gives them only through its geoms, and
// a body's inertia turned to its principal axes. An inertia is { mass, centre, tensor }: the centre of mass and the
// 3 by 3 inertia tensor about it (row-major), in the frame it is given in. Pure, no dependencies but frames.mjs.

import { fromMatrix, rotate } from './frames.mjs';

const zero = () => [0, 0, 0, 0, 0, 0, 0, 0, 0];
const diagonal = ([a, b, c]) => [a, 0, 0, 0, b, 0, 0, 0, c];

// A primitive geom's volume and its inertia per kilogram about its centre, along its own axes, from MuJoCo's sizes
// (half sizes; a capsule's and a cylinder's second size is half its length, along z), as mjCGeom works them out.
export function primitiveInertia(type, size) {
    const [a = 0, b = a, c = b] = size;
    if (type === 'sphere') return { volume: (4 / 3) * Math.PI * a ** 3, unit: Array(3).fill((2 / 5) * a * a) };
    if (type === 'ellipsoid') return { volume: (4 / 3) * Math.PI * a * b * c, unit: [(b * b + c * c) / 5, (a * a + c * c) / 5, (a * a + b * b) / 5] };
    if (type === 'box') return { volume: 8 * a * b * c, unit: [(b * b + c * c) / 3, (a * a + c * c) / 3, (a * a + b * b) / 3] };
    if (type === 'cylinder') {
        const length = 2 * size[1];
        return { volume: Math.PI * a * a * length, unit: [(3 * a * a + length * length) / 12, (3 * a * a + length * length) / 12, (a * a) / 2] };
    }
    if (type === 'capsule') {
        // A cylinder and two half spheres, each its share of the mass as its share of the volume (MuJoCo's own sums).
        const length = 2 * size[1];
        const sphere = (4 / 3) * Math.PI * a ** 3;
        const cylinder = Math.PI * a * a * length;
        const volume = sphere + cylinder;
        const across = (cylinder * (length * length / 12 + a * a / 4) + sphere * ((2 * a * a) / 5 + (length * length) / 4 + (3 * length * a) / 8)) / volume;
        return { volume, unit: [across, across, (a * a * (0.4 * sphere + 0.5 * cylinder)) / volume] };
    }
    return null;
}

// A closed triangle mesh's volume, centre and inertia tensor per kilogram about that centre, by summing the signed
// tetrahedra its triangles make with the origin (exact for a closed mesh, whatever its shape). `triangles` are arrays
// of three corners. MuJoCo's default for a mesh may differ (it may work from the mesh's convex hull), so an import
// says when a body's inertia comes from a mesh.
export function meshInertia(triangles) {
    let volume = 0;
    const first = [0, 0, 0];
    // Second moments: ∫ x x, y y, z z, x y, x z, y z over the volume.
    const second = [0, 0, 0, 0, 0, 0];
    for (const [p, q, r] of triangles) {
        const v = (p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) + p[2] * (q[0] * r[1] - q[1] * r[0])) / 6;
        volume += v;
        for (let axis = 0; axis < 3; axis += 1) first[axis] += (v * (p[axis] + q[axis] + r[axis])) / 4;
        const product = (i, j) => (v / 20) * (2 * (p[i] * p[j] + q[i] * q[j] + r[i] * r[j])
            + p[i] * q[j] + q[i] * p[j] + p[i] * r[j] + r[i] * p[j] + q[i] * r[j] + r[i] * q[j]);
        second[0] += product(0, 0); second[1] += product(1, 1); second[2] += product(2, 2);
        second[3] += product(0, 1); second[4] += product(0, 2); second[5] += product(1, 2);
    }
    if (!(Math.abs(volume) > 1e-15)) return null;
    // A mesh wound the other way has a negative volume; its integrals change sign with it.
    const sign = Math.sign(volume);
    volume *= sign;
    const centre = first.map((value) => (value * sign) / volume);
    const [xx, yy, zz, xy, xz, yz] = second.map((value) => (value * sign) / volume);
    // About the centre, per unit mass: I = ∫ (r·r) 1 - r rᵀ, shifted from the origin to the centre.
    const [cx, cy, cz] = centre;
    const sxx = xx - cx * cx;
    const syy = yy - cy * cy;
    const szz = zz - cz * cz;
    const sxy = xy - cx * cy;
    const sxz = xz - cx * cz;
    const syz = yz - cy * cz;
    return { volume, centre, tensor: [syy + szz, -sxy, -sxz, -sxy, sxx + szz, -syz, -sxz, -syz, sxx + syy] };
}

// A mass `mass` with its unit inertia tensor `unitTensor` about its centre in a frame turned by `quaternion` and at
// `position`, expressed in the outer frame: { mass, centre, tensor }.
export function placed(mass, unitTensor, { position, quaternion }) {
    const R = rotationMatrix(quaternion);
    const turned = multiplyMatrices(multiplyMatrices(R, unitTensor), transpose(R)).map((value) => value * mass);
    return { mass, centre: [...position], tensor: turned };
}

// Several inertias in one frame, as one: their masses summed, about their joint centre (by the parallel axis theorem).
export function combine(parts) {
    const mass = parts.reduce((sum, part) => sum + part.mass, 0);
    if (!(mass > 0)) return { mass: 0, centre: [0, 0, 0], tensor: zero() };
    const centre = [0, 1, 2].map((axis) => parts.reduce((sum, part) => sum + part.mass * part.centre[axis], 0) / mass);
    const tensor = zero();
    for (const part of parts) {
        const d = part.centre.map((value, axis) => value - centre[axis]);
        const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        for (let i = 0; i < 3; i += 1) {
            for (let j = 0; j < 3; j += 1) tensor[3 * i + j] += part.tensor[3 * i + j] + part.mass * ((i === j ? dd : 0) - d[i] * d[j]);
        }
    }
    return { mass, centre, tensor };
}

// A symmetric tensor's principal moments and the turn to its principal axes (a quaternion [w, x, y, z], its columns
// the axes, right-handed), largest moment first as MuJoCo orders them, by Jacobi rotations.
export function principal(tensor) {
    const a = [[tensor[0], tensor[1], tensor[2]], [tensor[3], tensor[4], tensor[5]], [tensor[6], tensor[7], tensor[8]]];
    const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 64; sweep += 1) {
        const off = Math.abs(a[0][1]) + Math.abs(a[0][2]) + Math.abs(a[1][2]);
        if (off < 1e-300 || off < 1e-15 * (Math.abs(a[0][0]) + Math.abs(a[1][1]) + Math.abs(a[2][2]))) break;
        for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
            if (Math.abs(a[p][q]) < 1e-300) continue;
            const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
            const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
            const c = 1 / Math.sqrt(t * t + 1);
            const s = t * c;
            for (let k = 0; k < 3; k += 1) {
                const akp = a[k][p];
                const akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for (let k = 0; k < 3; k += 1) {
                const apk = a[p][k];
                const aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for (let k = 0; k < 3; k += 1) {
                const vkp = v[k][p];
                const vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    const order = [0, 1, 2].sort((i, j) => a[j][j] - a[i][i]);
    const columns = order.map((index) => [v[0][index], v[1][index], v[2][index]]);
    // Right-handed: the third axis the cross product of the first two.
    const [x, y] = columns;
    const z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
    const moments = order.map((index) => a[index][index]);
    return { moments, quaternion: fromMatrix([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]) };
}

export const diagonalTensor = diagonal;

// MuJoCo's fullinertia, "ixx iyy izz ixy ixz iyz", as a tensor.
export const fullTensor = ([xx, yy, zz, xy, xz, yz]) => [xx, xy, xz, xy, yy, yz, xz, yz, zz];

function rotationMatrix(quaternion) {
    const columns = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((axis) => rotate(quaternion, axis));
    return [columns[0][0], columns[1][0], columns[2][0], columns[0][1], columns[1][1], columns[2][1], columns[0][2], columns[1][2], columns[2][2]];
}
function multiplyMatrices(a, b) {
    const out = zero();
    for (let i = 0; i < 3; i += 1) for (let j = 0; j < 3; j += 1) for (let k = 0; k < 3; k += 1) out[3 * i + j] += a[3 * i + k] * b[3 * k + j];
    return out;
}
const transpose = (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
