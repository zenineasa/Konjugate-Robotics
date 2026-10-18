/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Triangle meshes for showing a robot: read from the STL and OBJ files a model names, and made for MuJoCo's primitive
// shapes, as { position, normal, index } documents (the canvas's mesh document: flat arrays, no normals, as Pose
// Visualizer computes them). No dependencies.

// Collects triangles, merging corners at the same place (to a hundredth of a millimetre), so a mesh read from an STL
// file, which repeats each corner for every triangle meeting there, is about a sixth of the size.
export class MeshBuilder {
    constructor() {
        this.position = [];
        this.index = [];
        this.corners = new Map();
    }

    corner([x, y, z]) {
        const rounded = [x, y, z].map((value) => Math.round(value * 1e5) / 1e5 + 0);
        const key = rounded.join(',');
        let at = this.corners.get(key);
        if (at === undefined) {
            at = this.position.length / 3;
            this.position.push(...rounded);
            this.corners.set(key, at);
        }
        return at;
    }

    triangle(a, b, c) {
        const corners = [this.corner(a), this.corner(b), this.corner(c)];
        // A triangle merged to a line or a point has no face to draw.
        if (corners[0] !== corners[1] && corners[1] !== corners[2] && corners[0] !== corners[2]) this.index.push(...corners);
    }

    document() {
        return { position: this.position, normal: [], index: this.index };
    }
}

// The triangles of an STL file, binary or text, as arrays of three corners.
export function readStl(bytes, name = 'the mesh') {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const count = bytes.byteLength >= 84 ? view.getUint32(80, true) : -1;
    if (count >= 0 && bytes.byteLength === 84 + count * 50) {
        const triangles = [];
        for (let triangle = 0; triangle < count; triangle += 1) {
            const at = 84 + triangle * 50 + 12;
            const corner = (offset) => [view.getFloat32(at + offset, true), view.getFloat32(at + offset + 4, true), view.getFloat32(at + offset + 8, true)];
            triangles.push([corner(0), corner(12), corner(24)]);
        }
        return triangles;
    }
    const text = new TextDecoder().decode(bytes);
    if (!/^\s*solid/.test(text)) throw new Error(`${name} is neither a binary nor a text STL file.`);
    const corners = [...text.matchAll(/vertex\s+(\S+)\s+(\S+)\s+(\S+)/g)].map((match) => match.slice(1, 4).map(Number));
    if (corners.length % 3 || corners.some((corner) => corner.some((value) => !Number.isFinite(value)))) throw new Error(`${name} has a malformed facet.`);
    const triangles = [];
    for (let at = 0; at < corners.length; at += 3) triangles.push(corners.slice(at, at + 3));
    return triangles;
}

// The triangles of an OBJ file's faces (a face of more than three corners as a fan), its other records left out.
export function readObj(bytes, name = 'the mesh') {
    const vertices = [];
    const triangles = [];
    for (const line of new TextDecoder().decode(bytes).split(/\r?\n/)) {
        const [kind, ...fields] = line.trim().split(/\s+/);
        if (kind === 'v') vertices.push(fields.slice(0, 3).map(Number));
        else if (kind === 'f') {
            const corners = fields.map((field) => {
                const at = Number.parseInt(field.split('/')[0], 10);
                const vertex = vertices[at < 0 ? vertices.length + at : at - 1];
                if (!vertex) throw new Error(`${name} has a face naming a vertex it does not have.`);
                return vertex;
            });
            for (let at = 1; at + 1 < corners.length; at += 1) triangles.push([corners[0], corners[at], corners[at + 1]]);
        }
    }
    return triangles;
}

// ---- MuJoCo's primitive shapes, in their own frame (z along a capsule's or cylinder's length) -------------------

const segments = 16;
const rings = 8;

// A surface of revolution about z from (radius, height) pairs, bottom to top.
function revolve(profile) {
    const triangles = [];
    const point = ([radius, height], step) => {
        const angle = (2 * Math.PI * step) / segments;
        return [radius * Math.cos(angle), radius * Math.sin(angle), height];
    };
    for (let ring = 0; ring + 1 < profile.length; ring += 1) {
        for (let step = 0; step < segments; step += 1) {
            const a = point(profile[ring], step);
            const b = point(profile[ring], step + 1);
            const c = point(profile[ring + 1], step + 1);
            const d = point(profile[ring + 1], step);
            triangles.push([a, b, c], [a, c, d]);
        }
    }
    return triangles;
}

const hemisphere = (radius, centre, upward) => Array.from({ length: rings + 1 }, (_, ring) => {
    const angle = (Math.PI / 2) * (ring / rings);
    return upward
        ? [radius * Math.cos(angle), centre + radius * Math.sin(angle)]
        : [radius * Math.sin(angle), centre - radius * Math.cos(angle)];
});

export function primitiveTriangles(type, size) {
    const [a = 0, b = a, c = b] = size;
    if (type === 'box') {
        const corner = (x, y, z) => [x * a, y * b, z * c];
        const faces = [[[-1, -1, -1], [-1, 1, -1], [1, 1, -1], [1, -1, -1]], [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
            [[-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]], [[-1, 1, -1], [-1, 1, 1], [1, 1, 1], [1, 1, -1]],
            [[-1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1]], [[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]]];
        return faces.flatMap(([p, q, r, s]) => [[corner(...p), corner(...q), corner(...r)], [corner(...p), corner(...r), corner(...s)]]);
    }
    if (type === 'sphere') return revolve([...hemisphere(a, 0, false), ...hemisphere(a, 0, true).slice(1)]);
    if (type === 'ellipsoid') return revolve([...hemisphere(1, 0, false), ...hemisphere(1, 0, true).slice(1)]).map((triangle) => triangle.map(([x, y, z]) => [x * a, y * b, z * c]));
    if (type === 'capsule') return revolve([...hemisphere(a, -size[1], false), ...hemisphere(a, size[1], true)]);
    if (type === 'cylinder') return revolve([[0, -size[1]], [a, -size[1]], [a, size[1]], [0, size[1]]]);
    return null;
}
