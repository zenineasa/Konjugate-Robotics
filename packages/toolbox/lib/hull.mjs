/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A mesh's convex hull, as MuJoCo collides a mesh: the smallest convex solid about its corners. Found by quickhull:
// from a tetrahedron of four far apart corners, each face takes the corners beyond it, and the corner furthest beyond
// a face replaces every face it can see by a fan of faces from it to the edge of what it sees, until no corner lies
// beyond any face (by more than a hundred millionth of the mesh's size). Its triangles face outward.
//
// hullOf(points) gives { points (the hull's corners), triangles ([a, b, c] into points, counterclockwise seen from
// outside), normals and offsets (each triangle's plane: n · x = offset), edges ({ a, b, left, right }: the triangles
// either side, left the one that runs a to b), neighbours (each corner's corners along an edge), around (each corner's
// triangles), polygons ({ vertices in order counterclockwise from outside, normal }: triangles lying within MuJoCo's
// angle of each other merged, as MuJoCo makes its polygons), polygonsAround (each corner's polygons), radius (the
// furthest corner from the origin) }. Pure, no dependencies.

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const length = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => { const l = length(a); return [a[0] / l, a[1] / l, a[2] / l]; };

// Two triangles whose normals lie within this of each other (one less their dot product) are one polygon, as MuJoCo
// merges a hull's nearly flat faces (qhull's merging, which this follows only roughly: a face flat to a ten millionth
// is one polygon in both).
export const polygonTolerance = 1e-7;

// The corners' hull, as above; an Error where they span no volume. Corners on a face of it or along an edge between two
// faces are not its corners: it is found again without them.
export function hullOf(input) {
    let hull = quickhull(input);
    for (;;) {
        const flat = hull.points.map((_, v) => {
            const planes = [];
            for (const t of hull.around[v]) if (!planes.some((n) => 1 - dot(n, hull.normals[t]) < 1e-10)) planes.push(hull.normals[t]);
            return planes.length <= 2;
        });
        if (!flat.some(Boolean)) return hull;
        hull = quickhull(hull.points.filter((_, v) => !flat[v]));
    }
}

function quickhull(input) {
    const points = [];
    const seen = new Set();
    for (const point of input) {
        const key = point.join(',');
        if (!seen.has(key)) { seen.add(key); points.push(point.map(Number)); }
    }
    if (points.length < 4) throw new Error('A collision mesh needs at least four corners that are not all in one plane.');
    const span = Math.max(...[0, 1, 2].map((axis) => Math.max(...points.map((p) => p[axis])) - Math.min(...points.map((p) => p[axis]))));
    const tolerance = 1e-8 * span;

    // The first tetrahedron: the two corners furthest apart along an axis, the corner furthest from their line, and
    // the corner furthest from the plane of those three.
    let [i0, i1] = [0, 0];
    let best = -1;
    for (let axis = 0; axis < 3; axis++) {
        let low = 0, high = 0;
        points.forEach((p, i) => { if (p[axis] < points[low][axis]) low = i; if (p[axis] > points[high][axis]) high = i; });
        if (points[high][axis] - points[low][axis] > best) { best = points[high][axis] - points[low][axis]; [i0, i1] = [low, high]; }
    }
    const line = unit(sub(points[i1], points[i0]));
    let i2 = -1; best = tolerance;
    points.forEach((p, i) => { const w = sub(p, points[i0]); const d = length(sub(w, line.map((x) => x * dot(w, line)))); if (d > best) { best = d; i2 = i; } });
    if (i2 < 0) throw new Error('A collision mesh needs corners that are not all on one line.');
    const base = unit(cross(sub(points[i1], points[i0]), sub(points[i2], points[i0])));
    let i3 = -1; best = tolerance;
    points.forEach((p, i) => { const d = Math.abs(dot(sub(p, points[i0]), base)); if (d > best) { best = d; i3 = i; } });
    if (i3 < 0) throw new Error('A collision mesh needs corners that are not all in one plane.');

    const faces = [];
    const edgeFace = new Map(); // 'a,b' → the face that runs a to b
    const addFace = (a, b, c) => {
        const normal = unit(cross(sub(points[b], points[a]), sub(points[c], points[a])));
        const face = { vertices: [a, b, c], normal, offset: dot(normal, points[a]), outside: [], alive: true };
        faces.push(face);
        edgeFace.set(`${a},${b}`, face); edgeFace.set(`${b},${c}`, face); edgeFace.set(`${c},${a}`, face);
        return face;
    };
    const above = (face, p) => dot(face.normal, points[p]) - face.offset;
    if (dot(sub(points[i3], points[i0]), base) > 0) [i1, i2] = [i2, i1];
    const first = [addFace(i0, i1, i2), addFace(i0, i3, i1), addFace(i1, i3, i2), addFace(i2, i3, i0)];
    const assign = (candidates, newFaces) => {
        for (const p of candidates) {
            let far = null, farthest = tolerance;
            for (const face of newFaces) { const d = above(face, p); if (d > farthest) { farthest = d; far = face; } }
            if (far) far.outside.push(p);
        }
    };
    assign(points.map((_, i) => i).filter((i) => ![i0, i1, i2, i3].includes(i)), first);

    for (let at = 0; at < faces.length; at++) {
        const face = faces[at];
        if (!face.alive || !face.outside.length) continue;
        let apex = face.outside[0];
        for (const p of face.outside) if (above(face, p) > above(face, apex)) apex = p;
        // The faces the apex sees, from this one across their edges; the horizon, the edges between seen and unseen,
        // each as the seen face runs it.
        const visible = new Set([face]);
        const horizon = [];
        const visit = (current) => {
            const [a, b, c] = current.vertices;
            for (const [x, y] of [[a, b], [b, c], [c, a]]) {
                const other = edgeFace.get(`${y},${x}`);
                if (visible.has(other)) continue;
                if (above(other, apex) > tolerance) { visible.add(other); visit(other); }
                else horizon.push([x, y]);
            }
        };
        visit(face);
        const orphans = [];
        for (const seen of visible) {
            seen.alive = false;
            orphans.push(...seen.outside.filter((p) => p !== apex));
            const [a, b, c] = seen.vertices;
            for (const key of [`${a},${b}`, `${b},${c}`, `${c},${a}`]) if (edgeFace.get(key) === seen) edgeFace.delete(key);
        }
        assign(orphans, horizon.map(([x, y]) => addFace(x, y, apex)));
    }

    // The corners kept, renumbered.
    const kept = faces.filter(({ alive }) => alive);
    const used = [...new Set(kept.flatMap(({ vertices }) => vertices))].sort((a, b) => a - b);
    const number = new Map(used.map((old, i) => [old, i]));
    const hullPoints = used.map((old) => points[old]);
    const triangles = kept.map(({ vertices }) => vertices.map((v) => number.get(v)));
    return structureOf(hullPoints, triangles);
}

// The rest of a hull's structure, from its corners and outward triangles.
export function structureOf(points, triangles) {
    const normals = triangles.map(([a, b, c]) => unit(cross(sub(points[b], points[a]), sub(points[c], points[a]))));
    const offsets = triangles.map(([a], t) => dot(normals[t], points[a]));
    const runs = new Map();
    triangles.forEach(([a, b, c], t) => { runs.set(`${a},${b}`, t); runs.set(`${b},${c}`, t); runs.set(`${c},${a}`, t); });
    const edges = [];
    triangles.forEach(([a, b, c], t) => {
        for (const [x, y] of [[a, b], [b, c], [c, a]]) if (x < y) edges.push({ a: x, b: y, left: t, right: runs.get(`${y},${x}`) });
    });
    const neighbours = points.map(() => []);
    for (const { a, b } of edges) { neighbours[a].push(b); neighbours[b].push(a); }
    const around = points.map(() => []);
    triangles.forEach((triangle, t) => triangle.forEach((v) => around[v].push(t)));

    // MuJoCo's polygons: triangles grown from each not yet taken, across edges to triangles within its angle of the first.
    const polygonOf = triangles.map(() => -1);
    const polygons = [];
    triangles.forEach((_, seed) => {
        if (polygonOf[seed] >= 0) return;
        const members = [seed];
        polygonOf[seed] = polygons.length;
        for (let k = 0; k < members.length; k++) {
            const [a, b, c] = triangles[members[k]];
            for (const [x, y] of [[a, b], [b, c], [c, a]]) {
                const other = runs.get(`${y},${x}`);
                if (polygonOf[other] < 0 && 1 - dot(normals[other], normals[seed]) < polygonTolerance) { polygonOf[other] = polygons.length; members.push(other); }
            }
        }
        // Its outline: the edges of its triangles not shared within it, joined end to end.
        const next = new Map();
        for (const t of members) {
            const [a, b, c] = triangles[t];
            for (const [x, y] of [[a, b], [b, c], [c, a]]) if (polygonOf[runs.get(`${y},${x}`)] !== polygons.length) next.set(x, y);
        }
        const start = Math.min(...next.keys());
        const vertices = [start];
        for (let v = next.get(start); v !== start && vertices.length <= next.size; v = next.get(v)) vertices.push(v);
        let normal = [0, 0, 0];
        for (const t of members) {
            const [a, b, c] = triangles[t];
            const area = cross(sub(points[b], points[a]), sub(points[c], points[a]));
            normal = [normal[0] + area[0], normal[1] + area[1], normal[2] + area[2]];
        }
        polygons.push({ vertices, normal: unit(normal) });
    });
    const polygonsAround = points.map(() => []);
    polygons.forEach(({ vertices }, p) => vertices.forEach((v) => polygonsAround[v].push(p)));
    const radius = Math.max(...points.map(length));
    return { points, triangles, normals, offsets, edges, neighbours, around, polygons, polygonsAround, radius };
}
