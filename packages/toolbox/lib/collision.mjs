/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Where two shapes touch, as MuJoCo 3.16 finds it (its engine_collision_primitive.c and engine_collision_box.c, and
// mju_makeFrame), for each pair of shapes the toolbox collides: a plane (the floor) with a sphere, a capsule, a
// cylinder or a box; a sphere with a sphere, a capsule, a cylinder or a box; a capsule with a capsule or a box; a box
// with a box. A cylinder against a capsule, a cylinder or a box, which MuJoCo finds by its iterative convex search, is
// found here exactly (see "the largest gap" below), with points of its own where it lies flat; and a mesh, as its
// convex hull, against the floor (MuJoCo's own rule) and against a sphere or a capsule (exactly, by the largest gap).
// Each function gives a fixed number of contact slots, as many as the pair can have at once, each { active, dist, pos,
// frame }: whether MuJoCo would have that contact now (in the order its function makes them), how far apart the
// shapes are there (negative when they overlap), the point (midway through the overlap) and the frame (the normal,
// from the first shape to the second, then two tangents).
//
// Written once over an algebra, so the model builder writes them as equations (lib/modelBuilder.mjs's contact
// relationships) and the tests work them out in numbers, both from the same steps. An algebra has: num, add, sub, mul,
// div, neg, sqrt, abs, min, max; comparisons gt, lt, ge, le giving conditions, combined by and, or, not; pick(condition,
// a, b); and name(tag, value), which an algebra of equations makes a named definition. Where MuJoCo divides only in
// the branch it takes, the other branch here divides by something safe, so equations evaluated whole stay finite.
//
// A shape: { kind: 'plane' | 'sphere' | 'capsule' | 'cylinder' | 'box' | 'mesh', pos [3], mat [9] (rows, as MuJoCo's
// xmat), size [3] (MuJoCo's: a sphere's radius; a capsule's or a cylinder's radius and half length; a box's half
// sizes) }; a mesh's also hull (lib/hull.mjs's, in its frame) and centre. Pure, no dependencies.

export const minimumValue = 1e-15; // MuJoCo's mjMINVAL

// Numbers: conditions are booleans.
export const numbers = Object.freeze({
    num: (value) => value,
    add: (...values) => values.reduce((sum, value) => sum + value, 0),
    sub: (a, b) => a - b,
    mul: (...values) => values.reduce((product, value) => product * value, 1),
    div: (a, b) => a / b,
    neg: (a) => -a,
    sqrt: Math.sqrt,
    abs: Math.abs,
    min: Math.min,
    max: Math.max,
    gt: (a, b) => a > b,
    lt: (a, b) => a < b,
    ge: (a, b) => a >= b,
    le: (a, b) => a <= b,
    and: (...conditions) => conditions.every(Boolean),
    or: (...conditions) => conditions.some(Boolean),
    not: (condition) => !condition,
    pick: (condition, a, b) => (condition ? a : b),
    name: (tag, value) => value,
    always: true,
    never: false
});

// The pairs collided, by their kinds in MuJoCo's order (plane, sphere, capsule, cylinder, box), and how many contact
// slots each has.
export const geomOrder = ['plane', 'sphere', 'capsule', 'cylinder', 'box', 'mesh'];
export const contactSlots = Object.freeze({
    'plane|sphere': 1, 'plane|capsule': 2, 'plane|cylinder': 4, 'plane|box': 4,
    'sphere|sphere': 1, 'sphere|capsule': 1, 'sphere|cylinder': 1, 'sphere|box': 1, 'capsule|capsule': 2, 'capsule|cylinder': 2, 'capsule|box': 2, 'cylinder|cylinder': 4, 'cylinder|box': 4, 'box|box': 8,
    'plane|mesh': 4, 'sphere|mesh': 1, 'capsule|mesh': 2
});
export const pairKey = (first, second) => `${first}|${second}`;
export const collides = (first, second) => pairKey(...[first, second].sort((a, b) => geomOrder.indexOf(a) - geomOrder.indexOf(b))) in contactSlots;

// The contacts between `first` and `second` (already in MuJoCo's order: the first's kind no later than the second's),
// within `margin`. Given `reach` (how near two shapes must come to touch: at least the margin they touch within),
// none, and nothing worked out, while the spheres about the two lie further apart than that, as MuJoCo's own first
// test of a pair: as equations, the pair's contacts are a choice the engine works out only when taken.
export function collide(A, first, second, margin, { reach = null } = {}) {
    const key = pairKey(first.kind, second.kind);
    const found = {
        'plane|sphere': planeSphere, 'plane|capsule': planeCapsule, 'plane|cylinder': planeCylinder, 'plane|box': planeBox,
        'sphere|sphere': sphereSphere, 'sphere|capsule': sphereCapsule, 'sphere|cylinder': sphereCylinder, 'sphere|box': sphereBox,
        'capsule|capsule': capsuleCapsule, 'capsule|cylinder': capsuleCylinder,
        'capsule|box': capsuleBox, 'cylinder|cylinder': cylinderCylinder, 'cylinder|box': cylinderBox, 'box|box': boxBox,
        'plane|mesh': planeMesh, 'sphere|mesh': sphereMesh, 'capsule|mesh': capsuleMesh
    }[key];
    if (!found) throw new Error(`MuJoCo's ${first.kind} and ${second.kind} collision is not built yet.`);
    const contacts = () => found(A, first, second, A.num(margin)).map((slot, index) => ({ ...slot, frame: makeFrame(A, slot.frame, `f${index + 1}`) }));
    if (reach === null || first.kind === 'plane') return contacts();
    const radius = (shape) => ({ sphere: shape.size[0], capsule: shape.size[0] + shape.size[1], cylinder: Math.hypot(shape.size[0], shape.size[1]), box: Math.hypot(...shape.size), mesh: shape.hull?.radius })[shape.kind];
    const apart = radius(first) + radius(second) + reach;
    const between = v3.sub(A, second.pos, first.pos);
    const near = A.name('near', A.le(v3.dot(A, between, between), apart * apart));
    const none = () => ({ active: A.never, dist: A.num(0), pos: [A.num(0), A.num(0), A.num(0)], frame: Array.from({ length: 9 }, () => A.num(0)) });
    // In numbers, the choice is made here.
    if (typeof near === 'boolean') return near ? contacts() : Array.from({ length: contactSlots[key] }, none);
    return contacts().map((slot) => ({
        active: A.and(near, slot.active),
        dist: A.pick(near, slot.dist, A.num(0)),
        pos: slot.pos.map((value) => A.pick(near, value, A.num(0))),
        frame: slot.frame.map((value) => A.pick(near, value, A.num(0)))
    }));
}

// ---- vectors over an algebra ------------------------------------------------------------------------------------
const v3 = {
    add: (A, a, b) => [0, 1, 2].map((i) => A.add(a[i], b[i])),
    sub: (A, a, b) => [0, 1, 2].map((i) => A.sub(a[i], b[i])),
    scale: (A, a, s) => a.map((value) => A.mul(value, s)),
    dot: (A, a, b) => A.add(A.mul(a[0], b[0]), A.mul(a[1], b[1]), A.mul(a[2], b[2])),
    cross: (A, a, b) => [
        A.sub(A.mul(a[1], b[2]), A.mul(a[2], b[1])), A.sub(A.mul(a[2], b[0]), A.mul(a[0], b[2])), A.sub(A.mul(a[0], b[1]), A.mul(a[1], b[0]))
    ],
    pick: (A, condition, a, b) => [0, 1, 2].map((i) => A.pick(condition, a[i], b[i])),
    name: (A, tag, a) => a.map((value, i) => A.name(`${tag}${i + 1}`, value)),
    // A matrix (rows) times a vector, and its transpose times one.
    mat: (A, m, v) => [0, 1, 2].map((i) => A.add(A.mul(m[3 * i], v[0]), A.mul(m[3 * i + 1], v[1]), A.mul(m[3 * i + 2], v[2]))),
    matT: (A, m, v) => [0, 1, 2].map((i) => A.add(A.mul(m[i], v[0]), A.mul(m[3 + i], v[1]), A.mul(m[6 + i], v[2])))
};
const column = (m, i) => [m[i], m[3 + i], m[6 + i]];
const clip = (A, value, low, high) => A.min(high, A.max(low, value));

// mju_normalize3: the vector over its length, or (1, 0, 0) when that is below mjMINVAL; and the length.
function normalize(A, tag, a) {
    const length = A.name(`${tag}n`, A.sqrt(v3.dot(A, a, a)));
    const small = A.lt(length, minimumValue);
    const safe = A.pick(small, A.num(1), length);
    return { length, unit: v3.name(A, `${tag}u`, [0, 1, 2].map((i) => A.pick(small, A.num(i === 0 ? 1 : 0), A.div(a[i], safe)))) };
}

// mju_makeFrame: the normal normalized; the first tangent as given, or (0, 1, 0) (or (0, 0, 1) when the normal is
// nearly along y), made square to the normal; the second their cross product.
function makeFrame(A, frame, tag) {
    const normal = frame.slice(0, 3);
    const given = frame.slice(3, 6);
    const unset = A.lt(v3.dot(A, given, given), 0.25);
    const fallback = [A.num(0), A.pick(A.and(A.lt(normal[1], 0.5), A.gt(normal[1], -0.5)), A.num(1), A.num(0)), A.pick(A.and(A.lt(normal[1], 0.5), A.gt(normal[1], -0.5)), A.num(0), A.num(1))];
    const start = v3.name(A, `${tag}s`, v3.pick(A, unset, fallback, given));
    const along = v3.dot(A, normal, start);
    const tangent = normalize(A, `${tag}t`, v3.sub(A, start, v3.scale(A, normal, along))).unit;
    return [...normal, ...tangent, ...v3.cross(A, normal, tangent)];
}

const noTangent = (A) => [A.num(0), A.num(0), A.num(0)];

// ---- plane against ... -------------------------------------------------------------------------------------------
// mjraw_PlaneSphere: the plane's z axis the normal; the sphere's centre's height over it less its radius.
function planeSphereAt(A, plane, centre, radius, margin, tag) {
    const normal = column(plane.mat, 2);
    const height = A.name(`${tag}h`, v3.dot(A, v3.sub(A, centre, plane.pos), normal));
    const dist = A.name(`${tag}d`, A.sub(height, radius));
    const pos = v3.name(A, `${tag}p`, v3.add(A, centre, v3.scale(A, normal, A.sub(A.mul(-0.5, dist), radius))));
    return { active: A.le(height, A.add(margin, radius)), dist, pos, frame: [...normal, ...noTangent(A)] };
}

function planeSphere(A, plane, sphere, margin) {
    return [planeSphereAt(A, plane, sphere.pos, sphere.size[0], margin, 'ps')];
}

// mjc_PlaneCapsule: a sphere test at each end of its segment, each frame's tangent along its axis.
function planeCapsule(A, plane, capsule, margin) {
    const axis = column(capsule.mat, 2);
    const segment = v3.scale(A, axis, capsule.size[1]);
    return [v3.add(A, capsule.pos, segment), v3.sub(A, capsule.pos, segment)].map((end, index) => {
        const slot = planeSphereAt(A, plane, end, capsule.size[0], margin, `pc${index + 1}`);
        return { ...slot, frame: [...slot.frame.slice(0, 3), ...axis] };
    });
}

// mjc_PlaneCylinder: the rim's point nearest the plane at the end nearer it, then at the other end, then two more
// points of the nearer end's rim a third of the way round, all only when the first touches.
function planeCylinder(A, plane, cylinder, margin) {
    const normal = column(plane.mat, 2);
    const rawAxis = column(cylinder.mat, 2);
    const rawProjection = v3.dot(A, normal, rawAxis);
    const flip = A.gt(rawProjection, 0);
    const axis = v3.name(A, 'cya', v3.pick(A, flip, v3.scale(A, rawAxis, -1), rawAxis));
    const projection = A.name('cyp', A.pick(flip, A.neg(rawProjection), rawProjection));
    const dist0 = A.name('cy0', v3.dot(A, v3.sub(A, cylinder.pos, plane.pos), normal));
    const across = v3.sub(A, v3.scale(A, axis, projection), normal);
    const lengthSquared = A.name('cyl', v3.dot(A, across, across));
    const general = A.ge(lengthSquared, minimumValue * minimumValue);
    const scale = A.div(cylinder.size[0], A.sqrt(A.pick(general, lengthSquared, A.num(1))));
    const vec = v3.name(A, 'cyv', v3.pick(A, general, v3.scale(A, across, scale), v3.scale(A, column(cylinder.mat, 0), cylinder.size[0])));
    const projectionVec = A.name('cypv', v3.dot(A, vec, normal));
    const halfAxis = v3.scale(A, axis, cylinder.size[1]);
    const halfProjection = A.mul(projection, cylinder.size[1]);
    const point = (dist, offset, tag) => v3.name(A, tag, v3.add(A, v3.add(A, cylinder.pos, offset), v3.scale(A, normal, A.mul(-0.5, dist))));
    const d1 = A.name('cyd1', A.add(dist0, halfProjection, projectionVec));
    const first = A.le(d1, margin);
    const d2 = A.name('cyd2', A.add(A.sub(dist0, halfProjection), projectionVec));
    const second = A.and(first, A.le(d2, margin));
    const d3 = A.name('cyd3', A.add(dist0, halfProjection, A.mul(-0.5, projectionVec)));
    const third = A.and(first, A.le(d3, margin));
    const sideways = normalize(A, 'cys', v3.cross(A, vec, halfAxis)).unit;
    const side = v3.scale(A, sideways, A.mul(cylinder.size[0], Math.sqrt(3) / 2));
    const nearEnd = v3.add(A, halfAxis, v3.scale(A, vec, -0.5));
    // A slot for each point MuJoCo may make (it packs the ones that touch; the slots here stay where they are).
    const slots = [
        { active: first, dist: d1, pos: point(d1, v3.add(A, vec, halfAxis), 'cyq1') },
        { active: second, dist: d2, pos: point(d2, v3.sub(A, vec, halfAxis), 'cyq2') },
        { active: third, dist: d3, pos: point(d3, v3.add(A, side, nearEnd), 'cyq3') },
        { active: third, dist: d3, pos: point(d3, v3.sub(A, nearEnd, side), 'cyq4') }
    ];
    return slots.map((slot) => ({ ...slot, frame: [...normal, ...noTangent(A)] }));
}

// mjc_PlaneBox: each corner below the box's centre (along the normal) within the margin, the first four in MuJoCo's
// order of corners.
function planeBox(A, plane, box, margin) {
    const normal = column(plane.mat, 2);
    const dist = A.name('pbh', v3.dot(A, v3.sub(A, box.pos, plane.pos), normal));
    const corners = Array.from({ length: 8 }, (_, i) => {
        const local = [i & 1 ? box.size[0] : -box.size[0], i & 2 ? box.size[1] : -box.size[1], i & 4 ? box.size[2] : -box.size[2]];
        const corner = v3.mat(A, box.mat, local);
        const depth = A.name(`pbl${i + 1}`, v3.dot(A, normal, corner));
        const cornerDist = A.name(`pbd${i + 1}`, A.add(dist, depth));
        const touches = A.name(`pbo${i + 1}`, A.and(A.le(cornerDist, margin), A.le(depth, 0)));
        const pos = v3.name(A, `pbp${i + 1}`, v3.add(A, v3.add(A, corner, box.pos), v3.scale(A, normal, A.mul(-0.5, cornerDist))));
        return { touches, dist: cornerDist, pos };
    });
    // How many corners before each touch, named once, so no slot's choice repeats the count before it.
    const before = [A.num(0)];
    corners.forEach(({ touches }, i) => before.push(A.name(`pbn${i + 1}`, A.add(before[i], A.pick(touches, 1, 0)))));
    // Slot k holds the (k+1)th touching corner: corner i is it when exactly k corners before it touch.
    return [0, 1, 2, 3].map((k) => {
        const takers = corners.map(({ touches }, i) => A.name(`pbt${k + 1}x${i + 1}`, A.and(touches, A.gt(before[i], k - 0.5), A.lt(before[i], k + 0.5))));
        const pickAll = (values) => values.reduceRight((rest, value, i) => A.pick(takers[i], value, rest), values[values.length - 1]);
        return {
            active: A.or(...takers),
            dist: A.name(`pbs${k + 1}d`, pickAll(corners.map(({ dist: value }) => value))),
            pos: [0, 1, 2].map((axis) => A.name(`pbs${k + 1}p${axis + 1}`, pickAll(corners.map(({ pos }) => pos[axis])))),
            frame: [...normal, ...noTangent(A)]
        };
    });
}

// ---- spheres and capsules ----------------------------------------------------------------------------------------
// mjraw_SphereSphere at centres p1 and p2: the normal from the first to the second (when the centres meet, the cross
// product of their z axes).
function sphereSphereAt(A, p1, r1, mat1, p2, r2, mat2, margin, tag) {
    const difference = v3.sub(A, p2, p1);
    const centres = normalize(A, `${tag}c`, difference);
    const reach = A.add(margin, r1, r2);
    const active = A.le(v3.dot(A, difference, difference), A.mul(reach, reach));
    const dist = A.name(`${tag}d`, A.sub(centres.length, A.add(r1, r2)));
    const crossed = normalize(A, `${tag}x`, v3.cross(A, column(mat1, 2), column(mat2, 2))).unit;
    const normal = v3.name(A, `${tag}n`, v3.pick(A, A.lt(centres.length, minimumValue), crossed, centres.unit));
    const pos = v3.name(A, `${tag}p`, v3.add(A, p1, v3.scale(A, normal, A.add(r1, A.mul(0.5, dist)))));
    return { active, dist, pos, frame: [...normal, ...noTangent(A)] };
}

function sphereSphere(A, first, second, margin) {
    return [sphereSphereAt(A, first.pos, first.size[0], first.mat, second.pos, second.size[0], second.mat, margin, 'ss')];
}

// mjraw_SphereCapsule: the sphere against the capsule's segment's point nearest its centre.
function sphereCapsule(A, sphere, capsule, margin) {
    const axis = column(capsule.mat, 2);
    const along = clip(A, v3.dot(A, axis, v3.sub(A, sphere.pos, capsule.pos)), A.neg(capsule.size[1]), capsule.size[1]);
    const nearest = v3.name(A, 'scq', v3.add(A, capsule.pos, v3.scale(A, axis, along)));
    return [sphereSphereAt(A, sphere.pos, sphere.size[0], sphere.mat, nearest, capsule.size[0], capsule.mat, margin, 'sc')];
}

// mjc_SphereCylinder: the sphere's centre beside the cylinder (within its height) against the axis's point level with
// it, as two spheres; over a cap (within its radius) against the cap's plane, the normal turned to point from the
// sphere; otherwise against the rim's nearest point, as a sphere of no size. A centre inside the cylinder goes out
// through the side or the cap, whichever is nearer (the cap where they are equal).
function sphereCylinder(A, sphere, cylinder, margin) {
    const radius = cylinder.size[0];
    const height = cylinder.size[1];
    const axis = column(cylinder.mat, 2);
    const offset = v3.sub(A, sphere.pos, cylinder.pos);
    const along = A.name('syx', v3.dot(A, offset, axis));
    const radial = v3.name(A, 'syr', v3.sub(A, offset, v3.scale(A, axis, along)));
    const radialSquared = A.name('syq', v3.dot(A, radial, radial));
    const radialLength = A.name('syl', A.sqrt(radialSquared));
    const beside = A.lt(A.abs(along), height);
    const over = A.lt(radialSquared, A.mul(radius, radius));
    // Inside both: the side only where the cap is no nearer.
    const capNearer = A.lt(A.sub(height, A.abs(along)), A.sub(radius, radialLength));
    const side = A.name('sys', A.pick(A.and(beside, over), A.not(capNearer), beside));
    const cap = A.name('syc', A.and(A.not(side), over));
    const top = A.gt(along, 0);
    const sign = A.pick(top, A.num(1), A.num(-1));
    // Beside: two spheres, the second of the cylinder's radius at the axis's point level with the centre.
    const level = v3.name(A, 'syv', v3.add(A, cylinder.pos, v3.scale(A, axis, along)));
    const bySide = sphereSphereAt(A, sphere.pos, sphere.size[0], sphere.mat, level, radius, cylinder.mat, margin, 'syb');
    // Over a cap: the cap's plane, facing out of the cylinder, the normal then turned to point from the sphere.
    const capCentre = v3.name(A, 'syk', v3.add(A, cylinder.pos, v3.scale(A, axis, A.mul(sign, height))));
    const outward = v3.scale(A, axis, sign);
    const plane = { pos: capCentre, mat: [0, 0, outward[0], 0, 0, outward[1], 0, 0, outward[2]].map((value) => (typeof value === 'number' ? A.num(value) : value)) };
    const byCap = planeSphereAt(A, plane, sphere.pos, sphere.size[0], margin, 'sya');
    // Otherwise the rim's nearest point, a sphere of no size.
    const safeLength = A.pick(A.gt(radialSquared, 0), radialLength, A.num(1));
    const rim = v3.name(A, 'sye', v3.add(A, capCentre, v3.scale(A, radial, A.div(radius, safeLength))));
    const byRim = sphereSphereAt(A, sphere.pos, sphere.size[0], sphere.mat, rim, A.num(0), cylinder.mat, margin, 'syo');
    const choose = (bySideValue, byCapValue, byRimValue) => A.pick(side, bySideValue, A.pick(cap, byCapValue, byRimValue));
    const capNormal = byCap.frame.slice(0, 3).map((value) => A.neg(value));
    return [{
        active: A.or(A.and(side, bySide.active), A.and(cap, byCap.active), A.and(A.not(side), A.not(cap), byRim.active)),
        dist: A.name('syd', choose(bySide.dist, byCap.dist, byRim.dist)),
        pos: v3.name(A, 'syp', [0, 1, 2].map((i) => choose(bySide.pos[i], byCap.pos[i], byRim.pos[i]))),
        frame: [...v3.name(A, 'syn', [0, 1, 2].map((i) => choose(bySide.frame[i], capNormal[i], byRim.frame[i]))), ...noTangent(A)]
    }];
}

// mjraw_CapsuleCapsule: the two segments' nearest points; when the axes are parallel, up to two pairs of points from
// the segments' ends.
function capsuleCapsule(A, first, second, margin) {
    const r1 = first.size[0];
    const r2 = second.size[0];
    const axis1 = v3.scale(A, column(first.mat, 2), first.size[1]);
    const axis2 = v3.scale(A, column(second.mat, 2), second.size[1]);
    const dif = v3.sub(A, first.pos, second.pos);
    const ma = A.name('cca', v3.dot(A, axis1, axis1));
    const mb = A.name('ccb', A.neg(v3.dot(A, axis1, axis2)));
    const mc = A.name('ccc', v3.dot(A, axis2, axis2));
    const u = A.name('ccu', A.neg(v3.dot(A, axis1, dif)));
    const v = A.name('ccv', v3.dot(A, axis2, dif));
    const det = A.name('ccdet', A.sub(A.mul(ma, mc), A.mul(mb, mb)));
    const generalCase = A.ge(A.abs(det), minimumValue);
    const safeDet = A.pick(generalCase, det, A.num(1));
    // General: the projections, clipped to the segments as MuJoCo clips them.
    let x1 = A.div(A.sub(A.mul(mc, u), A.mul(mb, v)), safeDet);
    let x2 = A.div(A.sub(A.mul(ma, v), A.mul(mb, u)), safeDet);
    const over1 = A.gt(x1, 1);
    const under1 = A.lt(x1, -1);
    x2 = A.name('ccx2a', A.pick(over1, A.div(A.sub(v, mb), mc), A.pick(under1, A.div(A.add(v, mb), mc), x2)));
    x1 = A.name('ccx1a', A.pick(over1, A.num(1), A.pick(under1, A.num(-1), x1)));
    const over2 = A.gt(x2, 1);
    const under2 = A.lt(x2, -1);
    x1 = A.name('ccx1', A.pick(over2, clip(A, A.div(A.sub(u, mb), ma), -1, 1), A.pick(under2, clip(A, A.div(A.add(u, mb), ma), -1, 1), x1)));
    x2 = A.name('ccx2', A.pick(over2, A.num(1), A.pick(under2, A.num(-1), x2)));
    const at = (pos, axis, x) => v3.add(A, pos, v3.scale(A, axis, x));
    const general = sphereSphereAt(A, at(first.pos, axis1, x1), r1, first.mat, at(second.pos, axis2, x2), r2, second.mat, margin, 'ccg');
    // Parallel: the first segment's ends against the second, then the second's ends against the first, up to two.
    const ends = [
        [1, clip(A, A.div(A.sub(v, mb), mc), -1, 1), 'first'], [-1, clip(A, A.div(A.add(v, mb), mc), -1, 1), 'first'],
        [clip(A, A.div(A.sub(u, mb), ma), -1, 1), 1, 'second'], [clip(A, A.div(A.add(u, mb), ma), -1, 1), -1, 'second']
    ].map(([a, b], index) => sphereSphereAt(A, at(first.pos, axis1, a), r1, first.mat, at(second.pos, axis2, b), r2, second.mat, margin, `ccp${index + 1}`));
    // MuJoCo stops after two; the slots take the touching ends in order.
    const t = ends.map(({ active }) => active);
    const firstTaker = [t[0], A.and(A.not(t[0]), t[1]), A.and(A.not(t[0]), A.not(t[1]), t[2]), A.and(A.not(t[0]), A.not(t[1]), A.not(t[2]), t[3])];
    const secondTaker = [
        A.never,
        A.and(t[0], t[1]),
        A.and(A.or(A.and(t[0], A.not(t[1])), A.and(A.not(t[0]), t[1])), t[2]),
        A.and(A.or(A.and(t[0], A.not(t[1]), A.not(t[2])), A.and(A.not(t[0]), t[1], A.not(t[2])), A.and(A.not(t[0]), A.not(t[1]), t[2])), t[3])
    ];
    const choose = (takers, tag) => {
        const pickAll = (values) => values.reduceRight((rest, value, i) => A.pick(takers[i], value, rest), values[3]);
        return {
            active: A.or(...takers),
            dist: A.name(`${tag}d`, pickAll(ends.map(({ dist }) => dist))),
            pos: [0, 1, 2].map((axis) => A.name(`${tag}p${axis + 1}`, pickAll(ends.map(({ pos }) => pos[axis])))),
            frame: [0, 1, 2].map((axis) => A.name(`${tag}n${axis + 1}`, pickAll(ends.map(({ frame }) => frame[axis])))).concat(noTangent(A))
        };
    };
    const parallel1 = choose(firstTaker, 'ccq1');
    const parallel2 = choose(secondTaker, 'ccq2');
    const slot1 = {
        active: A.pick(generalCase, general.active, parallel1.active),
        dist: A.name('ccs1d', A.pick(generalCase, general.dist, parallel1.dist)),
        pos: v3.name(A, 'ccs1p', v3.pick(A, generalCase, general.pos, parallel1.pos)),
        frame: [...v3.name(A, 'ccs1n', v3.pick(A, generalCase, general.frame.slice(0, 3), parallel1.frame.slice(0, 3))), ...noTangent(A)]
    };
    return [slot1, { ...parallel2, active: A.and(A.not(generalCase), parallel2.active) }];
}

// mjraw_SphereBox: the box's point nearest the sphere's centre (in the box's frame); when the centre is inside, the
// nearest face's.
function sphereBox(A, sphere, box, margin, tag = 'sb') {
    const r = sphere.size[0];
    const centre = v3.name(A, `${tag}c`, v3.matT(A, box.mat, v3.sub(A, sphere.pos, box.pos)));
    const clamped = v3.name(A, `${tag}k`, centre.map((value, i) => clip(A, value, A.neg(box.size[i]), box.size[i])));
    const toward = normalize(A, `${tag}t`, v3.sub(A, clamped, centre));
    const inside = A.name(`${tag}i`, A.le(toward.length, minimumValue));
    // Inside: the nearest of the six faces (the first of equals, in MuJoCo's order). Each step's nearest so far, and
    // whether a face is nearer than those before it, named once, so no later step repeats an earlier one.
    const faces = Array.from({ length: 6 }, (_, i) => A.name(`${tag}f${i + 1}`, A.abs(A.sub(A.mul(i % 2 ? 1 : -1, box.size[Math.floor(i / 2)]), centre[Math.floor(i / 2)]))));
    let closest = A.mul(2, A.add(box.size[0], box.size[1], box.size[2]));
    let chosen = [A.never, A.never, A.never, A.never, A.never, A.never];
    faces.forEach((value, i) => {
        const nearer = A.name(`${tag}g${i + 1}`, A.gt(closest, value));
        chosen = chosen.map((was, j) => (j === i ? nearer : A.and(was, A.not(nearer))));
        closest = A.name(i === faces.length - 1 ? `${tag}m` : `${tag}m${i + 1}`, A.pick(nearer, value, closest));
    });
    const nearest = v3.name(A, `${tag}e`, [0, 1, 2].map((axis) => A.add(A.pick(chosen[2 * axis], A.num(1), A.num(0)), A.pick(chosen[2 * axis + 1], A.num(-1), A.num(0)))));
    const insidePos = v3.add(A, centre, v3.scale(A, nearest, A.mul(0.5, A.sub(r, closest))));
    const deepest = v3.add(A, centre, v3.scale(A, toward.unit, r));
    const outsidePos = v3.scale(A, v3.add(A, clamped, deepest), 0.5);
    const local = v3.pick(A, inside, insidePos, outsidePos);
    const normal = v3.name(A, `${tag}n`, v3.mat(A, box.mat, v3.pick(A, inside, nearest, toward.unit)));
    const dist = A.name(`${tag}d`, A.sub(A.pick(inside, A.neg(closest), toward.length), r));
    return [{
        active: A.le(A.sub(toward.length, r), margin),
        dist,
        pos: v3.name(A, `${tag}p`, v3.add(A, v3.mat(A, box.mat, local), box.pos)),
        frame: [...normal, ...noTangent(A)]
    }];
}

// mjraw_CapsuleBox (Svetoslav Kolev's, in MuJoCo): the capsule's segment's point nearest the box (a capsule's end
// against a face, or the segment against one of the box's twelve edges), a sphere test there; then, where the segment
// lies along a face or an edge, a second sphere test further along it, as far as stays over the box. Its integer
// bookkeeping (which face, edge or corner is nearest, and how the capsule points) is kept here as numbers of 0 and 1
// over the algebra, so it can be written as equations too.
function capsuleBox(A, capsule, box, margin) {
    const r = capsule.size[0];
    const halfLength = capsule.size[1];
    const size = box.size;
    const pos = v3.name(A, 'cbp', v3.matT(A, box.mat, v3.sub(A, capsule.pos, box.pos)));
    const axis = v3.name(A, 'cba', v3.matT(A, box.mat, column(capsule.mat, 2)));
    const halfAxis = v3.name(A, 'cbh', v3.scale(A, axis, halfLength));
    const bit = (condition) => A.pick(condition, A.num(1), A.num(0));
    const direction = [0, 1, 2].map((k) => A.name(`cbdir${k + 1}`, bit(A.gt(halfAxis[k], 0))));
    const reach = A.add(margin, A.mul(2, A.add(r, halfLength, size[0], size[1], size[2])));
    let best = A.mul(reach, reach);
    let bestSegment = A.num(0);
    let bestBox = A.num(0);
    // cltype: -4 none, -3 or -1 an end against a face, 0 to 8 an edge; kept as its value.
    let type = A.num(-4);
    // The nearest face (clface) as a weight per axis, and a weight for none (-1).
    let face = [A.num(0), A.num(0), A.num(0)];
    let noFace = A.num(0);
    // The nearest corner's bits (clcorner) and the nearest edge's axis (cledge), as weights.
    let corner = [A.num(0), A.num(0), A.num(0)];
    let edge = [A.num(0), A.num(0), A.num(0)];
    // A capsule's end against a face: the end clamped to the box, where at most one coordinate is clamped.
    for (const end of [-1, 1]) {
        const point = v3.add(A, pos, v3.scale(A, halfAxis, end));
        const below = point.map((value, j) => A.lt(value, A.neg(size[j])));
        const above = point.map((value, j) => A.gt(value, size[j]));
        const clamped = point.map((value, j) => A.pick(below[j], A.neg(size[j]), A.pick(above[j], size[j], value)));
        const out = [0, 1, 2].map((j) => bit(A.or(below[j], above[j])));
        const count = A.add(out[0], out[1], out[2]);
        const gap = v3.sub(A, clamped, point);
        const distance = A.name(`cbe${end > 0 ? 2 : 1}`, v3.dot(A, gap, gap));
        const better = A.and(A.le(count, 1), A.lt(distance, best));
        // The face is the last axis clamped (c2), or none.
        const last = [0, 1, 2].map((j) => A.mul(out[j], ...[0, 1, 2].filter((k) => k > j).map((k) => A.sub(1, out[k]))));
        best = A.name(`cbb${end > 0 ? 2 : 1}`, A.pick(better, distance, best));
        bestSegment = A.pick(better, A.num(end), bestSegment);
        type = A.pick(better, A.num(-2 + end), type);
        face = face.map((value, j) => A.pick(better, last[j], value));
        noFace = A.pick(better, A.sub(1, A.add(...last)), noFace);
    }
    // The segment against each edge of the box (along axis j, from corner i with bit j clear).
    for (let j = 0; j < 3; j += 1) {
        for (let i = 0; i < 8; i += 1) {
            if (i & (1 << j)) continue;
            const start = [0, 1, 2].map((k) => (k === j ? A.num(0) : A.mul((i & (1 << k)) ? 1 : -1, size[k])));
            const dif = v3.sub(A, start, pos);
            const ma = A.mul(size[j], size[j]);
            const mb = A.neg(A.mul(size[j], halfAxis[j]));
            const mc = A.mul(halfLength, halfLength);
            const u = A.neg(A.mul(size[j], dif[j]));
            const v = v3.dot(A, halfAxis, dif);
            const det = A.sub(A.mul(ma, mc), A.mul(mb, mb));
            const usable = A.ge(A.abs(det), minimumValue);
            const safe = A.pick(usable, det, A.num(1));
            let x1 = A.div(A.sub(A.mul(mc, u), A.mul(mb, v)), safe);
            let x2 = A.div(A.sub(A.mul(ma, v), A.mul(mb, u)), safe);
            let s1 = A.num(1);
            let s2 = A.num(1);
            const over1 = A.gt(x1, 1);
            const under1 = A.lt(x1, -1);
            x2 = A.pick(over1, A.div(A.sub(v, mb), mc), A.pick(under1, A.div(A.add(v, mb), mc), x2));
            s1 = A.pick(over1, A.num(2), A.pick(under1, A.num(0), s1));
            x1 = A.pick(over1, A.num(1), A.pick(under1, A.num(-1), x1));
            const over2 = A.gt(x2, 1);
            const under2 = A.lt(x2, -1);
            const redo = A.pick(over2, A.div(A.sub(u, mb), ma), A.div(A.add(u, mb), ma));
            const redone = A.or(over2, under2);
            s1 = A.pick(redone, A.pick(A.gt(redo, 1), A.num(2), A.pick(A.lt(redo, -1), A.num(0), s1)), s1);
            x1 = A.pick(redone, A.pick(A.gt(redo, 1), A.num(1), A.pick(A.lt(redo, -1), A.num(-1), redo)), x1);
            s2 = A.pick(over2, A.num(2), A.pick(under2, A.num(0), s2));
            x2 = A.pick(over2, A.num(1), A.pick(under2, A.num(-1), x2));
            x1 = A.name(`cbx${j}${i}a`, x1);
            x2 = A.name(`cbx${j}${i}b`, x2);
            const gap = v3.sub(A, dif, v3.scale(A, halfAxis, x2)).map((value, k) => (k === j ? A.add(value, A.mul(size[j], x1)) : value));
            const distance = A.name(`cbd${j}${i}`, v3.dot(A, gap, gap));
            const better = A.and(usable, A.lt(distance, A.sub(best, minimumValue)));
            const upper = A.ge(s1, 2); // c1 / 6 = 1: the closest point on the box at the edge's upper end
            best = A.name(`cbq${j}${i}`, A.pick(better, distance, best));
            bestSegment = A.name(`cbs${j}${i}`, A.pick(better, x2, bestSegment));
            bestBox = A.pick(better, x1, bestBox);
            type = A.name(`cbt${j}${i}`, A.pick(better, A.add(A.mul(3, s1), s2), type));
            corner = corner.map((value, k) => A.pick(better, k === j ? bit(upper) : A.num((i >> k) & 1), value));
            edge = edge.map((value, k) => A.pick(better, A.num(k === j ? 1 : 0), value));
        }
    }
    bestSegment = A.name('cbbs', bestSegment);
    bestBox = A.name('cbbb', bestBox);
    corner = corner.map((value, k) => A.name(`cbcn${k + 1}`, value));
    edge = edge.map((value, k) => A.name(`cbed${k + 1}`, value));
    face = face.map((value, k) => A.name(`cbfc${k + 1}`, value));
    const xor = (a, b) => A.sub(A.add(a, b), A.mul(2, a, b));
    const has = (bits, weights) => A.add(...[0, 1, 2].map((k) => A.mul(bits[k], weights[k])));
    const along = (values, weights) => A.add(...[0, 1, 2].map((k) => A.mul(values[k], weights[k])));
    const rotateWeights = (weights, by) => [0, 1, 2].map((k) => weights[(k + 3 - by) % 3]);
    const minimum = (current, candidate) => A.pick(A.lt(candidate, current), candidate, current);
    const absHalf = halfAxis.map((value) => A.abs(value));
    const span = (k) => A.pick(A.gt(absHalf[k], 0), A.div(A.mul(2, size[k]), A.pick(A.gt(absHalf[k], 0), absHalf[k], A.num(1))), A.num(1e300));
    const spanAlong = (weights) => along([0, 1, 2].map(span), weights);
    const isEdgeType = A.and(A.ge(type, 3), A.le(type, 5));
    const isCornerType = A.and(A.ge(type, 0), A.not(isEdgeType));
    // Nearest a corner: how the capsule points relative to it.
    const cornerBits = [0, 1, 2].map((k) => A.name(`cbk${k + 1}`, xor(direction[k], corner[k])));
    const cornerCount = A.add(...cornerBits);
    const twoBits = A.ge(cornerCount, 1.5);
    const oneHot = cornerBits.map((value) => A.pick(twoBits, A.sub(1, value), value)); // c1, or 7 - c1: one bit
    const cornerMul = A.pick(twoBits, A.num(-1), A.num(1));
    const de = A.pick(twoBits, A.add(1, bestSegment), A.sub(1, bestSegment));
    const dp = A.pick(twoBits, A.sub(1, bestSegment), A.add(1, bestSegment));
    const axAxis = along(axis, oneHot);
    const alongEdge = A.gt(A.mul(axAxis, axAxis), 0.5);
    const cornerSecond = A.pick(alongEdge,
        A.mul(minimum(de, spanAlong(oneHot)), cornerMul),
        A.mul(minimum(minimum(dp, spanAlong(rotateWeights(oneHot, 1))), spanAlong(rotateWeights(oneHot, 2))), A.neg(cornerMul)));
    const cornerUsable = A.and(isCornerType, A.gt(cornerCount, 0.5), A.lt(cornerCount, 2.5));
    // Nearest an edge: the capsule crossing it, or square to it (no second point).
    const edgeBits = [0, 1, 2].map((k) => A.mul(xor(direction[k], corner[k]), A.sub(1, edge[k])));
    const edgeCount = A.add(...edgeBits);
    const edgeUsable = A.and(isEdgeType, A.gt(edgeCount, 0.5), A.lt(edgeCount, 1.5));
    const first = rotateWeights(edge, 1);
    const second = rotateWeights(edge, 2);
    const absAxis = axis.map((value) => A.abs(value));
    const swap = A.gt(along(absAxis, first), along(absAxis, second));
    const ax2 = [0, 1, 2].map((k) => A.pick(swap, first[k], second[k]));
    const towards = A.gt(has(edgeBits, ax2), 0.5);
    const edgeMul = A.pick(towards, A.num(1), A.num(-1));
    let edgeSecond = A.pick(towards, A.sub(1, bestSegment), A.add(1, bestSegment));
    edgeSecond = minimum(edgeSecond, spanAlong(ax2));
    const sameSide = A.or(A.and(A.gt(has(direction, edge), 0.5), towards), A.and(A.lt(has(direction, edge), 0.5), A.not(towards)));
    const e2 = A.pick(sameSide, A.sub(1, bestBox), A.add(1, bestBox));
    const alongAx = along(absHalf, edge);
    edgeSecond = minimum(edgeSecond, A.pick(A.gt(alongAx, 0), A.div(A.mul(along(size, edge), e2), A.pick(A.gt(alongAx, 0), alongAx, A.num(1))), A.num(1e300)));
    edgeSecond = A.mul(edgeSecond, edgeMul);
    // An end nearest a face: how far the other end's way the segment stays over the box.
    const faceMul = A.pick(A.lt(type, -2), A.num(1), A.num(-1));
    const back = v3.sub(A, pos, v3.scale(A, halfAxis, faceMul));
    let faceSecond = A.num(2);
    for (let k = 0; k < 3; k += 1) {
        const usable = A.and(A.lt(face[k], 0.5), A.gt(A.abs(halfAxis[k]), 0));
        const safe = A.pick(A.gt(A.abs(halfAxis[k]), 0), halfAxis[k], A.num(1));
        for (const bound of [size[k], A.neg(size[k])]) {
            const e1 = A.mul(A.div(A.sub(bound, back[k]), safe), faceMul);
            faceSecond = A.pick(A.and(usable, A.gt(e1, 0), A.lt(e1, faceSecond)), e1, faceSecond);
        }
    }
    faceSecond = A.mul(faceSecond, faceMul);
    const faceUsable = A.and(A.lt(type, 0), A.gt(type, -3.5), A.lt(noFace, 0.5));
    const secondPos = A.name('cbsp', A.pick(cornerUsable, cornerSecond, A.pick(edgeUsable, edgeSecond, A.pick(faceUsable, faceSecond, A.num(-4)))));
    const any = A.gt(type, -3.5);
    const sphereAt = (offset, tag) => ({
        kind: 'sphere', size: [r], mat: capsule.mat,
        pos: v3.name(A, `${tag}w`, v3.add(A, v3.mat(A, box.mat, v3.add(A, pos, v3.scale(A, halfAxis, offset))), box.pos))
    });
    const [one] = sphereBox(A, sphereAt(bestSegment, 'cb1'), box, margin, 'cb1');
    const [two] = sphereBox(A, sphereAt(A.add(secondPos, bestSegment), 'cb2'), box, margin, 'cb2');
    return [{ ...one, active: A.and(any, one.active) }, { ...two, active: A.and(any, A.gt(secondPos, -3), two.active) }];
}

// ---- box against box ---------------------------------------------------------------------------------------------
// mjc_BoxBox: the axis along which the boxes overlap least, of their six face normals and the nine cross products of
// an edge of each (those of edges nearly parallel left out), the first such in that order where two are equal. Along
// a face: the other box's face turned most against it is clipped to it, and each corner of what remains (the other
// box's corners within the face, the face's corners within the other box's, and where their edges cross) is a contact
// while within the margin, each at its own depth, midway through it. Along two edges: one contact, midway between
// their nearest points. None where any axis separates them by more than the margin. As MuJoCo has it to rounding
// wherever they overlap by less than a millimetre; deeper, where an edge's axis lies within a few degrees of a face's
// and they overlap about as much along each, MuJoCo sometimes takes the face where this takes the edge.
function boxBox(A, first, second, margin) {
    const A1 = [0, 1, 2].map((i) => column(first.mat, i));
    const A2 = [0, 1, 2].map((j) => column(second.mat, j));
    const s1 = first.size;
    const s2 = second.size;
    const between = v3.name(A, 'bbd', v3.sub(A, second.pos, first.pos));
    const reach = (axis) => A.add(...[0, 1, 2].map((k) => A.mul(s1[k], A.abs(v3.dot(A, axis, A1[k])))), ...[0, 1, 2].map((k) => A.mul(s2[k], A.abs(v3.dot(A, axis, A2[k])))));
    // Each axis, turned to point from the first box to the second, and how far they overlap along it.
    const candidates = [];
    [[A1, 'a'], [A2, 'b']].forEach(([axes, tag]) => axes.forEach((axis, k) => {
        const along = A.name(`bb${tag}c${k + 1}`, v3.dot(A, between, axis));
        const sign = A.pick(A.lt(along, 0), A.num(-1), A.num(1));
        candidates.push({
            kind: tag === 'a' ? 'first' : 'second', index: k, usable: A.always,
            axis: v3.name(A, `bb${tag}n${k + 1}`, v3.scale(A, axis, sign)),
            depth: A.name(`bb${tag}o${k + 1}`, A.sub(reach(axis), A.abs(along)))
        });
    }));
    A1.forEach((a, i) => A2.forEach((b, j) => {
        const tag = `bbe${i + 1}${j + 1}`;
        const raw = v3.name(A, `${tag}x`, v3.cross(A, a, b));
        const length = A.name(`${tag}l`, A.sqrt(v3.dot(A, raw, raw)));
        const usable = A.name(`${tag}u`, A.ge(length, 1e-6));
        const safe = A.pick(usable, length, A.num(1));
        const unit = raw.map((value) => A.div(value, safe));
        const along = A.name(`${tag}c`, v3.dot(A, between, unit));
        const sign = A.pick(A.lt(along, 0), A.num(-1), A.num(1));
        const axis = v3.name(A, `${tag}n`, v3.scale(A, unit, sign));
        candidates.push({ kind: 'edge', i, j, usable, axis, depth: A.name(`${tag}o`, A.sub(reach(axis), A.abs(along))) });
    }));
    const apart = A.name('bbsep', A.or(...candidates.map(({ usable, depth }) => A.and(usable, A.lt(depth, A.neg(margin))))));
    // The least overlap, the first of equals: each candidate's whether it is nearer than those before it, named once.
    let least = A.num(1e30);
    let chosen = candidates.map(() => A.never);
    candidates.forEach(({ usable, depth }, c) => {
        const nearer = A.name(`bbg${c + 1}`, A.and(usable, A.lt(depth, least)));
        chosen = chosen.map((was, k) => (k === c ? nearer : A.and(was, A.not(nearer))));
        least = A.name(`bbm${c + 1}`, A.pick(nearer, depth, least));
    });
    chosen = chosen.map((flag, c) => A.name(`bbk${c + 1}`, flag));
    const pickOf = (values) => values.reduceRight((rest, value, c) => A.pick(chosen[c], value, rest), values[values.length - 1]);
    const normal = v3.name(A, 'bbn', [0, 1, 2].map((axis) => pickOf(candidates.map((candidate) => candidate.axis[axis]))));
    const alongEdge = A.name('bbae', A.or(...chosen.slice(6)));
    const fromFirst = A.name('bbaf', A.or(...chosen.slice(0, 3)));
    const which = (flags) => (values) => values.reduceRight((rest, value, k) => A.pick(flags[k], value, rest), values[values.length - 1]);

    // Along a face: the reference box owns it (its outward normal `out`), the incident box is the other.
    const faceIndex = [0, 1, 2].map((k) => A.name(`bbfi${k + 1}`, A.or(chosen[k], chosen[3 + k])));
    const byFace = which(faceIndex);
    const ref = {
        pos: v3.pick(A, fromFirst, first.pos, second.pos), axes: [0, 1, 2].map((k) => v3.pick(A, fromFirst, A1[k], A2[k])),
        size: [0, 1, 2].map((k) => A.pick(fromFirst, s1[k], s2[k]))
    };
    const inc = {
        pos: v3.pick(A, fromFirst, second.pos, first.pos), axes: [0, 1, 2].map((k) => v3.pick(A, fromFirst, A2[k], A1[k])),
        size: [0, 1, 2].map((k) => A.pick(fromFirst, s2[k], s1[k]))
    };
    const out = v3.name(A, 'bbo', v3.scale(A, normal, A.pick(fromFirst, A.num(1), A.num(-1))));
    // The face's own axes (the other two, in order) and half sizes; its centre.
    const others = [[1, 2], [0, 2], [0, 1]];
    const U = v3.name(A, 'bbu', [0, 1, 2].map((axis) => byFace(others.map(([u]) => ref.axes[u][axis]))));
    const V = v3.name(A, 'bbv', [0, 1, 2].map((axis) => byFace(others.map(([, v]) => ref.axes[v][axis]))));
    const su = A.name('bbsu', byFace(others.map(([u]) => ref.size[u])));
    const sv = A.name('bbsv', byFace(others.map(([, v]) => ref.size[v])));
    const sk = byFace(ref.size);
    const centre = v3.name(A, 'bbfc', v3.add(A, ref.pos, v3.scale(A, out, sk)));
    // The incident face: the incident box's face turned most against `out`, the first of equals.
    let most = A.num(1e30);
    let incident = Array.from({ length: 6 }, () => A.never);
    [0, 1, 2].forEach((j) => {
        const facing = A.name(`bbic${j + 1}`, v3.dot(A, inc.axes[j], out));
        [[facing, 2 * j], [A.neg(facing), 2 * j + 1]].forEach(([value, f]) => {
            const nearer = A.name(`bbih${f + 1}`, A.lt(value, most));
            incident = incident.map((was, k) => (k === f ? nearer : A.and(was, A.not(nearer))));
            most = A.name(`bbim${f + 1}`, A.pick(nearer, value, most));
        });
    });
    incident = incident.map((flag, f) => A.name(`bbif${f + 1}`, flag));
    const byIncident = which(incident);
    const incAxis = (k) => [0, 1, 2].map((axis) => inc.axes[k][axis]);
    const fi = v3.name(A, 'bbia', [0, 1, 2].map((axis) => byIncident([0, 1, 2].flatMap((j) => [incAxis(j)[axis], A.neg(incAxis(j)[axis])]))));
    const fs = A.name('bbis', byIncident([0, 1, 2].flatMap((j) => [inc.size[j], inc.size[j]])));
    const ea = v3.name(A, 'bbie', [0, 1, 2].map((axis) => byIncident([0, 1, 2].flatMap((j) => [incAxis(others[j][0])[axis], incAxis(others[j][0])[axis]]))));
    const eb = v3.name(A, 'bbig', [0, 1, 2].map((axis) => byIncident([0, 1, 2].flatMap((j) => [incAxis(others[j][1])[axis], incAxis(others[j][1])[axis]]))));
    const sa = A.name('bbisa', byIncident([0, 1, 2].flatMap((j) => [inc.size[others[j][0]], inc.size[others[j][0]]])));
    const sb = A.name('bbisb', byIncident([0, 1, 2].flatMap((j) => [inc.size[others[j][1]], inc.size[others[j][1]]])));
    // In the face's coordinates (x along U, y along V, h along out from the face): the incident face's centre and its
    // two half edges.
    const local = (vector, tag) => [U, V, out].map((axis, k) => A.name(`${tag}${k + 1}`, v3.dot(A, vector, axis)));
    const c2 = local(v3.sub(A, v3.add(A, inc.pos, v3.scale(A, fi, fs)), centre), 'bbq');
    const e1 = local(v3.scale(A, ea, sa), 'bbr');
    const e2 = local(v3.scale(A, eb, sb), 'bbt');
    const cornerSigns = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
    const corners = cornerSigns.map(([a, b], q) => [0, 1, 2].map((k) => A.name(`bbw${q + 1}${k + 1}`, A.add(c2[k], A.mul(a, e1[k]), A.mul(b, e2[k])))));
    const points = [];
    // The incident face's corners within the face.
    corners.forEach(([x, y, h]) => points.push({ x, y, h, valid: A.and(A.le(A.abs(x), su), A.le(A.abs(y), sv)) }));
    // The face's corners strictly within the incident face (by the incident face's own coordinates), at its height there.
    const det = A.name('bbdet', A.sub(A.mul(e1[0], e2[1]), A.mul(e1[1], e2[0])));
    const flat = A.lt(A.abs(det), minimumValue);
    const safeDet = A.pick(flat, A.num(1), det);
    cornerSigns.forEach(([a, b], q) => {
        const px = A.sub(A.mul(a, su), c2[0]);
        const py = A.sub(A.mul(b, sv), c2[1]);
        const alpha = A.name(`bbya${q + 1}`, A.div(A.sub(A.mul(px, e2[1]), A.mul(py, e2[0])), safeDet));
        const beta = A.name(`bbyb${q + 1}`, A.div(A.sub(A.mul(e1[0], py), A.mul(e1[1], px)), safeDet));
        points.push({
            x: A.mul(a, su), y: A.mul(b, sv), h: A.add(c2[2], A.mul(alpha, e1[2]), A.mul(beta, e2[2])),
            valid: A.and(A.not(flat), A.lt(A.abs(alpha), 1), A.lt(A.abs(beta), 1))
        });
    });
    // Where each incident edge crosses each of the face's four edge lines, strictly, within the face's other extent.
    corners.forEach((start, q) => {
        const end = corners[(q + 1) % 4];
        [[0, 1, su, sv], [0, -1, su, sv], [1, 1, sv, su], [1, -1, sv, su]].forEach(([k, sign, limit, otherLimit], l) => {
            const fa = A.sub(A.mul(sign, start[k]), limit);
            const fb = A.sub(A.mul(sign, end[k]), limit);
            const crosses = A.or(A.and(A.lt(fa, 0), A.gt(fb, 0)), A.and(A.gt(fa, 0), A.lt(fb, 0)));
            const t = A.name(`bbz${q + 1}${l + 1}`, A.div(fa, A.pick(crosses, A.sub(fa, fb), A.num(1))));
            const at = [0, 1, 2].map((m) => A.add(start[m], A.mul(t, A.sub(end[m], start[m]))));
            points.push({ x: at[0], y: at[1], h: at[2], valid: A.and(crosses, A.le(A.abs(at[1 - k]), otherLimit)) });
        });
    });
    const facePoints = points.map(({ x, y, h, valid }, p) => {
        const touches = A.name(`bbpv${p + 1}`, A.and(A.not(apart), A.not(alongEdge), valid, A.le(h, margin)));
        const dist = A.name(`bbph${p + 1}`, h);
        const pos = v3.name(A, `bbpp${p + 1}`, v3.add(A, centre, v3.add(A, v3.add(A, v3.scale(A, U, x), v3.scale(A, V, y)), v3.scale(A, out, A.mul(0.5, dist)))));
        return { touches, dist, pos };
    });

    // Along two edges: the supporting edge of each (the edge farthest along the axis toward the other box), and the
    // point midway between their nearest points, each kept within its edge.
    const edges = candidates.slice(6);
    const byEdge = (values) => values.reduceRight((rest, value, e) => A.pick(chosen[6 + e], value, rest), values[values.length - 1]);
    const ia = edges.map(({ i }) => i);
    const jb = edges.map(({ j }) => j);
    const signOf = (value) => A.pick(A.lt(value, 0), A.num(-1), A.num(1));
    const corner = (pos, axes, size, toward, tag) => {
        const signs = axes.map((axis, k) => A.name(`${tag}s${k + 1}`, signOf(v3.dot(A, axis, toward))));
        return { signs, point: v3.name(A, `${tag}c`, axes.reduce((sum, axis, k) => v3.add(A, sum, v3.scale(A, axis, A.mul(size[k], signs[k]))), pos)) };
    };
    const away = v3.scale(A, normal, -1);
    const k1 = corner(first.pos, A1, s1, normal, 'bbx1');
    const k2 = corner(second.pos, A2, s2, away, 'bbx2');
    const u = v3.name(A, 'bbxu', [0, 1, 2].map((axis) => byEdge(ia.map((i) => A1[i][axis]))));
    const v = v3.name(A, 'bbxv', [0, 1, 2].map((axis) => byEdge(jb.map((j) => A2[j][axis]))));
    const su1 = A.name('bbxa', byEdge(ia.map((i) => s1[i])));
    const sv2 = A.name('bbxb', byEdge(jb.map((j) => s2[j])));
    const p1 = v3.name(A, 'bbxp', v3.sub(A, k1.point, v3.scale(A, u, A.mul(su1, byEdge(ia.map((i) => k1.signs[i]))))));
    const p2 = v3.name(A, 'bbxq', v3.sub(A, k2.point, v3.scale(A, v, A.mul(sv2, byEdge(jb.map((j) => k2.signs[j]))))));
    const w = v3.sub(A, p1, p2);
    const uv = A.name('bbxuv', v3.dot(A, u, v));
    const uw = A.name('bbxuw', v3.dot(A, u, w));
    const vw = A.name('bbxvw', v3.dot(A, v, w));
    const den = A.sub(1, A.mul(uv, uv));
    const safeDen = A.pick(A.lt(den, minimumValue), A.num(1), den);
    const ta = clip(A, A.div(A.sub(A.mul(uv, vw), uw), safeDen), A.neg(su1), su1);
    const tb = clip(A, A.div(A.sub(vw, A.mul(uv, uw)), safeDen), A.neg(sv2), sv2);
    const edgePos = v3.name(A, 'bbxm', v3.scale(A, v3.add(A, v3.add(A, p1, v3.scale(A, u, ta)), v3.add(A, p2, v3.scale(A, v, tb))), 0.5));
    const edgePoint = { touches: A.name('bbxt', A.and(A.not(apart), alongEdge)), dist: A.name('bbxd', A.neg(least)), pos: edgePos };

    // Slot k holds the (k+1)th contact: the edges' first, then the face's points in order.
    const all = [edgePoint, ...facePoints];
    const before = [A.num(0)];
    all.forEach(({ touches }, p) => before.push(A.name(`bbcn${p + 1}`, A.add(before[p], A.pick(touches, 1, 0)))));
    return Array.from({ length: 8 }, (_, k) => {
        const takers = all.map(({ touches }, p) => A.name(`bbs${k + 1}t${p + 1}`, A.and(touches, A.gt(before[p], k - 0.5), A.lt(before[p], k + 0.5))));
        const pickAll = (values) => values.reduceRight((rest, value, p) => A.pick(takers[p], value, rest), values[values.length - 1]);
        return {
            active: A.or(...takers),
            dist: A.name(`bbs${k + 1}d`, pickAll(all.map(({ dist }) => dist))),
            pos: [0, 1, 2].map((axis) => A.name(`bbs${k + 1}p${axis + 1}`, pickAll(all.map(({ pos }) => pos[axis])))),
            frame: [...normal, ...noTangent(A)]
        };
    });
}

// ---- the largest gap: capsules, cylinders and boxes, exactly ------------------------------------------------------
// MuJoCo finds a cylinder against a capsule, a cylinder or a box by its iterative convex search, to its tolerance.
// These find the same contact exactly, by what makes it exact for any two convex shapes: their signed distance (less
// than nothing where they overlap, by the least they must move apart) is the largest gap between them along any
// direction n, n · (c2 - c1) less how far each reaches along it, and the contact's normal is that direction, its point
// midway between where each reaches furthest. Each shape's reach is a sum of terms: a segment's, L |n · a| (a capsule's
// axis, a cylinder's, a box's edges); a disc's, R (1 - (n · b)²)^½ (a cylinder's caps); a ball's, r. Where none of its
// absolute values changes sign and no disc is face on, the gap is smooth, and its largest is found in closed form or
// by a few Newton steps on a circle of directions; elsewhere the direction is fixed (a face's normal, two edges' cross
// product). Any direction's gap is no more than the largest, so the largest over every such candidate is exact.

const dot3 = (A, a, b) => v3.dot(A, a, b);
// v less its part along the unit `axis`.
const square = (A, v, axis) => v3.sub(A, v, v3.scale(A, axis, dot3(A, v, axis)));
const signNumber = (A, value) => A.pick(A.lt(value, 0), A.num(-1), A.num(1));

// The gap along unit n: n · d less each segment's L |n · a| and each disc's R (1 - (n · b)²)^½ and the balls' r.
function gapAlong(A, n, { d, segments, discs, radius }) {
    const reach = [...segments.map(({ length, axis }) => A.mul(length, A.abs(dot3(A, n, axis)))),
        ...discs.map(({ radius: R, axis }) => { const c = dot3(A, n, axis); return A.mul(R, A.sqrt(A.max(0, A.sub(1, A.mul(c, c))))); })];
    return A.sub(A.sub(dot3(A, n, d), A.add(...reach)), radius);
}

// The direction on the circle square to the unit `axis` where the gap (`terms`) is largest, by Newton's steps from
// `starts`, each turning n by at most half a radian. Each search keeps the way each segment faces its start: within an
// arc of the circle between two creases of the gap (where a segment turns square to n) the gap is smooth, so the search
// settles on that arc's largest, however near a crease it lies (a start in every arc; the creases themselves are
// candidates of their own). Where it settles beyond its arc, its gap read there is less, and the arc's largest is at
// a crease. The best of the starts' ends.
function circleSearch(A, axis, terms, starts, steps, tag) {
    const { d, segments, discs } = terms;
    // Along the circle's tangent t = axis × n, each u · t is n · (u × axis): those cross products once for the circle.
    const crossed = (u, label) => v3.name(A, `${tag}${label}`, v3.cross(A, u, axis));
    const dx = crossed(d, 'x');
    const ex = segments.map(({ axis: e }, j) => crossed(e, `e${j}`));
    const bx = discs.map(({ axis: b }, j) => crossed(b, `c${j}`));
    const ends = starts.map((start, k) => {
        let n = start;
        // Each segment's reach, L |n · e|, as L σ n · e with σ its start's.
        const facing = segments.map(({ length, axis: e }, j) => A.name(`${tag}${k}f${j}`, A.mul(length, signNumber(A, dot3(A, start, e)))));
        const reachAxis = v3.name(A, `${tag}${k}r`, segments.reduce((sum, { axis: e }, j) => v3.add(A, sum, v3.scale(A, e, facing[j])), [A.num(0), A.num(0), A.num(0)]));
        const reachCrossed = v3.name(A, `${tag}${k}q`, segments.reduce((sum, _, j) => v3.add(A, sum, v3.scale(A, ex[j], facing[j])), [A.num(0), A.num(0), A.num(0)]));
        for (let i = 0; i < steps; i += 1) {
            let slope = A.sub(dot3(A, n, dx), dot3(A, n, reachCrossed));
            let bend = A.add(A.neg(dot3(A, n, d)), dot3(A, n, reachAxis));
            discs.forEach(({ radius, axis: b }, j) => {
                const c = A.name(`${tag}${k}c${j}x${i}`, dot3(A, n, b));
                const ct = A.name(`${tag}${k}e${j}x${i}`, dot3(A, n, bx[j]));
                const s = A.name(`${tag}${k}s${j}x${i}`, A.sqrt(A.max(1e-24, A.sub(1, A.mul(c, c)))));
                slope = A.add(slope, A.div(A.mul(radius, c, ct), s));
                bend = A.add(bend, A.mul(radius, A.add(A.div(A.sub(A.mul(ct, ct), A.mul(c, c)), s), A.div(A.mul(c, c, ct, ct), A.mul(s, s, s)))));
            });
            slope = A.name(`${tag}${k}g${i}`, slope);
            bend = A.name(`${tag}${k}h${i}`, bend);
            // Newton's step where the gap curves down; elsewhere its curvature shifted down by ten times its slope (a
            // step uphill in proportion to the slope, not a fixed one, which leaps past a direction near a cap's normal).
            const curving = A.lt(bend, A.mul(-1e-3, A.abs(slope)));
            const damped = A.pick(curving, bend, A.sub(A.neg(A.abs(bend)), A.add(A.mul(10, A.abs(slope)), 1e-12)));
            const step = A.name(`${tag}${k}d${i}`, clip(A, A.div(A.neg(slope), damped), -0.5, 0.5));
            // n + step t is √(1 + step²) long, t being a unit square to n.
            const t = v3.cross(A, axis, n);
            const length = A.sqrt(A.add(1, A.mul(step, step)));
            n = v3.name(A, `${tag}${k}n${i}`, v3.scale(A, v3.add(A, n, v3.scale(A, t, step)), A.div(1, length)));
        }
        return n;
    });
    let best = ends[0];
    let bestScore = A.name(`${tag}v0`, gapAlong(A, ends[0], terms));
    ends.slice(1).forEach((n, k) => {
        const value = A.name(`${tag}v${k + 1}`, gapAlong(A, n, terms));
        const better = A.name(`${tag}b${k + 1}`, A.gt(value, bestScore));
        best = v3.name(A, `${tag}w${k + 1}`, v3.pick(A, better, n, best));
        bestScore = A.name(`${tag}u${k + 1}`, A.pick(better, value, bestScore));
    });
    return best;
}

// The direction where the gap (`terms`) is largest anywhere on the sphere, by Newton's steps in the plane square to n
// from `starts` (each step at most half a radian; uphill, half a radian, where the gap is not curving down), each search
// keeping one way for each segment to face (as circleSearch does, so it settles beside a crease): a start's own, or
// the signs given with it ({ start, signs }) for a search of that region however its start faces; the best of the
// starts' ends. Each disc's reach is R |n - (n · b) b|; along unit tangents t₁, t₂ square to n its slope is
// -R (t · b) (n · b) / s and its curvature -R ((t₁ · t₂) - (t₁ · b)(t₂ · b)) / s + R (t₁ · b)(t₂ · b)(n · b)² / s³,
// s = (1 - (n · b)²)^½.
function sphereSearch(A, terms, starts, steps, tag) {
    const { d, segments, discs } = terms;
    const ends = starts.map((given, k) => {
        const start = Array.isArray(given) ? given : given.start;
        let n = start;
        const facing = segments.map(({ length, axis: e }, j) => A.name(`${tag}${k}f${j}`,
            A.mul(length, Array.isArray(given) ? signNumber(A, dot3(A, start, e)) : A.num(given.signs[j]))));
        const toward = v3.name(A, `${tag}${k}w`, segments.reduce((sum, { axis: e }, j) => v3.sub(A, sum, v3.scale(A, e, facing[j])), d));
        for (let i = 0; i < steps; i += 1) {
            const name = (label, value) => A.name(`${tag}${k}${label}${i}`, value);
            const helper = v3.pick(A, A.lt(A.abs(n[0]), 0.9), [A.num(1), A.num(0), A.num(0)], [A.num(0), A.num(1), A.num(0)]);
            const t1 = normalize(A, `${tag}${k}t${i}`, square(A, helper, n)).unit;
            const t2 = v3.name(A, `${tag}${k}T${i}`, v3.cross(A, n, t1));
            let g1 = dot3(A, t1, toward);
            let g2 = dot3(A, t2, toward);
            let along = dot3(A, n, toward);
            let h11 = A.num(0);
            let h12 = A.num(0);
            let h22 = A.num(0);
            discs.forEach(({ radius, axis: b }, j) => {
                const c = name(`c${j}x`, dot3(A, n, b));
                const p1 = name(`p${j}x`, dot3(A, t1, b));
                const p2 = name(`q${j}x`, dot3(A, t2, b));
                const size = name(`s${j}x`, A.sqrt(A.max(1e-24, A.sub(1, A.mul(c, c)))));
                const cube = A.mul(size, size, size);
                g1 = A.add(g1, A.div(A.mul(radius, p1, c), size));
                g2 = A.add(g2, A.div(A.mul(radius, p2, c), size));
                along = A.sub(along, A.mul(radius, size));
                h11 = A.sub(h11, A.mul(radius, A.sub(A.div(A.sub(1, A.mul(p1, p1)), size), A.div(A.mul(p1, p1, c, c), cube))));
                h12 = A.sub(h12, A.mul(radius, A.sub(A.div(A.neg(A.mul(p1, p2)), size), A.div(A.mul(p1, p2, c, c), cube))));
                h22 = A.sub(h22, A.mul(radius, A.sub(A.div(A.sub(1, A.mul(p2, p2)), size), A.div(A.mul(p2, p2, c, c), cube))));
            });
            g1 = name('g1', g1);
            g2 = name('g2', g2);
            along = name('nG', along);
            h11 = name('h11', A.sub(h11, along));
            h12 = name('h12', h12);
            h22 = name('h22', A.sub(h22, along));
            // Newton's step where the gap curves down every way; elsewhere its curvature shifted down past its largest by
            // ten times its slope (Levenberg and Marquardt's), so the step uphill is in proportion; at most half a radian.
            const gradientSize = name('gs', A.sqrt(A.add(A.mul(g1, g1), A.mul(g2, g2))));
            const largest = A.add(A.mul(0.5, A.add(h11, h22)), A.sqrt(A.add(A.mul(0.25, A.mul(A.sub(h11, h22), A.sub(h11, h22))), A.mul(h12, h12))));
            const shift = name('hm', A.max(0, A.add(largest, A.mul(10, gradientSize), 1e-12)));
            const k11 = A.sub(h11, shift);
            const k22 = A.sub(h22, shift);
            const det = name('hd', A.sub(A.mul(k11, k22), A.mul(h12, h12)));
            const x1 = name('x1', A.div(A.neg(A.sub(A.mul(k22, g1), A.mul(h12, g2))), det));
            const x2 = name('x2', A.div(A.neg(A.sub(A.mul(k11, g2), A.mul(h12, g1))), det));
            const stepSize = A.sqrt(A.add(A.mul(x1, x1), A.mul(x2, x2)));
            const scale = A.div(0.5, A.max(0.5, stepSize));
            const u1 = name('u1', A.mul(x1, scale));
            const u2 = name('u2', A.mul(x2, scale));
            const length = A.sqrt(A.add(1, A.mul(u1, u1), A.mul(u2, u2)));
            n = v3.name(A, `${tag}${k}n${i}`, v3.scale(A, v3.add(A, n, v3.add(A, v3.scale(A, t1, u1), v3.scale(A, t2, u2))), A.div(1, length)));
        }
        return n;
    });
    let best = ends[0];
    let bestScore = A.name(`${tag}v0`, gapAlong(A, ends[0], terms));
    ends.slice(1).forEach((n, k) => {
        const value = A.name(`${tag}v${k + 1}`, gapAlong(A, n, terms));
        const better = A.name(`${tag}b${k + 1}`, A.gt(value, bestScore));
        best = v3.name(A, `${tag}x${k + 1}`, v3.pick(A, better, n, best));
        bestScore = A.name(`${tag}y${k + 1}`, A.pick(better, value, bestScore));
    });
    return best;
}

// Starting directions on the circle square to unit `axis` just either side of where it crosses the plane square to
// unit `other` (a crease of the gap: n · other changes sign), or none where `other` lies along `axis`.
function besideCrease(A, axis, other, fallback, tag) {
    const crease = normalize(A, `${tag}k`, v3.cross(A, axis, other));
    const at = v3.pick(A, A.lt(crease.length, 1e-9), fallback, crease.unit);
    return [1, -1].flatMap((sign, m) => {
        const point = v3.scale(A, at, sign);
        const t = v3.cross(A, axis, point);
        return [0.05, -0.05].map((step, j) => normalize(A, `${tag}${m}${j}`, v3.add(A, point, v3.scale(A, t, step))).unit);
    });
}

// Of `candidates` ({ n, valid, pos }, each n a unit direction from the first shape to the second and pos its contact
// point), the one of largest gap (the first of those within a nanometre of it): { gap, chosen, n, pos }.
function largestGap(A, candidates, terms, tag) {
    let gap = A.num(-1e30);
    let chosen = candidates.map(() => A.never);
    candidates.forEach(({ n, valid }, c) => {
        const value = A.name(`${tag}v${c + 1}`, gapAlong(A, n, terms));
        // Larger by more than a nanometre: of directions as good as that (a cap's or a face's normal, and a search
        // that ended beside it), the first, so whether a contact lies flat turns on the shapes, not on rounding.
        const better = A.name(`${tag}q${c + 1}`, A.and(valid, A.gt(value, A.add(gap, 1e-9))));
        chosen = chosen.map((was, k) => (k === c ? better : A.and(was, A.not(better))));
        gap = A.name(`${tag}m${c + 1}`, A.pick(better, value, gap));
    });
    chosen = chosen.map((flag, c) => A.name(`${tag}k${c + 1}`, flag));
    const pickOf = (values) => values.reduceRight((rest, value, c) => A.pick(chosen[c], value, rest), values[values.length - 1]);
    return {
        gap, chosen,
        n: v3.name(A, `${tag}n`, [0, 1, 2].map((axis) => pickOf(candidates.map(({ n }) => n[axis])))),
        pos: v3.name(A, `${tag}p`, [0, 1, 2].map((axis) => pickOf(candidates.map(({ pos }) => pos[axis]))))
    };
}

// A cylinder's point reaching furthest along -n (toward a shape on n's far side): on its rim, or (n along its axis) its
// cap's centre.
function cylinderFacing(A, cylinder, n, tag) {
    const b = column(cylinder.mat, 2);
    const nb = dot3(A, n, b);
    const across = normalize(A, `${tag}r`, square(A, n, b));
    const radial = v3.pick(A, A.lt(across.length, 1e-12), [A.num(0), A.num(0), A.num(0)], across.unit);
    return v3.name(A, `${tag}f`, v3.sub(A, v3.sub(A, cylinder.pos, v3.scale(A, b, A.mul(signNumber(A, nb), cylinder.size[1]))), v3.scale(A, radial, cylinder.size[0])));
}

// A capsule against a cylinder: the deepest contact, exactly; and its other end's ball against the cylinder, where it
// touches too (a capsule lying along the side or across a cap), as MuJoCo gives a second contact there.
function capsuleCylinder(A, capsule, cylinder, margin) {
    const a = column(capsule.mat, 2);
    const b = column(cylinder.mat, 2);
    const L = capsule.size[1];
    const r = capsule.size[0];
    const H = cylinder.size[1];
    const R = cylinder.size[0];
    const d = v3.name(A, 'ycd', v3.sub(A, cylinder.pos, capsule.pos));
    const terms = { d, segments: [{ length: L, axis: a }, { length: H, axis: b }], discs: [{ radius: R, axis: b }], radius: r };
    const along = (n, t) => v3.add(A, v3.add(A, capsule.pos, v3.scale(A, a, t)), v3.scale(A, n, r));
    const midway = (p, q) => v3.scale(A, v3.add(A, p, q), 0.5);
    const endToward = (n) => A.mul(signNumber(A, dot3(A, n, a)), L);
    const candidates = [];
    // Face on to a cap: the capsule's end (or, lying square to the cap, its point over the cap's centre) on the cap.
    [1, -1].forEach((sign, k) => {
        const n = v3.scale(A, b, sign);
        const capCentre = v3.sub(A, cylinder.pos, v3.scale(A, b, A.mul(sign, H)));
        const flat = A.lt(A.abs(dot3(A, n, a)), 1e-12);
        const t = A.name(`yct${k}`, A.pick(flat, clip(A, dot3(A, v3.sub(A, capCentre, capsule.pos), a), A.neg(L), L), endToward(n)));
        const p = v3.name(A, `yca${k}`, along(n, t));
        const q = v3.sub(A, p, v3.scale(A, n, dot3(A, v3.sub(A, p, capCentre), n)));
        candidates.push({ n, valid: A.always, pos: midway(p, q) });
    });
    // Square to both axes: the capsule's segment against the cylinder's side, their lines' nearest points.
    const crossed = normalize(A, 'ycx', v3.cross(A, a, b));
    [1, -1].forEach((sign, k) => {
        const n = v3.name(A, `ycxn${k}`, v3.scale(A, crossed.unit, sign));
        const line = v3.sub(A, cylinder.pos, v3.scale(A, n, R));
        const w = v3.sub(A, capsule.pos, line);
        const ab = dot3(A, a, b);
        const den = A.sub(1, A.mul(ab, ab));
        const safe = A.pick(A.lt(den, minimumValue), A.num(1), den);
        const aw = dot3(A, a, w);
        const bw = dot3(A, b, w);
        const t = clip(A, A.div(A.sub(A.mul(ab, bw), aw), safe), A.neg(L), L);
        const u = clip(A, A.div(A.sub(bw, A.mul(ab, aw)), safe), A.neg(H), H);
        candidates.push({ n, valid: A.ge(crossed.length, 1e-9), pos: midway(along(n, t), v3.add(A, line, v3.scale(A, b, u))) });
    });
    // Square to the cylinder's axis: the capsule's end against the cylinder's side.
    [1, -1].forEach((sign, k) => {
        const w = v3.sub(A, d, v3.scale(A, a, A.mul(sign, L)));
        const n = normalize(A, `ycs${k}`, square(A, w, b)).unit;
        const p = v3.name(A, `ycsa${k}`, along(n, A.mul(sign, L)));
        const q = v3.add(A, v3.sub(A, cylinder.pos, v3.scale(A, n, R)), v3.scale(A, b, clip(A, dot3(A, v3.sub(A, p, cylinder.pos), b), A.neg(H), H)));
        candidates.push({ n, valid: A.always, pos: midway(p, q) });
    });
    // Smooth: the capsule's end against the rim, for each way the axes may point; n in the plane of w and b.
    [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sa, sb], k) => {
        const w = v3.sub(A, v3.sub(A, d, v3.scale(A, a, A.mul(sa, L))), v3.scale(A, b, A.mul(sb, H)));
        const across = normalize(A, `ycm${k}`, square(A, w, b));
        const wu = across.length;
        const wb = dot3(A, w, b);
        const n = normalize(A, `ycmn${k}`, v3.add(A, v3.scale(A, across.unit, A.sub(wu, R)), v3.scale(A, b, wb))).unit;
        const p = v3.name(A, `ycma${k}`, along(n, endToward(n)));
        candidates.push({ n, valid: A.gt(wu, R), pos: midway(p, cylinderFacing(A, cylinder, n, `ycmf${k}`)) });
    });
    // The capsule's segment lying square to n against the rim: Newton's steps on the circle square to its axis, from
    // where the cylinder's centre is (from each end of its axis) and from where its axis points.
    const fallback = normalize(A, 'ycwf', square(A, [A.num(1), A.num(0), A.num(0)], a)).unit;
    const startOf = (v, tag) => { const u = normalize(A, tag, square(A, v, a)); return v3.pick(A, A.lt(u.length, 1e-9), fallback, u.unit); };
    const rimward = startOf(b, 'ycwb');
    const starts = [startOf(d, 'ycw0'), rimward, v3.scale(A, rimward, -1), ...besideCrease(A, a, b, fallback, 'ycwk')];
    {
        const n = circleSearch(A, a, terms, starts, 14, 'ycc');
        const q = cylinderFacing(A, cylinder, n, 'yccf');
        const p = along(n, clip(A, dot3(A, v3.sub(A, q, capsule.pos), a), A.neg(L), L));
        candidates.push({ n, valid: A.always, pos: midway(p, q) });
    }
    const deepest = largestGap(A, candidates, terms, 'ycl');
    const dist = A.name('ycdist', deepest.gap);
    // The other end: the one further along the axis from the deepest contact.
    const far = A.mul(A.neg(signNumber(A, dot3(A, v3.sub(A, deepest.pos, capsule.pos), a))), L);
    const end = v3.name(A, 'yce', v3.add(A, capsule.pos, v3.scale(A, a, far)));
    const [second] = sphereCylinder(A, { pos: end, size: [r], mat: capsule.mat }, cylinder, margin);
    return [
        { active: A.le(dist, margin), dist, pos: deepest.pos, frame: [...deepest.n, ...noTangent(A)] },
        { ...second, active: A.and(A.le(dist, margin), second.active) }
    ];
}

// The same algebra, its names prefixed (to call a function of fixed names twice for one pair).
const prefixed = (A, prefix) => ({ ...A, name: (tag, value) => A.name(`${prefix}${tag}`, value) });
// A plane through `pos` facing unit `normal`, as the floor functions take one.
const planeAt = (A, pos, normal) => ({ pos, mat: [A.num(0), A.num(0), normal[0], A.num(0), A.num(0), normal[1], A.num(0), A.num(0), normal[2]] });
const pointOf = (A, slot) => ({ active: slot.active, dist: slot.dist, pos: slot.pos });

// The part of the line from p to q (each a contact point, its dist) within `inside(t)`'s interval of t from 0 to 1,
// as its two ends: [enter, leave], each at its own dist, both off where the interval is empty.
function clipped(A, p, q, low, high, tag) {
    const at = (t, label) => ({
        dist: A.name(`${tag}${label}d`, A.add(p.dist, A.mul(t, A.sub(q.dist, p.dist)))),
        pos: v3.name(A, `${tag}${label}p`, v3.add(A, p.pos, v3.scale(A, v3.sub(A, q.pos, p.pos), t)))
    });
    const some = A.le(low, high);
    return [{ ...at(low, 'a'), active: A.and(p.active, some) }, { ...at(high, 'b'), active: A.and(p.active, q.active, some, A.gt(A.sub(high, low), 1e-12)) }];
}

// The interval of t in [0, 1] where p + t (q - p) lies within the rectangle of half sizes su, sv along U, V about
// `centre` (in its plane).
function rectInterval(A, p, q, centre, U, V, su, sv, tag) {
    let low = A.num(0);
    let high = A.num(1);
    [[U, su], [V, sv]].forEach(([axis, half], k) => {
        const x0 = A.name(`${tag}x${k}`, dot3(A, v3.sub(A, p, centre), axis));
        const dx = A.name(`${tag}e${k}`, dot3(A, v3.sub(A, q, p), axis));
        const moving = A.gt(A.abs(dx), 1e-15);
        const safe = A.pick(moving, dx, A.num(1));
        const ta = A.div(A.sub(A.neg(half), x0), safe);
        const tb = A.div(A.sub(half, x0), safe);
        const outside = A.gt(A.abs(x0), half);
        low = A.name(`${tag}l${k}`, A.pick(moving, A.max(low, A.min(ta, tb)), A.pick(outside, A.num(2), low)));
        high = A.name(`${tag}h${k}`, A.pick(moving, A.min(high, A.max(ta, tb)), high));
    });
    return [low, high];
}

// The interval of t in [0, 1] where p + t (q - p), projected along unit `axis`, lies within radius R of `centre`.
function discInterval(A, p, q, centre, axis, R, tag) {
    const r0 = v3.name(A, `${tag}r`, square(A, v3.sub(A, p, centre), axis));
    const e = v3.name(A, `${tag}e`, square(A, v3.sub(A, q, p), axis));
    const ee = A.name(`${tag}a`, dot3(A, e, e));
    const re = A.name(`${tag}b`, dot3(A, r0, e));
    const rr = A.sub(dot3(A, r0, r0), A.mul(R, R));
    const moving = A.gt(ee, 1e-24);
    const safe = A.pick(moving, ee, A.num(1));
    const disc = A.name(`${tag}c`, A.sub(A.mul(re, re), A.mul(ee, rr)));
    const root = A.sqrt(A.max(0, disc));
    const low = A.pick(moving, A.pick(A.ge(disc, 0), A.max(0, A.div(A.sub(A.neg(re), root), safe)), A.num(2)), A.pick(A.le(rr, 0), A.num(0), A.num(2)));
    const high = A.pick(moving, A.min(1, A.div(A.add(A.neg(re), root), safe)), A.num(1));
    return [A.name(`${tag}l`, low), A.name(`${tag}h`, high)];
}

// Whether a point lies over the rectangle (within its half sizes along U and V about `centre`) and within `depth` of
// its plane on the far side (inside the solid the rectangle is a face of).
const overRect = (A, pos, dist, centre, U, V, su, sv, depth) => {
    const r = v3.sub(A, pos, centre);
    return A.and(A.le(A.abs(dot3(A, r, U)), su), A.le(A.abs(dot3(A, r, V)), sv), A.ge(dist, A.neg(depth)));
};
const overDisc = (A, pos, dist, centre, axis, R, depth) => {
    const r = square(A, v3.sub(A, pos, centre), axis);
    return A.and(A.le(dot3(A, r, r), A.mul(R, R)), A.ge(dist, A.neg(depth)));
};

// A cylinder lying on, standing on or tipped against a flat face (a plane through `centre` facing unit `out`, toward the
// cylinder): MuJoCo's plane against cylinder there (two points along its side, or three on its rim), the line between
// the first two kept where it lies `within` the face (an interval of t from `interval`), the other two where `over` it.
// Each point's normal is -out (from the cylinder to the face).
function cylinderOnFace(A, cylinder, centre, out, margin, interval, over, tag) {
    const B = prefixed(A, tag);
    const slots = planeCylinder(B, planeAt(A, centre, out), cylinder, margin);
    const [low, high] = interval(slots[0].pos, slots[1].pos, `${tag}i`);
    const line = clipped(A, pointOf(A, slots[0]), { ...pointOf(A, slots[1]), active: A.always }, low, high, `${tag}k`);
    const rim = slots.slice(2).map((slot) => ({ ...pointOf(A, slot), active: A.and(slot.active, over(slot.pos, slot.dist)) }));
    return [...line, ...rim].map((point) => ({ ...point, active: A.and(point.active, A.le(point.dist, margin)), normal: v3.scale(A, out, -1) }));
}

// The contact slots for a pair whose deepest contact may lie flat: where `flat` holds and any of `points` touches, the
// first four of `points` that touch (each { active, dist, pos, normal }); otherwise the deepest alone, in the first.
function flatOrDeepest(A, flat, points, deepest, dist, margin, tag) {
    const anyFlat = A.name(`${tag}f`, A.and(flat, A.or(...points.map(({ active }) => active))));
    const before = [A.num(0)];
    points.forEach(({ active }, p) => before.push(A.name(`${tag}c${p + 1}`, A.add(before[p], A.pick(active, 1, 0)))));
    return [0, 1, 2, 3].map((k) => {
        const takers = points.map(({ active }, p) => A.name(`${tag}t${k + 1}x${p + 1}`, A.and(active, A.gt(before[p], k - 0.5), A.lt(before[p], k + 0.5))));
        const pickAll = (values) => values.reduceRight((rest, value, p) => A.pick(takers[p], value, rest), values[values.length - 1]);
        const one = k === 0;
        const flatDist = pickAll(points.map((point) => point.dist));
        const flatPos = [0, 1, 2].map((i) => pickAll(points.map((point) => point.pos[i])));
        const flatNormal = [0, 1, 2].map((i) => pickAll(points.map((point) => point.normal[i])));
        return {
            active: A.name(`${tag}s${k + 1}a`, A.pick(anyFlat, A.or(...takers), one ? A.le(dist, margin) : A.never)),
            dist: A.name(`${tag}s${k + 1}d`, one ? A.pick(anyFlat, flatDist, dist) : flatDist),
            pos: v3.name(A, `${tag}s${k + 1}p`, one ? v3.pick(A, anyFlat, flatPos, deepest.pos) : flatPos),
            frame: [...v3.name(A, `${tag}s${k + 1}n`, one ? v3.pick(A, anyFlat, flatNormal, deepest.n) : flatNormal), ...noTangent(A)]
        };
    });
}

// A cylinder against a box: the deepest contact, exactly.
function cylinderBox(A, cylinder, box, margin) {
    const b = column(cylinder.mat, 2);
    const H = cylinder.size[1];
    const R = cylinder.size[0];
    const E = [0, 1, 2].map((k) => column(box.mat, k));
    const s = box.size;
    const d = v3.name(A, 'ybd', v3.sub(A, box.pos, cylinder.pos));
    const terms = { d, segments: [{ length: H, axis: b }, ...E.map((axis, k) => ({ length: s[k], axis }))], discs: [{ radius: R, axis: b }], radius: A.num(0) };
    const midway = (p, q) => v3.scale(A, v3.add(A, p, q), 0.5);
    const reaching = (n, tag) => cylinderFacing(A, cylinder, v3.scale(A, n, -1), tag);
    // The box's corner reaching furthest along -n, for the signs given (each +1 where n · e_k > 0).
    const cornerOf = (signs) => E.reduce((sum, axis, k) => v3.sub(A, sum, v3.scale(A, axis, A.mul(signs[k], s[k]))), box.pos);
    const signsAlong = (n) => E.map((axis) => signNumber(A, dot3(A, n, axis)));
    const candidates = [];
    // A face of the box: the cylinder's point reaching furthest, on the face's plane.
    E.forEach((axis, k) => [1, -1].forEach((sign, m) => {
        const n = v3.scale(A, axis, sign);
        const p = reaching(n, `ybfp${k}${m}`);
        const plane = v3.sub(A, box.pos, v3.scale(A, n, s[k]));
        candidates.push({ n, valid: A.always, pos: midway(p, v3.sub(A, p, v3.scale(A, n, dot3(A, v3.sub(A, p, plane), n)))) });
    }));
    // A cap of the cylinder: the box's corner reaching furthest, on the cap's plane.
    [1, -1].forEach((sign, m) => {
        const n = v3.scale(A, b, sign);
        const q = v3.name(A, `ybcq${m}`, cornerOf(signsAlong(n)));
        const cap = v3.add(A, cylinder.pos, v3.scale(A, b, A.mul(sign, H)));
        candidates.push({ n, valid: A.always, pos: midway(v3.sub(A, q, v3.scale(A, n, dot3(A, v3.sub(A, q, cap), n))), q) });
    });
    // The cylinder's side against an edge of the box: their lines' nearest points.
    E.forEach((axis, k) => {
        const crossed = normalize(A, `ybx${k}`, v3.cross(A, b, axis));
        [1, -1].forEach((sign, m) => {
            const n = v3.name(A, `ybxn${k}${m}`, v3.scale(A, crossed.unit, sign));
            const line = v3.add(A, cylinder.pos, v3.scale(A, n, R));
            const signs = signsAlong(n).map((value, j) => (j === k ? A.num(0) : value));
            const edge = v3.name(A, `ybxe${k}${m}`, cornerOf(signs));
            const w = v3.sub(A, line, edge);
            const be = dot3(A, b, axis);
            const den = A.sub(1, A.mul(be, be));
            const safe = A.pick(A.lt(den, minimumValue), A.num(1), den);
            const bw = dot3(A, b, w);
            const ew = dot3(A, axis, w);
            const t = clip(A, A.div(A.sub(A.mul(be, ew), bw), safe), A.neg(H), H);
            const u = clip(A, A.div(A.sub(ew, A.mul(be, bw)), safe), A.neg(s[k]), s[k]);
            candidates.push({ n, valid: A.ge(crossed.length, 1e-9), pos: midway(v3.add(A, line, v3.scale(A, b, t)), v3.add(A, edge, v3.scale(A, axis, u))) });
        });
    });
    // Each corner of the box against the cylinder's side (n square to its axis) and against its rim (smooth).
    for (let corner = 0; corner < 8; corner += 1) {
        const signs = [0, 1, 2].map((k) => ((corner >> k) & 1 ? -1 : 1));
        const w0 = E.reduce((sum, axis, k) => v3.sub(A, sum, v3.scale(A, axis, A.mul(signs[k], s[k]))), d);
        const q = v3.name(A, `ybvq${corner}`, cornerOf(signs.map((value) => A.num(value))));
        const side = normalize(A, `ybvs${corner}`, square(A, w0, b)).unit;
        const onSide = v3.add(A, v3.add(A, cylinder.pos, v3.scale(A, side, R)), v3.scale(A, b, clip(A, dot3(A, v3.sub(A, q, cylinder.pos), b), A.neg(H), H)));
        candidates.push({ n: side, valid: A.always, pos: midway(onSide, q) });
        [1, -1].forEach((sb, m) => {
            const w = v3.sub(A, w0, v3.scale(A, b, A.mul(sb, H)));
            const across = normalize(A, `ybvm${corner}${m}`, square(A, w, b));
            const n = normalize(A, `ybvn${corner}${m}`, v3.add(A, v3.scale(A, across.unit, A.sub(across.length, R)), v3.scale(A, b, dot3(A, w, b)))).unit;
            candidates.push({ n, valid: A.gt(across.length, R), pos: midway(reaching(n, `ybvr${corner}${m}`), q) });
        });
    }
    // An edge of the box lying square to n against the rim: Newton's steps on the circle square to it.
    E.forEach((axis, k) => {
        const fallback = normalize(A, `ybef${k}`, square(A, [A.num(1), A.num(0), A.num(0)], axis)).unit;
        const startOf = (v, tag) => { const u = normalize(A, tag, square(A, v, axis)); return v3.pick(A, A.lt(u.length, 1e-9), fallback, u.unit); };
        // The middle of each quarter between the box's faces' normals, and either side of the cylinder's crease.
        const [u, v] = [0, 1, 2].filter((j) => j !== k).map((j) => E[j]);
        const diagonals = [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([p, q], m) => normalize(A, `ybeq${k}${m}`, v3.add(A, v3.scale(A, u, p), v3.scale(A, v, q))).unit);
        // And the circle's directions nearest the cylinder's axis, where a cap is nearly face on and the gap is steep.
        const rimward = startOf(b, `ybeb${k}`);
        const starts = [...diagonals, rimward, v3.scale(A, rimward, -1), ...besideCrease(A, axis, b, fallback, `ybek${k}`)];
        const n = circleSearch(A, axis, terms, starts, 14, `ybe${k}c`);
        const p = reaching(n, `ybep${k}`);
        const signs = signsAlong(n).map((value, j) => (j === k ? A.num(0) : value));
        const edge = cornerOf(signs);
        const q = v3.add(A, edge, v3.scale(A, axis, clip(A, dot3(A, v3.sub(A, p, edge), axis), A.neg(s[k]), s[k])));
        candidates.push({ n, valid: A.always, pos: midway(p, q) });
    });
    const deepest = largestGap(A, candidates, terms, 'ybl');
    const dist = A.name('ybdist', deepest.gap);
    // Deepest along a face's normal (candidates 1 to 6): the cylinder on that face. Along a cap's normal (7 and 8): the
    // box's corners over the cap, and the cylinder's rim over the box's face turned most toward it. Otherwise the
    // deepest alone. The faces in the candidates' order: each axis, + then -.
    const faces = E.flatMap((axis, k) => [1, -1].map((sign) => ({ out: v3.scale(A, axis, -sign), k })));
    const faceChosen = deepest.chosen.slice(0, 6);
    const capChosen = deepest.chosen.slice(6, 8);
    const pickFace = (flags, values) => values.reduceRight((rest, value, c) => A.pick(flags[c], value, rest), values[values.length - 1]);
    const faceData = (flags, tag) => {
        const out = v3.name(A, `${tag}o`, [0, 1, 2].map((i) => pickFace(flags, faces.map(({ out: o }) => o[i]))));
        const sk = A.name(`${tag}k`, pickFace(flags, faces.map(({ k }) => s[k])));
        const others = [[1, 2], [0, 2], [0, 1]];
        const U = v3.name(A, `${tag}u`, [0, 1, 2].map((i) => pickFace(flags, faces.map(({ k }) => E[others[k][0]][i]))));
        const V = v3.name(A, `${tag}v`, [0, 1, 2].map((i) => pickFace(flags, faces.map(({ k }) => E[others[k][1]][i]))));
        const su = A.name(`${tag}su`, pickFace(flags, faces.map(({ k }) => s[others[k][0]])));
        const sv = A.name(`${tag}sv`, pickFace(flags, faces.map(({ k }) => s[others[k][1]])));
        const centre = v3.name(A, `${tag}c`, v3.add(A, box.pos, v3.scale(A, out, sk)));
        return { out, centre, U, V, su, sv, depth: A.mul(2, sk) };
    };
    const onFace = (face, tag) => cylinderOnFace(A, cylinder, face.centre, face.out, margin,
        (p, q, label) => rectInterval(A, p, q, face.centre, face.U, face.V, face.su, face.sv, label),
        (pos, d2) => overRect(A, pos, d2, face.centre, face.U, face.V, face.su, face.sv, face.depth), tag);
    const flatFace = onFace(faceData(faceChosen, 'ybff'), 'ybfa');
    // The box's face turned most toward the cylinder (its outward normal most against the deepest's).
    let most = A.num(1e30);
    let turned = faces.map(() => A.never);
    faces.forEach(({ out }, c) => {
        const value = A.name(`ybtv${c + 1}`, dot3(A, out, deepest.n));
        const nearer = A.name(`ybtq${c + 1}`, A.lt(value, most));
        turned = turned.map((was, k) => (k === c ? nearer : A.and(was, A.not(nearer))));
        most = A.name(`ybtm${c + 1}`, A.pick(nearer, value, most));
    });
    const rimOnBox = onFace(faceData(turned.map((flag, c) => A.name(`ybtk${c + 1}`, flag)), 'ybtf'), 'ybta');
    // The cap facing the box, and the box's corners over it.
    const capSign = A.name('ybcs', signNumber(A, dot3(A, deepest.n, b)));
    const capOut = v3.name(A, 'ybco', v3.scale(A, b, capSign));
    const capCentre = v3.name(A, 'ybcc', v3.add(A, cylinder.pos, v3.scale(A, capOut, H)));
    const corners = planeBox(prefixed(A, 'ybcb'), planeAt(A, capCentre, capOut), box, margin)
        .map((slot) => ({ ...pointOf(A, slot), active: A.and(slot.active, overDisc(A, slot.pos, slot.dist, capCentre, b, R, A.mul(2, H))), normal: capOut }));
    const isFace = A.name('ybif', A.or(...faceChosen));
    const isCap = A.name('ybic', A.or(...capChosen));
    const off = (point) => ({ ...point, active: A.never });
    const points = [
        ...flatFace.map((point, k) => ({
            active: A.pick(isFace, point.active, A.pick(isCap, corners[k].active, A.never)),
            dist: A.pick(isFace, point.dist, corners[k].dist),
            pos: v3.pick(A, isFace, point.pos, corners[k].pos),
            normal: v3.pick(A, isFace, point.normal, corners[k].normal)
        })),
        // Only where that face is turned within 2.6 degrees of facing the cap (a big box resting on a small post).
        ...rimOnBox.map((point) => ({ ...point, active: A.and(isCap, A.lt(most, -0.999), point.active) }))
    ];
    return flatOrDeepest(A, A.or(isFace, isCap), points.map((point) => point ?? off(point)), deepest, dist, margin, 'ybz');
}

// A cylinder against a cylinder: the deepest contact, exactly.
function cylinderCylinder(A, first, second, margin) {
    const a = column(first.mat, 2);
    const b = column(second.mat, 2);
    const [R1, H1] = first.size;
    const [R2, H2] = second.size;
    const d = v3.name(A, 'yyd', v3.sub(A, second.pos, first.pos));
    const terms = { d, segments: [{ length: H1, axis: a }, { length: H2, axis: b }], discs: [{ radius: R1, axis: a }, { radius: R2, axis: b }], radius: A.num(0) };
    const midway = (p, q) => v3.scale(A, v3.add(A, p, q), 0.5);
    const reaching = (n, tag) => cylinderFacing(A, first, v3.scale(A, n, -1), tag);
    const facing = (n, tag) => cylinderFacing(A, second, n, tag);
    const onPlane = (p, origin, n) => v3.sub(A, p, v3.scale(A, n, dot3(A, v3.sub(A, p, origin), n)));
    const candidates = [];
    // A cap of either, face on.
    [1, -1].forEach((sign, m) => {
        const n = v3.scale(A, a, sign);
        const q = facing(n, `yyaq${m}`);
        candidates.push({ n, valid: A.always, pos: midway(onPlane(q, v3.add(A, first.pos, v3.scale(A, a, A.mul(sign, H1))), n), q) });
        const nb = v3.scale(A, b, sign);
        const p = reaching(nb, `yybp${m}`);
        candidates.push({ n: nb, valid: A.always, pos: midway(p, onPlane(p, v3.sub(A, second.pos, v3.scale(A, b, A.mul(sign, H2))), nb)) });
    });
    // The two sides: their lines' nearest points.
    const crossed = normalize(A, 'yyx', v3.cross(A, a, b));
    [1, -1].forEach((sign, m) => {
        const n = v3.name(A, `yyxn${m}`, v3.scale(A, crossed.unit, sign));
        const p0 = v3.add(A, first.pos, v3.scale(A, n, R1));
        const q0 = v3.sub(A, second.pos, v3.scale(A, n, R2));
        const w = v3.sub(A, p0, q0);
        const ab = dot3(A, a, b);
        const den = A.sub(1, A.mul(ab, ab));
        const safe = A.pick(A.lt(den, minimumValue), A.num(1), den);
        const aw = dot3(A, a, w);
        const bw = dot3(A, b, w);
        const t = clip(A, A.div(A.sub(A.mul(ab, bw), aw), safe), A.neg(H1), H1);
        const u = clip(A, A.div(A.sub(bw, A.mul(ab, aw)), safe), A.neg(H2), H2);
        candidates.push({ n, valid: A.ge(crossed.length, 1e-9), pos: midway(v3.add(A, p0, v3.scale(A, a, t)), v3.add(A, q0, v3.scale(A, b, u))) });
    });
    // One's side against the other's rim: Newton's steps on the circle square to its axis.
    const startsFor = (axis, tag) => {
        const fallback = normalize(A, `${tag}f`, square(A, [A.num(1), A.num(0), A.num(0)], axis)).unit;
        const startOf = (v, label) => { const u = normalize(A, `${tag}${label}`, square(A, v, axis)); return v3.pick(A, A.lt(u.length, 1e-9), fallback, u.unit); };
        const other = axis === a ? b : a;
        const H = axis === a ? H2 : H1;
        const rimward = startOf(other, 'r');
        return [startOf(d, 'd'), rimward, v3.scale(A, rimward, -1), ...besideCrease(A, axis, other, fallback, `${tag}k`)];
    };
    {
        const n = circleSearch(A, a, terms, startsFor(a, 'yysa'), 14, 'yyca');
        const q = facing(n, 'yycaq');
        candidates.push({ n, valid: A.always, pos: midway(v3.add(A, v3.add(A, first.pos, v3.scale(A, n, R1)), v3.scale(A, a, clip(A, dot3(A, v3.sub(A, q, first.pos), a), A.neg(H1), H1))), q) });
    }
    {
        const n = circleSearch(A, b, terms, startsFor(b, 'yysb'), 14, 'yycb');
        const p = reaching(n, 'yycbp');
        candidates.push({ n, valid: A.always, pos: midway(p, v3.add(A, v3.sub(A, second.pos, v3.scale(A, n, R2)), v3.scale(A, b, clip(A, dot3(A, v3.sub(A, p, second.pos), b), A.neg(H2), H2)))) });
    }
    // Rim against rim: Newton's steps on the sphere, from beside the best so far (which may be a cap's normal, where a
    // disc's reach has a point and no slope) and from each way the axes may point.
    const sofar = largestGap(A, candidates, terms, 'yyk');
    const helper = v3.pick(A, A.lt(A.abs(sofar.n[0]), 0.9), [A.num(1), A.num(0), A.num(0)], [A.num(0), A.num(1), A.num(0)]);
    const side1 = normalize(A, 'yyt', square(A, helper, sofar.n)).unit;
    const side2 = v3.cross(A, sofar.n, side1);
    const beside = [[side1, 0.02], [side1, -0.02], [side2, 0.02], [side2, -0.02]].map(([side, step], m) => normalize(A, `yyh${m}`, v3.add(A, sofar.n, v3.scale(A, side, step))).unit);
    // And from a thousandth of a radian off each cap's normal, the way the rest of the gap rises there (the largest
    // may lie a hair from where a disc is face on).
    const offCaps = [[a, 1, 0], [a, -1, 0], [b, 1, 1], [b, -1, 1]].map(([axis, sign, j], m) => {
        const tip = v3.scale(A, axis, sign);
        const other = j === 0 ? { length: H2, axis: b, radius: R2 } : { length: H1, axis: a, radius: R1 };
        // The rest of the gap's gradient at the tip: n · d less the other's segment and disc.
        const across = normalize(A, `yyq${m}`, square(A, tip, other.axis));
        const rest = v3.sub(A, v3.sub(A, d, v3.scale(A, other.axis, A.mul(other.length, signNumber(A, dot3(A, tip, other.axis))))), v3.scale(A, across.unit, other.radius));
        const rise = normalize(A, `yyc${m}`, square(A, rest, tip));
        return normalize(A, `yyo${m}`, v3.add(A, tip, v3.scale(A, v3.pick(A, A.lt(rise.length, 1e-12), side1, rise.unit), 1e-3))).unit;
    });
    const starts = [...beside, ...offCaps, ...[[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sa, sb], m) => ({
        start: normalize(A, `yyss${m}`, v3.sub(A, v3.sub(A, d, v3.scale(A, a, A.mul(sa, H1))), v3.scale(A, b, A.mul(sb, H2)))).unit, signs: [sa, sb]
    }))];
    {
        const n = sphereSearch(A, terms, starts, 14, 'yyr');
        candidates.push({ n, valid: A.always, pos: midway(reaching(n, 'yyrp'), facing(n, 'yyrq')) });
    }
    const deepest = largestGap(A, candidates, terms, 'yyl');
    const dist = A.name('yydist', deepest.gap);
    // Deepest along a cap's normal (candidates 1 to 4): each cylinder on the other's cap facing it, its points over
    // that cap (the second's dropped where the first's already has one); otherwise the deepest alone.
    const isCap = A.name('yyic', A.or(...deepest.chosen.slice(0, 4)));
    const signA = A.name('yysa', signNumber(A, dot3(A, deepest.n, a)));
    const signB = A.name('yysb', signNumber(A, dot3(A, deepest.n, b)));
    const outA = v3.name(A, 'yyoa', v3.scale(A, a, signA));
    const outB = v3.name(A, 'yyob', v3.scale(A, b, A.neg(signB)));
    const capA = v3.name(A, 'yyca', v3.add(A, first.pos, v3.scale(A, outA, H1)));
    const capB = v3.name(A, 'yycb', v3.add(A, second.pos, v3.scale(A, outB, H2)));
    const onA = cylinderOnFace(A, second, capA, outA, margin, (p, q, label) => discInterval(A, p, q, capA, a, R1, label),
        (pos, d2) => overDisc(A, pos, d2, capA, a, R1, A.mul(2, H1)), 'yypa').map((point) => ({ ...point, normal: outA }));
    const onB = cylinderOnFace(A, first, capB, outB, margin, (p, q, label) => discInterval(A, p, q, capB, b, R2, label),
        (pos, d2) => overDisc(A, pos, d2, capB, b, R2, A.mul(2, H2)), 'yypb');
    const repeated = (point) => A.or(...onA.map((other) => {
        const gap = v3.sub(A, point.pos, other.pos);
        return A.and(other.active, A.lt(dot3(A, gap, gap), 1e-14));
    }));
    // The second's cap only where it is turned within 2.6 degrees of facing the first's (one stacked on the other).
    const facingCaps = A.name('yyff', A.gt(A.abs(dot3(A, deepest.n, b)), 0.999));
    const facingFirst = A.name('yyfa', A.gt(A.abs(dot3(A, deepest.n, a)), 0.999));
    const points = [...onA.map((point) => ({ ...point, active: A.and(facingFirst, point.active) })),
        ...onB.map((point) => ({ ...point, active: A.and(facingCaps, point.active, A.not(repeated(point))) }))];
    return flatOrDeepest(A, isCap, points, deepest, dist, margin, 'yyz');
}


// ---- meshes: the floor, spheres and capsules against a mesh's convex hull ----------------------------------------
// MuJoCo collides a mesh as its convex hull (lib/hull.mjs: its corners, triangles, edges and polygons, in the mesh's
// own frame, given with the shape as `hull`, and `centre`, the mesh's centre in that frame). These work in that frame,
// where the hull is numbers: the other shape is brought into it, and what is found is taken back out.
//
// A sphere or a capsule against the hull, exactly, by the largest gap (as above): every direction where the gap is
// largest lies where one feature of the hull (a face, an edge or a corner) meets one of the other shape's (a ball, a
// capsule's end or its whole segment), and for each such pair the direction is found in closed form, and kept only
// where the hull's feature is indeed what reaches furthest along it (within its normal cone: a face's normal; between
// the normals of an edge's two faces; away from each of a corner's neighbours). A kept direction's gap is exact, so no
// more than the largest; and the largest is among them.

// The sum of picks of values, only one of whose conditions holds (or none: 0), in named partial sums of 64.
function pickOne(A, flags, values, tag) {
    const terms = flags.map((flag, c) => A.pick(flag, values[c], A.num(0)));
    if (terms.length <= 64) return A.add(...terms);
    const parts = [];
    for (let at = 0; at < terms.length; at += 64) parts.push(A.name(`${tag}${parts.length + 1}`, A.add(...terms.slice(at, at + 64))));
    return A.add(...parts);
}

// Of `candidates` ({ value, valid, n, pos }), the one of largest value (the first within a nanometre of it):
// { gap, chosen, n, pos }.
function bestOf(A, candidates, tag) {
    let gap = A.num(-1e30);
    const better = candidates.map(({ value, valid }, c) => {
        const flag = A.name(`${tag}q${c + 1}`, A.and(valid, A.gt(value, A.add(gap, 1e-9))));
        gap = A.name(`${tag}m${c + 1}`, A.pick(flag, value, gap));
        return flag;
    });
    // Chosen: better than all before it, and none after it better.
    const later = [];
    let none = A.always;
    for (let c = candidates.length - 1; c >= 0; c--) {
        later[c] = none;
        none = c > 0 && (c % 64 === 0) ? A.name(`${tag}l${c}`, A.and(none, A.not(better[c]))) : A.and(none, A.not(better[c]));
    }
    const chosen = better.map((flag, c) => A.name(`${tag}k${c + 1}`, A.and(flag, later[c])));
    return {
        gap, chosen,
        n: v3.name(A, `${tag}n`, [0, 1, 2].map((axis) => pickOne(A, chosen, candidates.map(({ n }) => n[axis]), `${tag}n${axis + 1}x`))),
        pos: v3.name(A, `${tag}p`, [0, 1, 2].map((axis) => pickOne(A, chosen, candidates.map(({ pos }) => pos[axis]), `${tag}p${axis + 1}x`)))
    };
}

const numberCross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const numberDot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const numberSub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const numberUnit = (a) => { const l = Math.hypot(...a); return a.map((x) => x / l); };
// A constant vector times an expression vector, and the like, kept short where a constant is 0.
const cdot = (A, c, v) => {
    const terms = [0, 1, 2].filter((i) => c[i] !== 0).map((i) => A.mul(c[i], v[i]));
    return terms.length === 0 ? A.num(0) : terms.length === 1 ? terms[0] : A.add(...terms);
};

// What the collisions need of a hull, worked out once: each edge's direction, length and the two vectors whose dot
// products with (x - its first corner) are both at least 0 where x is beyond it (within its normal cone); the edges
// whose faces are not flat to each other; and, for each corner and each polygon about it, the polygon's other corners
// MuJoCo adds against the floor (up to three, its largest quadrilateral from that corner).
const prepared = new WeakMap();
function prepare(hull) {
    if (prepared.has(hull)) return prepared.get(hull);
    const { points, normals } = hull;
    const edges = hull.edges.filter(({ left, right }) => 1 - numberDot(normals[left], normals[right]) > 1e-12).map(({ a, b, left, right }) => {
        const along = numberSub(points[b], points[a]);
        const u = numberUnit(along);
        const [nL, nR] = [normals[left], normals[right]];
        // x - p_a, square to u, is α nL + β nR with α, β ≥ 0: β's sign is its dot with nL × u (over nR's), α's with nR × u.
        const kL = numberCross(nL, u);
        const kR = numberCross(nR, u);
        const KL = kL.map((x) => x * Math.sign(numberDot(nR, kL)));
        const KR = kR.map((x) => x * Math.sign(numberDot(nL, kR)));
        return { a, b, u, KL, KR };
    });
    const area4 = (a, b, c, d) => Math.hypot(...numberCross(numberSub(points[a], points[c]), numberSub(points[b], points[d]))) / 2;
    // MuJoCo's hull4f: from the anchor, the quadrilateral of largest area along the polygon's corners.
    const quad = (polygon, anchor) => {
        const n = polygon.length;
        const at = polygon.indexOf(anchor);
        let [b, c, d] = [(at + 1) % n, (at + 2) % n, (at + 3) % n];
        if (n <= 4) return polygon.filter((_, i) => i !== at);
        let best = area4(polygon[at], polygon[b], polygon[c], polygon[d]);
        for (;;) {
            const dn = (d + 1) % n;
            let next = area4(polygon[at], polygon[b], polygon[c], polygon[dn]);
            if (next <= best) break;
            d = dn; best = next;
            for (;;) { const cn = (c + 1) % n; next = area4(polygon[at], polygon[b], polygon[cn], polygon[d]); if (next <= best) break; c = cn; best = next; }
            for (;;) { const bn = (b + 1) % n; next = area4(polygon[at], polygon[bn], polygon[c], polygon[d]); if (next <= best) break; b = bn; best = next; }
        }
        return [polygon[b], polygon[c], polygon[d]];
    };
    const floorSets = points.map((_, v) => hull.polygonsAround[v].map((p) => ({ polygon: p, others: quad(hull.polygons[p].vertices, v) })));
    // Each polygon's edges as planes square to it (m · x ≤ o inside), for clipping a segment to it.
    const polygonOf = hull.triangles.map(() => -1);
    hull.polygons.forEach(({ vertices, normal }, p) => {
        const set = new Set(vertices);
        hull.triangles.forEach((triangle, t) => { if (polygonOf[t] < 0 && triangle.every((v) => set.has(v)) && 1 - numberDot(normals[t], normal) < 1e-6) polygonOf[t] = p; });
    });
    const sides = hull.polygons.map(({ vertices, normal }) => vertices.map((v, i) => {
        const w = vertices[(i + 1) % vertices.length];
        const m = numberUnit(numberCross(numberSub(points[w], points[v]), normal));
        return { m, o: numberDot(m, points[v]) };
    }));
    const result = { edges, floorSets, polygonOf, sides };
    prepared.set(hull, result);
    return result;
}

// A local point or direction of the mesh, in the world.
const meshPoint = (A, mesh, p) => v3.add(A, mesh.pos, v3.mat(A, mesh.mat, p));
const meshDirection = (A, mesh, n) => v3.mat(A, mesh.mat, n);
const constVector = (A, c) => c.map((x) => A.num(x));

// mjc_PlaneConvex (MuJoCo 3.16): the hull's corner deepest below the plane, the first contact; then, of the polygons
// about it, the one turned most against the plane's normal, cut to its largest quadrilateral from that corner, and each
// of its other corners within the margin and below the mesh's centre (along the normal), in that order.
function planeMesh(A, plane, mesh, margin) {
    const { hull } = mesh;
    const { floorSets } = prepare(hull);
    const normal = column(plane.mat, 2);
    const local = v3.name(A, 'mpl', v3.matT(A, mesh.mat, normal));
    const height = A.name('mph', v3.dot(A, v3.sub(A, mesh.pos, plane.pos), normal));
    const heights = hull.points.map((p, v) => A.name(`mpd${v + 1}`, A.add(height, cdot(A, p, local))));
    // The deepest corner: the first of the lowest.
    let low = A.num(1e30);
    const lower = heights.map((h, v) => { const flag = A.name(`mpq${v + 1}`, A.lt(h, low)); low = A.name(`mpm${v + 1}`, A.pick(flag, h, low)); return flag; });
    let none = A.always;
    const deepest = [];
    for (let v = hull.points.length - 1; v >= 0; v--) {
        deepest[v] = A.name(`mpk${v + 1}`, A.and(lower[v], none));
        none = (v % 64 === 0) ? A.name(`mpn${v}`, A.and(none, A.not(lower[v]))) : A.and(none, A.not(lower[v]));
    }
    const dist = low;
    const corner = v3.name(A, 'mpc', [0, 1, 2].map((axis) => pickOne(A, deepest, hull.points.map((p) => p[axis]), `mpc${axis + 1}x`)));
    const first = { active: A.le(dist, margin), dist, pos: v3.name(A, 'mpp', v3.sub(A, meshPoint(A, mesh, corner), v3.scale(A, normal, A.mul(0.5, dist)))) };
    // Each polygon's turn against the normal; about each corner, the first most turned.
    const turns = hull.polygons.map(({ normal: pn }, p) => A.name(`mpt${p + 1}`, cdot(A, pn, local)));
    const pairs = [];
    floorSets.forEach((sets, v) => {
        let most = A.num(1);
        const flags = sets.map(({ polygon }, j) => {
            const flag = A.name(`mpf${v + 1}x${j + 1}`, A.lt(turns[polygon], most));
            most = A.name(`mpw${v + 1}x${j + 1}`, A.pick(flag, turns[polygon], most));
            return flag;
        });
        sets.forEach(({ others }, j) => pairs.push({ flag: A.name(`mps${v + 1}x${j + 1}`, A.and(deepest[v], flags[j], ...flags.slice(j + 1).map((later) => A.not(later)))), others }));
    });
    const centreHeight = cdot(A, mesh.centre, local);
    const extra = [0, 1, 2].map((k) => {
        const holders = pairs.filter(({ others }) => others.length > k);
        const has = holders.length ? A.name(`mpe${k + 1}h`, A.or(...holders.map(({ flag }) => flag))) : A.never;
        const p = v3.name(A, `mpe${k + 1}v`, [0, 1, 2].map((axis) => pickOne(A, holders.map(({ flag }) => flag), holders.map(({ others }) => hull.points[others[k]][axis]), `mpe${k + 1}v${axis + 1}x`)));
        const along = A.name(`mpe${k + 1}l`, v3.dot(A, p, local));
        const d = A.name(`mpe${k + 1}d`, A.add(height, along));
        return { active: A.name(`mpe${k + 1}a`, A.and(has, first.active, A.le(d, margin), A.le(A.sub(along, centreHeight), 0))), dist: d, pos: v3.sub(A, meshPoint(A, mesh, p), v3.scale(A, normal, A.mul(0.5, d))) };
    });
    // Packed after the first, in order.
    const before = [A.num(0)];
    extra.forEach(({ active }, k) => before.push(A.name(`mpb${k + 1}`, A.add(before[k], A.pick(active, 1, 0)))));
    const slots = [0, 1, 2].map((s) => {
        const takers = extra.map(({ active }, k) => A.name(`mpg${s + 1}x${k + 1}`, A.and(active, A.gt(before[k], s - 0.5), A.lt(before[k], s + 0.5))));
        return {
            active: A.or(...takers),
            dist: A.name(`mpg${s + 1}d`, pickOne(A, takers, extra.map(({ dist: d }) => d), `mpg${s + 1}dx`)),
            pos: [0, 1, 2].map((axis) => A.name(`mpg${s + 1}p${axis + 1}`, pickOne(A, takers, extra.map(({ pos }) => pos[axis]), `mpg${s + 1}p${axis + 1}x`))),
            frame: [...normal, ...noTangent(A)]
        };
    });
    return [{ ...first, frame: [...normal, ...noTangent(A)] }, ...slots];
}

// The hull's candidates against a point `c` (local, an expression) of a shape reaching `r` beyond it, each { value
// (the gap), valid, n (local, from the hull toward c), pos (local, midway) }: faces, edges and corners. `extra(n)`, a
// further condition on a direction (a capsule's end reaching furthest along -n), `tag`, and whether to give faces.
function pointCandidates(A, hull, edges, c, r, extra, tag, faces = true) {
    const candidates = [];
    if (faces) hull.normals.forEach((n, t) => {
        const g = A.name(`${tag}f${t + 1}`, A.sub(cdot(A, n, c), hull.offsets[t]));
        const nv = constVector(A, n);
        candidates.push({ value: A.sub(g, r), valid: extra(nv), n: nv, pos: v3.sub(A, c, v3.scale(A, nv, A.mul(0.5, A.add(g, r)))) });
    });
    edges.forEach(({ a, u, KL, KR }, e) => {
        const w = v3.name(A, `${tag}e${e + 1}w`, v3.sub(A, c, constVector(A, hull.points[a])));
        const t = A.name(`${tag}e${e + 1}t`, cdot(A, u, w));
        const across = v3.sub(A, w, v3.scale(A, constVector(A, u), t));
        const length = A.name(`${tag}e${e + 1}l`, A.sqrt(A.max(A.sub(v3.dot(A, w, w), A.mul(t, t)), 0)));
        const safe = A.max(length, 1e-300);
        const n = v3.name(A, `${tag}e${e + 1}n`, across.map((x) => A.div(x, safe)));
        const cone = A.and(A.ge(cdot(A, KL, w), 0), A.ge(cdot(A, KR, w), 0), A.gt(length, 1e-15));
        candidates.push({ value: A.sub(length, r), valid: A.and(cone, extra(n)), n, pos: v3.add(A, v3.add(A, constVector(A, hull.points[a]), v3.scale(A, constVector(A, u), t)), v3.scale(A, n, A.mul(0.5, A.sub(length, r)))) });
    });
    hull.points.forEach((p, v) => {
        const w = v3.name(A, `${tag}v${v + 1}w`, v3.sub(A, c, constVector(A, p)));
        const length = A.name(`${tag}v${v + 1}l`, A.sqrt(v3.dot(A, w, w)));
        const safe = A.max(length, 1e-300);
        const n = v3.name(A, `${tag}v${v + 1}n`, w.map((x) => A.div(x, safe)));
        const cone = A.and(A.gt(length, 1e-15), ...hull.neighbours[v].map((q) => A.le(cdot(A, numberSub(hull.points[q], p), w), 0)));
        candidates.push({ value: A.sub(length, r), valid: A.and(cone, extra(n)), n, pos: v3.add(A, constVector(A, p), v3.scale(A, n, A.mul(0.5, A.sub(length, r)))) });
    });
    return candidates;
}

// A sphere against a mesh: one contact, exactly (MuJoCo's convex search, to its tolerance).
function sphereMesh(A, sphere, mesh, margin) {
    const { hull } = mesh;
    const c = v3.name(A, 'msc', v3.matT(A, mesh.mat, v3.sub(A, sphere.pos, mesh.pos)));
    const best = bestOf(A, pointCandidates(A, hull, prepare(hull).edges, c, sphere.size[0], () => A.always, 'ms'), 'msb');
    const dist = A.name('msdist', best.gap);
    return [{ active: A.le(dist, margin), dist, pos: meshPoint(A, mesh, best.pos), frame: [...v3.scale(A, meshDirection(A, mesh, best.n), -1), ...noTangent(A)] }];
}

// A capsule against a mesh: the deepest contact, exactly; and where it lies along a face of the hull (its axis within
// 2.6 degrees of the face's plane), its segment cut to the face's polygon, a contact at each end of what remains, as
// MuJoCo gives two contacts there (where it gives two).
function capsuleMesh(A, capsule, mesh, margin) {
    const { hull } = mesh;
    const { edges, polygonOf, sides } = prepare(hull);
    const r = capsule.size[0];
    const L = capsule.size[1];
    const c = v3.name(A, 'mcc', v3.matT(A, mesh.mat, v3.sub(A, capsule.pos, mesh.pos)));
    const a = v3.name(A, 'mca', v3.matT(A, mesh.mat, column(capsule.mat, 2)));
    const ends = [-1, 1].map((sign, k) => v3.name(A, `mce${k}`, v3.add(A, c, v3.scale(A, a, sign * L))));
    const candidates = [];
    // Faces: the end reaching further along -n.
    hull.normals.forEach((n, t) => {
        const heights = ends.map((end) => cdot(A, n, end));
        const nearer = A.name(`mcf${t + 1}k`, A.le(heights[0], heights[1]));
        const g = A.name(`mcf${t + 1}`, A.sub(A.min(heights[0], heights[1]), hull.offsets[t]));
        const nv = constVector(A, n);
        const end = v3.pick(A, nearer, ends[0], ends[1]);
        candidates.push({ value: A.sub(g, r), valid: A.always, n: nv, pos: v3.sub(A, end, v3.scale(A, nv, A.mul(0.5, A.add(g, r)))) });
    });
    const faceCount = candidates.length;
    // Each end against the edges and corners, where it is the end reaching further along -n.
    ends.forEach((end, k) => {
        const toward = v3.sub(A, ends[1 - k], end);
        candidates.push(...pointCandidates(A, hull, edges, end, r, (n) => A.ge(v3.dot(A, n, toward), 0), `mc${k}`, false));
    });
    // The segment square to n against a corner: n from the corner to the axis's line.
    hull.points.forEach((p, v) => {
        const w0 = v3.sub(A, c, constVector(A, p));
        const t = v3.dot(A, w0, a);
        const w = v3.name(A, `mcs${v + 1}w`, v3.sub(A, w0, v3.scale(A, a, t)));
        const length = A.name(`mcs${v + 1}l`, A.sqrt(v3.dot(A, w, w)));
        const safe = A.max(length, 1e-300);
        const n = v3.name(A, `mcs${v + 1}n`, w.map((x) => A.div(x, safe)));
        const cone = A.and(A.gt(length, 1e-15), ...hull.neighbours[v].map((q) => A.le(cdot(A, numberSub(hull.points[q], p), w), 0)));
        const within = A.le(A.abs(t), L);
        candidates.push({ value: A.sub(length, r), valid: A.and(cone, within), n, pos: v3.add(A, constVector(A, p), v3.scale(A, n, A.mul(0.5, A.sub(length, r)))) });
    });
    // The segment against an edge: n square to both, the way that lies within the edge's normal cone.
    edges.forEach(({ a: start, u, KL, KR }, e) => {
        const m = v3.name(A, `mcx${e + 1}m`, v3.cross(A, a, constVector(A, u)));
        const size = A.name(`mcx${e + 1}s`, A.sqrt(v3.dot(A, m, m)));
        const plus = A.and(A.ge(cdot(A, KL, m), 0), A.ge(cdot(A, KR, m), 0));
        const minus = A.and(A.le(cdot(A, KL, m), 0), A.le(cdot(A, KR, m), 0));
        const sign = A.pick(plus, A.num(1), A.num(-1));
        const safe = A.max(size, 1e-300);
        const n = v3.name(A, `mcx${e + 1}n`, m.map((x) => A.div(A.mul(sign, x), safe)));
        const d0 = v3.sub(A, c, constVector(A, hull.points[start]));
        const gap = A.name(`mcx${e + 1}g`, v3.dot(A, n, d0));
        // The lines' nearest points: c + s a on the capsule's, p + t u on the edge's.
        const b = cdot(A, u, a);
        const du = cdot(A, u, d0);
        const da = v3.dot(A, d0, a);
        const den = A.max(A.sub(1, A.mul(b, b)), 1e-300);
        const s = A.div(A.sub(A.mul(b, du), da), den);
        const t = A.div(A.sub(du, A.mul(b, da)), den);
        const onEdge = v3.add(A, constVector(A, hull.points[start]), v3.scale(A, constVector(A, u), t));
        const onCapsule = v3.sub(A, v3.add(A, c, v3.scale(A, a, s)), v3.scale(A, n, r));
        candidates.push({ value: A.sub(gap, r), valid: A.and(A.or(plus, minus), A.gt(size, 1e-9), A.le(A.abs(s), L)), n, pos: v3.scale(A, v3.add(A, onEdge, onCapsule), 0.5) });
    });
    const best = bestOf(A, candidates, 'mcb');
    const dist = A.name('mcdist', best.gap);
    const normal = v3.name(A, 'mcn', v3.scale(A, meshDirection(A, mesh, best.n), -1));
    const deepest = { active: A.le(dist, margin), dist, pos: meshPoint(A, mesh, best.pos), frame: [...normal, ...noTangent(A)] };
    // Lying along a face: the face's polygon, and the segment (along the capsule's surface toward it) cut to it.
    const isFace = A.name('mcif', A.or(...best.chosen.slice(0, faceCount)));
    const flat = A.name('mcfl', A.and(isFace, A.lt(A.abs(v3.dot(A, best.n, a)), 0.045)));
    const polygonFlags = hull.polygons.map((_, p) => {
        const members = best.chosen.slice(0, faceCount).filter((_, t) => polygonOf[t] === p);
        return members.length ? A.name(`mcg${p + 1}`, A.or(...members)) : A.never;
    });
    const start = v3.name(A, 'mcl0', v3.sub(A, ends[0], v3.scale(A, best.n, r)));
    const run = v3.name(A, 'mcl1', v3.sub(A, ends[1], ends[0]));
    // Each polygon's interval of the run inside it, [low, high] within [0, 1].
    const intervals = sides.map((list, p) => {
        let low = A.num(0);
        let high = A.num(1);
        list.forEach(({ m, o }) => {
            const at = A.sub(cdot(A, m, start), o);
            const rate = cdot(A, m, run);
            const t = A.div(A.neg(at), A.pick(A.gt(A.abs(rate), 1e-300), rate, A.num(1)));
            low = A.pick(A.lt(rate, 0), A.max(low, t), low);
            high = A.pick(A.gt(rate, 0), A.min(high, t), high);
            // Parallel to the side and outside it: nothing inside.
            high = A.pick(A.and(A.le(A.abs(rate), 1e-300), A.gt(at, 0)), A.num(-1), high);
        });
        return [A.name(`mci${p + 1}a`, low), A.name(`mci${p + 1}b`, high)];
    });
    const low = A.name('mcia', pickOne(A, polygonFlags, intervals.map(([l]) => l), 'mciax'));
    const high = A.name('mcib', pickOne(A, polygonFlags, intervals.map(([, h]) => h), 'mcibx'));
    const offset = A.name('mco', pickOne(A, best.chosen.slice(0, faceCount), hull.offsets, 'mcox'));
    const along = (t, k) => {
        const x = v3.name(A, `mcz${k}x`, v3.add(A, start, v3.scale(A, run, t)));
        const d = A.name(`mcz${k}d`, A.sub(v3.dot(A, best.n, x), offset));
        return { dist: d, pos: meshPoint(A, mesh, v3.sub(A, x, v3.scale(A, best.n, A.mul(0.5, d)))) };
    };
    const lying = A.name('mcly', A.and(flat, A.le(low, high)));
    const [p0, p1] = [along(low, 0), along(high, 1)];
    return [
        { active: A.pick(lying, A.le(p0.dist, margin), deepest.active), dist: A.pick(lying, p0.dist, dist), pos: v3.pick(A, lying, p0.pos, deepest.pos), frame: deepest.frame },
        { active: A.and(lying, A.le(p1.dist, margin), A.gt(A.sub(high, low), 1e-12)), dist: p1.dist, pos: p1.pos, frame: deepest.frame }
    ];
}
