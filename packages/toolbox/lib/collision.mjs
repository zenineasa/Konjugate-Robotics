/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Where two shapes touch, as MuJoCo 3.16 finds it (its engine_collision_primitive.c and engine_collision_box.c, and
// mju_makeFrame), for each pair of shapes the toolbox collides: a plane (the floor) with a sphere, a capsule, a
// cylinder or a box; a sphere with a sphere, a capsule or a box; a capsule with a capsule or a box. Each function gives a fixed
// number of contact slots, as many as the pair can have at once, each { active, dist, pos, frame }: whether MuJoCo
// would have that contact now (in the order its function makes them), how far apart the shapes are there (negative
// when they overlap), the point (midway through the overlap) and the frame (the normal, from the first shape to the
// second, then two tangents).
//
// Written once over an algebra, so the model builder writes them as equations (lib/modelBuilder.mjs's contact
// relationships) and the tests work them out in numbers, both from the same steps. An algebra has: num, add, sub, mul,
// div, neg, sqrt, abs, min, max; comparisons gt, lt, ge, le giving conditions, combined by and, or, not; pick(condition,
// a, b); and name(tag, value), which an algebra of equations makes a named definition. Where MuJoCo divides only in
// the branch it takes, the other branch here divides by something safe, so equations evaluated whole stay finite.
//
// A shape: { kind: 'plane' | 'sphere' | 'capsule' | 'cylinder' | 'box', pos [3], mat [9] (rows, as MuJoCo's xmat),
// size [3] (MuJoCo's: a sphere's radius; a capsule's or a cylinder's radius and half length; a box's half sizes) }.
// Pure, no dependencies.

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
export const geomOrder = ['plane', 'sphere', 'capsule', 'cylinder', 'box'];
export const contactSlots = Object.freeze({
    'plane|sphere': 1, 'plane|capsule': 2, 'plane|cylinder': 4, 'plane|box': 4,
    'sphere|sphere': 1, 'sphere|capsule': 1, 'sphere|box': 1, 'capsule|capsule': 2, 'capsule|box': 2
});
export const pairKey = (first, second) => `${first}|${second}`;
export const collides = (first, second) => pairKey(...[first, second].sort((a, b) => geomOrder.indexOf(a) - geomOrder.indexOf(b))) in contactSlots;

// The contacts between `first` and `second` (already in MuJoCo's order: the first's kind no later than the second's),
// within `margin`.
export function collide(A, first, second, margin) {
    const key = pairKey(first.kind, second.kind);
    const found = {
        'plane|sphere': planeSphere, 'plane|capsule': planeCapsule, 'plane|cylinder': planeCylinder, 'plane|box': planeBox,
        'sphere|sphere': sphereSphere, 'sphere|capsule': sphereCapsule, 'sphere|box': sphereBox, 'capsule|capsule': capsuleCapsule,
        'capsule|box': capsuleBox
    }[key];
    if (!found) throw new Error(`MuJoCo's ${first.kind} and ${second.kind} collision is not built yet.`);
    return found(A, first, second, A.num(margin)).map((slot, index) => ({ ...slot, frame: makeFrame(A, slot.frame, `f${index + 1}`) }));
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
