/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Contact in the drawing: which shapes of each solid touch (its colliders), how its surface holds (friction and
// softness), whether the floor is there, and which pairs of solids touch. Build makes each pair of colliders that may
// touch a relationship of its own (lib/modelBuilder.mjs), where MuJoCo's collision function for the pair
// (lib/collision.mjs) finds its contact points as equations, every instant.
//
// drawing.contact: { floor (the Ground's plane, z up through its origin), automatic (every pair of solids with
// colliders touches, less those a joint joins and those MuJoCo leaves out: two solids welded together, or one welded
// to the other's parent), pairs ([a, b] that touch whatever else, a solid id or 'ground' for the floor), exclude ([a,
// b] that never touch), floorSurface }. None of it when drawing.contact is missing: a drawing kept before contact came
// in builds as it did.
// A solid's colliders: its own shape, unless `touches` is false; a mesh solid has none (its colliders come with an
// imported model's collision shapes). Its `surface`: { friction (sliding), solref, solimp, margin }, MuJoCo's defaults
// where missing. Pure.

import { collides, contactSlots, geomOrder, pairKey } from './collision.mjs';
import { groundId, solidById, structure, treeOrder } from './drawing.mjs';
import { mjcfGeometry } from './shapes.mjs';

export const defaultSurface = Object.freeze({ friction: 1, solref: Object.freeze([0.02, 1]), solimp: Object.freeze([0.9, 0.95, 0.001, 0.5, 2]), margin: 0 });
// MuJoCo's least friction (mjMINMU): a contact's friction is never below it.
export const leastFriction = 1e-5;

export const contactOf = (drawing) => drawing.contact ?? null;
export const defaultContact = () => ({ floor: true, automatic: true, pairs: [], exclude: [], floorSurface: {} });
export const surfaceOf = (owner) => ({ ...defaultSurface, ...(owner?.surface ?? {}) });
export const floorSurfaceOf = (drawing) => ({ ...defaultSurface, ...(contactOf(drawing)?.floorSurface ?? {}) });

// A solid's colliders, each { kind (MuJoCo's), size (MuJoCo's), position and rotation in the solid's frame }.
export function collidersOf(solid) {
    if (solid.touches === false) return [];
    if (solid.colliders) {
        return solid.colliders.map(({ shape, size, position = [0, 0, 0], rotation = [1, 0, 0, 0] }) => ({ kind: shape, size: mjcfGeometry(shape, size).size, position, rotation }));
    }
    if (solid.shape === 'mesh') return [];
    return [{ kind: solid.shape, size: mjcfGeometry(solid.shape, solid.size).size, position: [0, 0, 0], rotation: [1, 0, 0, 0] }];
}

// Changes the drawing's contact settings (`changes` merged in; `null` turns contact off). Returns a new drawing.
export function updateContact(drawing, changes) {
    const next = structuredClone(drawing);
    if (changes === null) { delete next.contact; return next; }
    const current = next.contact ?? defaultContact();
    next.contact = { ...current, ...changes, floorSurface: { ...(current.floorSurface ?? {}), ...(changes.floorSurface ?? {}) } };
    return next;
}

const samePair = ([a, b], [c, d]) => (a === c && b === d) || (a === d && b === c);

// Says whether solids `a` and `b` (or a solid and 'ground') touch: true always, false never, null as the automatic
// rule has it. Turns contact on if it was off. Returns a new drawing.
export function setTouching(drawing, a, b, touching) {
    const next = updateContact(drawing, {});
    const contact = next.contact;
    contact.pairs = contact.pairs.filter((pair) => !samePair(pair, [a, b]));
    contact.exclude = contact.exclude.filter((pair) => !samePair(pair, [a, b]));
    if (touching === true) contact.pairs.push([a, b]);
    if (touching === false) contact.exclude.push([a, b]);
    return next;
}

// With solids gone: pairs naming them go.
export function withoutGoneSolids(drawing) {
    if (!drawing.contact) return drawing;
    const ids = new Set([groundId, ...drawing.solids.map(({ id }) => id)]);
    const kept = (pairs) => (pairs ?? []).filter((pair) => pair.every((id) => ids.has(id)));
    drawing.contact = { ...drawing.contact, pairs: kept(drawing.contact.pairs), exclude: kept(drawing.contact.exclude) };
    return drawing;
}

// Whether two solids touch: named, excluded, or by the automatic rule, MuJoCo's own: not when a joint joins them,
// not when they are welded together (by fixed joints) or one is welded to the other's parent, and not when neither
// can move.
function touchingRule(drawing, built) {
    const contact = contactOf(drawing);
    const solidOf = new Map(drawing.frames.map((frame) => [frame.id, frame.solid]));
    const joined = drawing.joints.map((joint) => [solidOf.get(joint.parent), solidOf.get(joint.child)]);
    const weldOf = (id) => {
        let at = id;
        while (at !== groundId && built.parentJoint.get(at)?.joint.kind === 'fixed') at = built.parentJoint.get(at).parent;
        return at;
    };
    const weldParentOf = (weld) => (weld === groundId ? groundId : weldOf(built.parentJoint.get(weld)?.parent ?? groundId));
    return (a, b) => {
        if ((contact.exclude ?? []).some((pair) => samePair(pair, [a, b]))) return false;
        if ((contact.pairs ?? []).some((pair) => samePair(pair, [a, b]))) return true;
        if (b === groundId || a === groundId ? !contact.floor : !contact.automatic) return false;
        if (joined.some((pair) => samePair(pair, [a, b]))) return false;
        const [wa, wb] = [weldOf(a), weldOf(b)];
        if (wa === wb) return false;
        if (wa === groundId && wb === groundId) return false;
        if (wa !== groundId && wb !== groundId && (wa === weldParentOf(wb) || wb === weldParentOf(wa))) return false;
        return true;
    };
}

// Every pair of colliders that may touch, in MuJoCo's order (the first's kind no later than the second's; the floor a
// plane of the Ground's): { first: { solid, index, collider }, second, slots, name }; and `unsupported`, the pairs
// that touch but whose collision is not built yet, each { first, second, kinds }.
export function contactPairs(drawing, built = structure(drawing)) {
    const contact = contactOf(drawing);
    if (!contact) return { pairs: [], unsupported: [] };
    const touching = touchingRule(drawing, built);
    const order = treeOrder(drawing, built).map(({ id }) => id);
    const entries = order.flatMap((id) => collidersOf(solidById(drawing, id)).map((collider, index) => ({ solid: id, index, collider })));
    const floor = { solid: groundId, index: 0, collider: { kind: 'plane', size: [0, 0, 1], position: [0, 0, 0], rotation: [1, 0, 0, 0] } };
    const candidates = [];
    if (contact.floor || (contact.pairs ?? []).some((pair) => pair.includes(groundId))) for (const entry of entries) candidates.push([floor, entry]);
    for (let i = 0; i < entries.length; i += 1) for (let j = i + 1; j < entries.length; j += 1) if (entries[i].solid !== entries[j].solid) candidates.push([entries[i], entries[j]]);
    const pairs = [];
    const unsupported = [];
    const nameOf = (entry) => (entry.solid === groundId ? 'Floor' : solidById(drawing, entry.solid).name);
    for (const [a, b] of candidates) {
        if (!touching(a.solid, b.solid)) continue;
        const [first, second] = geomOrder.indexOf(a.collider.kind) <= geomOrder.indexOf(b.collider.kind) ? [a, b] : [b, a];
        if (!collides(first.collider.kind, second.collider.kind)) { unsupported.push({ first, second, kinds: [first.collider.kind, second.collider.kind] }); continue; }
        const many = (entry) => (entry.solid !== groundId && collidersOf(solidById(drawing, entry.solid)).length > 1 ? ` ${entry.index + 1}` : '');
        pairs.push({ first, second, slots: contactSlots[pairKey(first.collider.kind, second.collider.kind)], name: `${nameOf(first)}${many(first)} and ${nameOf(second)}${many(second)}` });
    }
    return { pairs, unsupported };
}

// The checks for contact: settings that cannot be, and pairs that touch but cannot yet.
export function contactChecks(drawing) {
    const errors = [];
    const warnings = [];
    if (!contactOf(drawing)) return { errors, warnings };
    const owners = [...drawing.solids.filter(({ surface }) => surface).map((solid) => [solid.id, solid.name, solid.surface]), [null, 'The floor', contactOf(drawing).floorSurface ?? {}]];
    for (const [id, name, surface] of owners) {
        if (surface.friction !== undefined && !(Number.isFinite(surface.friction) && surface.friction >= 0)) errors.push({ id, message: `${name}: its friction must be zero or more.` });
        if (surface.margin !== undefined && !(Number.isFinite(surface.margin) && surface.margin >= 0)) errors.push({ id, message: `${name}: its contact margin must be zero or more.` });
        if (surface.solref && !(surface.solref.length === 2 && surface.solref.every(Number.isFinite))) errors.push({ id, message: `${name}: its contact's solref is two numbers.` });
        if (surface.solimp && !(surface.solimp.length === 5 && surface.solimp.every(Number.isFinite))) errors.push({ id, message: `${name}: its contact's solimp is five numbers.` });
    }
    for (const { first, second, kinds } of contactPairs(drawing).unsupported) {
        const name = (entry) => (entry.solid === groundId ? 'the floor' : solidById(drawing, entry.solid).name);
        warnings.push({ id: second.solid, message: `${name(first)} and ${name(second)}: a ${kinds[0]} against a ${kinds[1]} does not touch yet; they pass through each other.` });
    }
    return { errors, warnings };
}
