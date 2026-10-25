/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Reads a MuJoCo model (MJCF) for an import into the drawing: its bodies (each one's place in its parent, its joints,
// its mass and inertia, and how it looks), its actuators, and what it has that the drawing cannot hold, counted so the
// import can say so. MuJoCo compiles the model when it runs; this reads the same files in JavaScript, resolving what
// decides those answers: included files, the compiler's angle unit and Euler sequence, default classes (nested, and a
// body's childclass for what is inside it), <frame> elements, the actuator shortcuts, autolimits, and a body's
// inertia worked out from its geoms when the model gives none. A model it cannot read is refused with the reason, not
// guessed at.
//
// No dependencies but frames.mjs, meshes.mjs and inertia.mjs: a small XML reader of its own, for MJCF's plain
// elements and attributes.

import { compose, frameOf, fromAxisAngle, identity, normalise as unitQuaternion, rotate } from './frames.mjs';
import { combine, diagonalTensor, fullTensor, meshInertia, placed, primitiveInertia, principal } from './inertia.mjs';
import { MeshBuilder, primitiveTriangles, readObj, readStl } from './meshes.mjs';

// ---- XML -------------------------------------------------------------------------------------------------------

const entities = { lt: '<', gt: '>', amp: '&', quot: '"', apos: "'" };
const decode = (text) => text.replace(/&(lt|gt|amp|quot|apos|#\d+|#x[0-9a-f]+);/gi, (match, name) => (name[0] === '#'
    ? String.fromCodePoint(name[1].toLowerCase() === 'x' ? Number.parseInt(name.slice(2), 16) : Number(name.slice(1)))
    : entities[name.toLowerCase()]));

// Elements as { name, attributes, children }; comments, declarations and text left out.
export function parseXml(text, fileName = 'the model') {
    const source = String(text).replace(/<!--[\s\S]*?-->/g, '').replace(/<\?[\s\S]*?\?>/g, '').replace(/<!DOCTYPE[\s\S]*?>/gi, '');
    if (/<!\[CDATA\[/.test(source)) throw new Error(`${fileName} has a CDATA section, which this reader does not take.`);
    const root = { name: '#document', attributes: {}, children: [] };
    const stack = [root];
    const tag = /<(\/?)([A-Za-z_][\w:.-]*)((?:\s+[\w:.-]+\s*=\s*(?:"[^"]*"|'[^']*'))*)\s*(\/?)>/g;
    const attribute = /([\w:.-]+)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
    let end = 0;
    for (const match of source.matchAll(tag)) {
        if (/</.test(source.slice(end, match.index))) throw new Error(`${fileName} has markup this reader cannot follow near character ${end}.`);
        end = match.index + match[0].length;
        const [, closing, name, attributeText, selfClosing] = match;
        if (closing) {
            const open = stack.pop();
            if (open?.name !== name) throw new Error(`${fileName} closes <${name}> where <${open?.name}> is open.`);
            continue;
        }
        const element = { name, attributes: {}, children: [] };
        for (const [, key, double, single] of attributeText.matchAll(attribute)) element.attributes[key] = decode(double ?? single);
        stack.at(-1).children.push(element);
        if (!selfClosing) stack.push(element);
    }
    if (stack.length !== 1) throw new Error(`${fileName} ends with <${stack.at(-1).name}> still open.`);
    if (root.children.length !== 1) throw new Error(`${fileName} should have one top element.`);
    return root.children[0];
}

// ---- MJCF -------------------------------------------------------------------------------------------------------

const numbers = (text) => String(text ?? '').trim().split(/\s+/).filter(Boolean).map(Number);

// The directory part of a path in the model's folder ("" at its top).
const directoryOf = (path) => (path.includes('/') ? path.slice(0, path.lastIndexOf('/') + 1) : '');
const normalisePath = (path) => {
    const parts = [];
    for (const part of path.split('/')) {
        if (part === '..') parts.pop();
        else if (part && part !== '.') parts.push(part);
    }
    return parts.join('/');
};

// The model's top element with every <include> replaced by the elements of the file it names (paths relative to the
// main file, as MuJoCo takes them). `files` maps paths in the model's folder to their text or bytes.
function resolveIncludes(main, files) {
    const text = (path) => {
        const content = files[path];
        if (content === undefined) throw new Error(`The model has no file ${path}.`);
        return typeof content === 'string' ? content : new TextDecoder().decode(content);
    };
    const base = directoryOf(main);
    const seen = new Set([main]);
    const expand = (element) => {
        element.children = element.children.flatMap((child) => {
            if (child.name !== 'include') return [expand(child)];
            const path = normalisePath(base + child.attributes.file);
            if (seen.has(path)) throw new Error(`The model includes ${path} more than once.`);
            seen.add(path);
            const included = parseXml(text(path), path);
            if (included.name !== 'mujoco') throw new Error(`${path} should have <mujoco> as its top element.`);
            return expand(included).children;
        });
        return element;
    };
    const root = parseXml(text(main), main);
    if (root.name !== 'mujoco') throw new Error(`${main} should have <mujoco> as its top element.`);
    return expand(root);
}

// Default classes by name, each { parent, elements: { joint: {...}, position: {...}, ... } }; the top one is "main".
function defaultClasses(root) {
    const classes = new Map();
    const visit = (element, parent) => {
        const name = element.attributes.class ?? (parent === null ? 'main' : null);
        if (name === null) throw new Error('A nested <default> needs a class name.');
        if (classes.has(name)) throw new Error(`The model defines default class ${name} twice.`);
        const entry = { parent, elements: {} };
        classes.set(name, entry);
        for (const child of element.children) {
            if (child.name === 'default') visit(child, name);
            else entry.elements[child.name] = { ...(entry.elements[child.name] ?? {}), ...child.attributes };
        }
    };
    const tops = root.children.filter((child) => child.name === 'default');
    if (tops.length > 1) {
        // Several top-level <default>s are one "main" class between them.
        const merged = { name: 'default', attributes: {}, children: tops.flatMap((top) => (top.attributes.class ? [top] : top.children)) };
        visit(merged, null);
    } else if (tops.length) visit(tops[0], null);
    if (!classes.has('main')) classes.set('main', { parent: null, elements: {} });
    return classes;
}

// An element's attribute as MuJoCo resolves it: its own, else its class's (or the class it inherits), up to "main".
function resolver(classes) {
    return (element, kind, className, attribute) => {
        if (element.attributes[attribute] !== undefined) return element.attributes[attribute];
        for (let name = className; name !== null; name = classes.get(name).parent) {
            const entry = classes.get(name);
            if (!entry) throw new Error(`The model uses default class ${name}, which it does not define.`);
            const value = entry.elements[kind]?.[attribute];
            if (value !== undefined) return value;
        }
        return undefined;
    };
}

const actuatorKinds = new Set(['general', 'motor', 'position', 'velocity', 'intvelocity', 'damper', 'cylinder', 'muscle', 'adhesion', 'plugin']);
const hex = (rgba) => `#${rgba.slice(0, 3).map((value) => Math.round(Math.min(Math.max(value, 0), 1) * 255).toString(16).padStart(2, '0')).join('')}`;

// The files a model reads, found from its MJCF alone: `fetchText(path)` answers a file's text by its path in the model's
// folder. Starting from `main`, every included file (relative to the main file, as MuJoCo takes them) and every mesh
// it names (in the compiler's meshdir, or assetdir), as paths in the folder; the MJCF files first. Textures and height
// fields are left out: an import does not read them.
export async function modelPaths(fetchText, main) {
    const base = directoryOf(main);
    const xml = [main];
    const meshes = [];
    let compiler = {};
    for (let at = 0; at < xml.length; at += 1) {
        const root = parseXml(await fetchText(xml[at]), xml[at]);
        const visit = (element) => {
            for (const child of element.children) {
                if (child.name === 'include' && child.attributes.file) {
                    const path = normalisePath(base + child.attributes.file);
                    if (!xml.includes(path)) xml.push(path);
                } else if (child.name === 'compiler') compiler = { ...compiler, ...child.attributes };
                else if (child.name === 'mesh' && child.attributes.file) meshes.push(child.attributes.file);
                visit(child);
            }
        };
        visit(root);
    }
    const meshDirectory = normalisePath(base + (compiler.meshdir ?? compiler.assetdir ?? ''));
    const meshPaths = [...new Set(meshes.map((file) => normalisePath(`${meshDirectory ? `${meshDirectory}/` : ''}${file}`)))];
    return [...xml, ...meshPaths];
}

// What an import needs of a model: { name, bodies, actuators, gravity, timestep, leftOut, meshTriangles }. `files` maps paths in the
// model's folder to their text or bytes; `main` is the model's MJCF among them (its scene file, say, which includes the
// robot). Each body, parent before child:
// - name, parent (an index into bodies, -1 for the world), local (its place in its parent, { position, quaternion },
//   <frame>s included) and world (its place in the world with every joint at its ref);
// - joints: each { name, kind (hinge, slide, ball or free), anchor and axis in the body's frame, range (radians or
//   metres; a ball's most turn), damping, friction, armature, ref, stiffness, springref };
// - inertia: { mass, centre, tensor } in the body's frame, from its <inertial> or (with none, or inertiafromgeom true)
//   its geoms; `inertiaFrom` says which ('inertial', 'geoms', 'meshes' when a mesh is among them, or 'none');
// - geoms: the visible ones (groups 0 to 2, not fully transparent), { type, size, frame, mesh, color }, for its look.
// `leftOut` lists, in words, what the model has that an import leaves out or takes approximately.
export function readModel(files, main) {
    const root = resolveIncludes(main, files);
    for (const child of root.children) {
        if (['replicate', 'attach', 'composite', 'flexcomp'].includes(child.name)) throw new Error(`The model uses <${child.name}>, which this reader does not take yet.`);
    }
    const classes = defaultClasses(root);
    const resolve = resolver(classes);
    const compiler = root.children.filter((child) => child.name === 'compiler').reduce((all, child) => ({ ...all, ...child.attributes }), {});
    const degrees = (compiler.angle ?? 'degree') === 'degree';
    const angle = (value) => (degrees ? (value * Math.PI) / 180 : value);
    const placement = { angle, sequence: compiler.eulerseq ?? 'xyz' };
    const autolimits = (compiler.autolimits ?? 'true') === 'true';
    const inertiaFromGeom = compiler.inertiafromgeom ?? 'auto';
    const [groupLow, groupHigh] = numbers(compiler.inertiagrouprange ?? '0 5');
    const meshDirectory = normalisePath(directoryOf(main) + (compiler.meshdir ?? compiler.assetdir ?? ''));
    const leftOut = [];
    const option = root.children.filter((child) => child.name === 'option').reduce((all, child) => ({ ...all, ...child.attributes }), {});
    const gravity = numbers(option.gravity ?? '0 0 -9.81');
    const timestep = Number(option.timestep ?? 0.002);
    if (compiler.settotalmass !== undefined && Number(compiler.settotalmass) > 0) leftOut.push(`the compiler's settotalmass (${compiler.settotalmass} kg): the masses are taken as the model gives them`);

    // The model's meshes and materials, by name.
    const meshes = new Map();
    const materials = new Map();
    for (const asset of root.children.filter((child) => child.name === 'asset')) {
        for (const element of asset.children) {
            const className = element.attributes.class ?? 'main';
            if (element.name === 'mesh') {
                const file = resolve(element, 'mesh', className, 'file');
                if (!file) { leftOut.push(`mesh ${element.attributes.name ?? ''}, given by its vertices rather than a file`); continue; }
                const name = element.attributes.name ?? file.split('/').pop().replace(/\.[^.]+$/, '');
                meshes.set(name, {
                    file: normalisePath(`${meshDirectory ? `${meshDirectory}/` : ''}${file}`),
                    scale: numbers(resolve(element, 'mesh', className, 'scale') ?? '1 1 1'),
                    refpos: numbers(resolve(element, 'mesh', className, 'refpos') ?? '0 0 0'),
                    refquat: unitQuaternion(numbers(resolve(element, 'mesh', className, 'refquat') ?? '1 0 0 0'))
                });
            } else if (element.name === 'material') {
                const rgba = resolve(element, 'material', className, 'rgba');
                if (element.attributes.name && rgba) materials.set(element.attributes.name, numbers(rgba));
            }
        }
    }
    const triangleCache = new Map();
    // A mesh's triangles where its geom puts them: its file's coordinates scaled, moved by its refpos and turned by its
    // refquat, in the geom's frame. (MuJoCo recentres a mesh on its own centre of mass inside, and moves its geom to
    // match, so the mesh shows where its file puts it.)
    const meshTriangles = (name) => {
        if (triangleCache.has(name)) return triangleCache.get(name);
        const mesh = meshes.get(name);
        if (!mesh) throw new Error(`A geom names mesh ${name}, which the model does not define.`);
        const bytes = files[mesh.file];
        if (bytes === undefined) throw new Error(`The model has no file ${mesh.file}.`);
        const data = typeof bytes === 'string' ? new TextEncoder().encode(bytes) : bytes;
        const extension = mesh.file.split('.').pop().toLowerCase();
        const read = extension === 'stl' ? readStl(data, mesh.file) : extension === 'obj' ? readObj(data, mesh.file) : null;
        if (!read) throw new Error(`${mesh.file} is a .${extension} mesh, which this reader does not take yet.`);
        const inverse = [mesh.refquat[0], -mesh.refquat[1], -mesh.refquat[2], -mesh.refquat[3]];
        const placedTriangles = read.map((triangle) => triangle.map((corner) => rotate(inverse, corner.map((value, axis) => value * mesh.scale[axis] - mesh.refpos[axis]))));
        triangleCache.set(name, placedTriangles);
        return placedTriangles;
    };

    const bodies = [];
    const counts = { worldGeoms: 0, springs: [], gravcomp: [] };
    // A geom's frame and sizes, fromto read as MuJoCo reads it.
    const geomFrame = (type, get, outer) => {
        let size = numbers(get('size') ?? '0 0 0');
        let frame;
        const fromto = get('fromto');
        if (fromto !== undefined && ['capsule', 'cylinder', 'box', 'ellipsoid'].includes(type)) {
            const [x0, y0, z0, x1, y1, z1] = numbers(fromto);
            const along = [x1 - x0, y1 - y0, z1 - z0];
            const length = Math.hypot(...along);
            const axis = [-along[1], along[0], 0];
            const turn = Math.hypot(...axis) < 1e-12 ? (along[2] < 0 ? [0, 1, 0, 0] : [1, 0, 0, 0]) : fromAxisAngle(axis, Math.acos(Math.max(-1, Math.min(1, along[2] / length))));
            frame = { position: [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], quaternion: turn };
            size = type === 'box' || type === 'ellipsoid' ? [size[0], size[1] ?? size[0], length / 2] : [size[0], length / 2];
        } else {
            frame = frameOf({ pos: get('pos'), quat: get('quat'), axisangle: get('axisangle'), euler: get('euler'), xyaxes: get('xyaxes'), zaxis: get('zaxis') }, placement);
        }
        return { size, frame: compose(outer, frame) };
    };

    // Walks a body's children; `outer` is the frame of any <frame> they sit in, relative to the body.
    const visit = (element, childClass, bodyIndex, world, outer, entry) => {
        const inside = element.attributes.childclass ?? childClass;
        for (const child of element.children) {
            if (child.name === 'frame') {
                const local = frameOf(child.attributes, placement);
                visit(child, child.attributes.childclass ?? inside, bodyIndex, world, compose(outer, local), entry);
            } else if (child.name === 'body') {
                const local = compose(outer, frameOf(child.attributes, placement));
                const body = {
                    name: child.attributes.name ?? `body ${bodies.length + 1}`, parent: bodyIndex, local, world: compose(world, local),
                    joints: [], inertia: null, inertiaFrom: 'none', geoms: [], inertialGeoms: [], explicit: null,
                    mocap: child.attributes.mocap === 'true'
                };
                if (child.attributes.gravcomp !== undefined && Number(child.attributes.gravcomp) !== 0) counts.gravcomp.push(body.name);
                bodies.push(body);
                visit(child, child.attributes.childclass ?? inside, bodies.length - 1, body.world, identity, body);
            } else if (child.name === 'freejoint') {
                if (entry) entry.joints.push({ name: child.attributes.name ?? '', kind: 'free' });
            } else if (child.name === 'joint' && entry) {
                const className = child.attributes.class ?? inside ?? 'main';
                const get = (attribute) => resolve(child, 'joint', className, attribute);
                const kind = get('type') ?? 'hinge';
                if (!['hinge', 'slide', 'ball', 'free'].includes(kind)) throw new Error(`Joint ${child.attributes.name ?? ''} is of type ${kind}, which MuJoCo does not have.`);
                const rangeText = get('range');
                const limited = get('limited') ?? 'auto';
                const range = rangeText === undefined ? null : numbers(rangeText);
                const isLimited = limited === 'true' || (limited === 'auto' && autolimits && range !== null && !(range[0] === 0 && range[1] === 0));
                const reference = Number(get('ref') ?? 0);
                const anchor = compose(outer, { position: numbers(get('pos') ?? '0 0 0'), quaternion: [1, 0, 0, 0] }).position;
                const axis = rotate(outer.quaternion, numbers(get('axis') ?? '0 0 1'));
                entry.joints.push({
                    name: child.attributes.name ?? `${entry.name} joint`, kind, anchor, axis,
                    range: isLimited && range ? (kind === 'slide' ? range : range.map(angle)) : null,
                    damping: Number(get('damping') ?? 0), friction: Number(get('frictionloss') ?? 0), armature: Number(get('armature') ?? 0),
                    reference: kind === 'hinge' ? angle(reference) : reference,
                    stiffness: Number(get('stiffness') ?? 0), springref: Number(get('springref') ?? 0)
                });
            } else if (child.name === 'inertial' && entry) {
                const frame = compose(outer, frameOf(child.attributes, placement));
                const mass = Number(child.attributes.mass ?? 0);
                const tensor = child.attributes.fullinertia !== undefined ? fullTensor(numbers(child.attributes.fullinertia)) : diagonalTensor(numbers(child.attributes.diaginertia ?? '0 0 0'));
                // The tensor is about the centre, along the inertial's own axes: turned into the body's frame.
                entry.explicit = mass > 0 ? placed(mass, tensor.map((value) => value / mass), frame) : { mass: 0, centre: frame.position, tensor: [0, 0, 0, 0, 0, 0, 0, 0, 0] };
            } else if (child.name === 'geom') {
                if (!entry) { counts.worldGeoms += 1; continue; }
                const className = child.attributes.class ?? inside ?? 'main';
                const get = (attribute) => resolve(child, 'geom', className, attribute);
                const type = get('type') ?? (get('mesh') ? 'mesh' : 'sphere');
                const group = Number(get('group') ?? 0);
                if (type === 'plane' || type === 'hfield' || type === 'sdf') continue;
                const { size, frame } = geomFrame(type, get, outer);
                const rgba = get('rgba') !== undefined ? numbers(get('rgba')) : (materials.get(get('material')) ?? [0.5, 0.5, 0.5, 1]);
                const geom = { type, size, frame, mesh: type === 'mesh' ? get('mesh') : null, color: hex(rgba) };
                if (group <= 2 && rgba[3] !== 0) entry.geoms.push(geom);
                if (group >= groupLow && group <= groupHigh) entry.inertialGeoms.push({ ...geom, mass: get('mass') === undefined ? null : Number(get('mass')), density: Number(get('density') ?? 1000) });
            } else if (['site', 'camera', 'light'].includes(child.name)) {
                // Not part of the motion.
            } else if (child.name === 'composite' || child.name === 'flexcomp' || child.name === 'replicate' || child.name === 'attach') {
                throw new Error(`The model uses <${child.name}>, which this reader does not take yet.`);
            }
        }
    };
    for (const world of root.children.filter((child) => child.name === 'worldbody')) visit(world, null, -1, identity, identity, null);

    // Each body's inertia: as given, or from its geoms.
    for (const body of bodies) {
        const fromGeoms = () => {
            const parts = [];
            let mesh = false;
            for (const geom of body.inertialGeoms) {
                let shape;
                if (geom.type === 'mesh') {
                    shape = meshInertia(meshTriangles(geom.mesh));
                    mesh = true;
                    if (!shape) continue;
                    const mass = geom.mass ?? geom.density * shape.volume;
                    const inner = placed(mass, shape.tensor, { position: shape.centre, quaternion: [1, 0, 0, 0] });
                    // The mesh's own centre and tensor, then the geom's frame.
                    const centre = compose(geom.frame, { position: inner.centre, quaternion: [1, 0, 0, 0] }).position;
                    const turned = placed(1, inner.tensor, { position: centre, quaternion: geom.frame.quaternion });
                    parts.push({ mass, centre, tensor: turned.tensor });
                } else {
                    shape = primitiveInertia(geom.type, geom.size);
                    if (!shape) continue;
                    const mass = geom.mass ?? geom.density * shape.volume;
                    parts.push(placed(mass, diagonalTensor(shape.unit), geom.frame));
                }
            }
            return parts.length ? { inertia: combine(parts), from: mesh ? 'meshes' : 'geoms' } : null;
        };
        const useGeoms = inertiaFromGeom === 'true' || (inertiaFromGeom === 'auto' && !body.explicit);
        const found = useGeoms ? fromGeoms() : null;
        if (found) { body.inertia = found.inertia; body.inertiaFrom = found.from; }
        else if (body.explicit) { body.inertia = body.explicit; body.inertiaFrom = 'inertial'; }
        delete body.explicit;
        delete body.inertialGeoms;
        for (const joint of body.joints) if (joint.stiffness) counts.springs.push(joint.name);
    }

    const actuators = [];
    for (const section of root.children.filter((child) => child.name === 'actuator')) {
        for (const element of section.children) {
            if (!actuatorKinds.has(element.name)) continue;
            const className = element.attributes.class ?? 'main';
            const get = (attribute) => resolve(element, element.name, className, attribute) ?? resolve(element, 'general', className, attribute);
            let gain;
            let kind;
            let damping = 0;
            let dampratio = 0;
            if (element.name === 'position') {
                gain = Number(get('kp') ?? 1);
                damping = Number(get('kv') ?? 0);
                // Its damping as a share of critical, which MuJoCo works out on compiling from the joint's inertia.
                dampratio = Number(get('dampratio') ?? 0);
                kind = gain > 0 ? 'position' : 'other';
            } else if (element.name === 'motor') {
                gain = 1;
                kind = 'motor';
            } else if (element.name === 'general') {
                const gainType = get('gaintype') ?? 'fixed';
                const biasType = get('biastype') ?? 'none';
                const gainParameters = numbers(get('gainprm') ?? '1');
                const biasParameters = numbers(get('biasprm') ?? '0');
                gain = gainParameters[0] ?? 1;
                if (gainType === 'fixed' && biasType === 'affine' && gain > 0 && (biasParameters[1] ?? 0) === -gain && (biasParameters[0] ?? 0) === 0) { kind = 'position'; damping = -(biasParameters[2] ?? 0); }
                else if (biasType === 'none' && gainType === 'fixed') kind = 'motor';
                else kind = 'other';
            } else {
                gain = Number(get('kv') ?? get('gain') ?? 1);
                kind = 'other';
            }
            const rangeOf = (name) => {
                const text = get(`${name}range`);
                const limited = get(`${name}limited`) ?? 'auto';
                const range = text === undefined ? null : numbers(text);
                return limited === 'true' || (limited === 'auto' && autolimits && range !== null && !(range[0] === 0 && range[1] === 0)) ? range : null;
            };
            const gear = numbers(get('gear') ?? '1')[0] ?? 1;
            const target = element.attributes.joint ?? null;
            actuators.push({
                name: element.attributes.name ?? '', element: element.name, joint: target, kind, gain, damping, dampratio, gear,
                forceRange: rangeOf('force'), controlRange: rangeOf('ctrl'),
                transmission: target ? 'joint' : ['tendon', 'site', 'body', 'jointinparent'].find((key) => element.attributes[key] !== undefined) ?? 'none'
            });
        }
    }

    // What the model has that the drawing cannot hold yet.
    const children = (name) => root.children.filter((child) => child.name === name).flatMap((child) => child.children);
    const tendons = children('tendon');
    const equalities = children('equality');
    if (counts.worldGeoms) leftOut.push(`${counts.worldGeoms === 1 ? 'a geom' : `${counts.worldGeoms} geoms`} of the world (a floor, say): nothing touches anything yet`);
    if (tendons.length) leftOut.push(`${tendons.length === 1 ? 'a tendon' : `${tendons.length} tendons`} (${tendons.map((tendon) => tendon.attributes.name ?? tendon.name).join(', ')})`);
    for (const equality of equalities) {
        const { name, joint1, joint2, body1, body2 } = equality.attributes;
        const between = [joint1 ?? body1, joint2 ?? body2].filter(Boolean);
        leftOut.push(name ? `the ${equality.name} constraint ${name}` : `the ${equality.name} constraint on ${between.join(' and ') || 'the model'}`);
    }
    if (children('contact').length) leftOut.push('the contact pairs and exclusions: nothing touches anything yet');
    for (const name of counts.springs) leftOut.push(`joint ${name}'s spring (its stiffness): the drawing's joints have none yet`);
    for (const name of counts.gravcomp) leftOut.push(`${name}'s gravity compensation`);
    if (gravity.some((value, axis) => Math.abs(value - [0, 0, -9.81][axis]) > 1e-9)) leftOut.push(`the model's gravity (${gravity.join(' ')}): a drawing has 9.81 m/s² downward`);
    return { name: root.attributes.model ?? null, bodies, actuators, gravity, timestep, leftOut, meshTriangles };
}

// Each body's look: a mesh of its visible geoms in its own frame, merged, and the colour of the largest; null for a
// body with nothing visible. In the canvas's mesh document form, { position, normal, index }.
export function bodyLook(model, body) {
    const builder = new MeshBuilder();
    let largest = { area: -1, color: null };
    for (const geom of body.geoms) {
        const triangles = geom.type === 'mesh' ? model.meshTriangles(geom.mesh) : primitiveTriangles(geom.type, geom.size);
        if (!triangles) continue;
        let area = 0;
        for (const triangle of triangles) {
            const corners = triangle.map((corner) => compose(geom.frame, { position: corner, quaternion: [1, 0, 0, 0] }).position);
            builder.triangle(...corners);
            const [a, b, c] = corners;
            const u = b.map((value, axis) => value - a[axis]);
            const v = c.map((value, axis) => value - a[axis]);
            area += Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]) / 2;
        }
        if (area > largest.area) largest = { area, color: geom.color };
    }
    return builder.index.length ? { mesh: builder.document(), color: largest.color } : null;
}

export { principal };
