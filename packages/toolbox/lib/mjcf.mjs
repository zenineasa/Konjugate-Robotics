/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Reads a MuJoCo model (MJCF) for what the window and the importer need to build a mechanism from it: its joints
// (name, kind, range, starting position), the actuators on them (a position servo, a motor or another kind, with its
// gain and force limit) and its bodies. MuJoCo itself compiles the model when it runs; this reads the same files in
// JavaScript, where MuJoCo is not, resolving what decides those answers: included files, the compiler's angle unit,
// default classes (nested, and a body's childclass for what is inside it) and the actuator shortcuts. It agrees with
// MuJoCo's own reading (packages/engine/tools/describeMechanism.py) on the models it is tested on; a model it cannot
// read is refused with the reason, not guessed at.
//
// No dependencies: a small XML reader of its own, for MJCF's plain elements and attributes.

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
const normalise = (path) => {
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
            const path = normalise(base + child.attributes.file);
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

// What a run of this model needs to know of it to build a mechanism: { joints, actuators, bodies }, as
// describeMechanism.py gives it from MuJoCo. `files` maps paths in the model's folder to text or bytes; `main` is
// the model's MJCF among them (its scene file, say, which includes the robot).
export function describeModel(files, main) {
    const root = resolveIncludes(main, files);
    const classes = defaultClasses(root);
    const resolve = resolver(classes);
    const compiler = root.children.filter((child) => child.name === 'compiler').reduce((all, child) => ({ ...all, ...child.attributes }), {});
    const degrees = (compiler.angle ?? 'degree') === 'degree';
    const angle = (value) => (degrees ? (value * Math.PI) / 180 : value);

    const joints = [];
    const bodies = [];
    const visitBody = (body, childClass) => {
        const inside = body.attributes.childclass ?? childClass;
        for (const child of body.children) {
            if (child.name === 'body') {
                bodies.push(child.attributes.name ?? '');
                visitBody(child, inside);
            } else if (child.name === 'freejoint') {
                joints.push({ name: child.attributes.name ?? '', kind: 'free', range: null, initialPosition: 0 });
            } else if (child.name === 'joint') {
                const className = child.attributes.class ?? inside;
                const kind = resolve(child, 'joint', className, 'type') ?? 'hinge';
                if (!['hinge', 'slide', 'ball', 'free'].includes(kind)) throw new Error(`Joint ${child.attributes.name ?? ''} is of type ${kind}, which MuJoCo does not have.`);
                const rangeText = resolve(child, 'joint', className, 'range');
                const limited = resolve(child, 'joint', className, 'limited') ?? 'auto';
                const range = rangeText === undefined ? null : numbers(rangeText);
                const isLimited = limited === 'true' || (limited === 'auto' && range !== null && !(range[0] === 0 && range[1] === 0));
                // A hinge's range is in the compiler's angle unit; a slide's in metres; a ball's maximum angle in the unit too.
                const converted = isLimited && range ? (kind === 'slide' ? range : range.map(angle)) : null;
                const reference = Number(resolve(child, 'joint', className, 'ref') ?? 0);
                joints.push({
                    name: child.attributes.name ?? '', kind,
                    range: converted,
                    initialPosition: kind === 'hinge' ? angle(reference) : kind === 'slide' ? reference : 0
                });
            }
        }
    };
    const worldbody = root.children.filter((child) => child.name === 'worldbody');
    for (const world of worldbody) visitBody(world, null);

    const actuators = [];
    for (const section of root.children.filter((child) => child.name === 'actuator')) {
        for (const element of section.children) {
            if (!actuatorKinds.has(element.name)) continue;
            const className = element.attributes.class ?? 'main';
            const get = (attribute) => resolve(element, element.name, className, attribute) ?? resolve(element, 'general', className, attribute);
            let gain;
            let kind;
            if (element.name === 'position') {
                gain = Number(get('kp') ?? 1);
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
                if (gainType === 'fixed' && biasType === 'affine' && gain > 0 && (biasParameters[1] ?? 0) === -gain) kind = 'position';
                else if (biasType === 'none' && gain === 1) kind = 'motor';
                else kind = 'other';
            } else {
                gain = Number(get('kv') ?? get('gain') ?? 1);
                kind = 'other';
            }
            const target = element.attributes.joint ?? null;
            const forceRangeText = get('forcerange');
            const forceLimited = get('forcelimited') ?? 'auto';
            const forceRange = forceRangeText === undefined ? null : numbers(forceRangeText);
            const isLimited = forceLimited === 'true' || (forceLimited === 'auto' && forceRange !== null && !(forceRange[0] === 0 && forceRange[1] === 0));
            actuators.push({ name: element.attributes.name ?? '', joint: target, kind, gain, forceRange: isLimited ? forceRange : null });
        }
    }
    return { joints, actuators, bodies };
}
