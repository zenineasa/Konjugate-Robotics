/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Builds a Konjugate project document for one mechanism solved by MuJoCo: one node whose computational-node provider
// (packages/engine/providers/mechanism.py) holds the MuJoCo model. For each joint the node has its position and speed,
// set by the provider at the end of each substep (algebraic, `setsValue`), in radians or metres per the joint.
//
// A joint may be driven, by a torque (a force, for a slide) the provider reads at the start of each substep:
// - `torque: { value, control? }`, a state of the mechanism's own set from a shared parameter (live with a slider);
// - `torqueFrom: { nodeId, stateId }`, another node's state, such as a motor's, read from the step's snapshot.
//
// One node, not a subsystem of joint and solid nodes, because a computational-node provider writes only its own node's
// states (see docs/direction.md, "Needs from core"). It has no dependencies of its own: the equation helpers come from
// the host (an importer receives them) or, in scripts and tests, from the Konjugate checkout.

// A joint's name as a lower camel case key ("left elbow" and "left_elbow" are "leftElbow"), unique among `taken`.
export function jointKey(name, taken = new Set()) {
    const words = String(name).split(/[^A-Za-z0-9]+/).filter(Boolean);
    let key = words.map((word, index) => (index ? word[0].toUpperCase() + word.slice(1) : word[0].toLowerCase() + word.slice(1))).join('') || 'joint';
    if (!/^[a-z]/.test(key)) key = `joint${key[0].toUpperCase()}${key.slice(1)}`;
    let unique = key;
    for (let suffix = 2; taken.has(unique); suffix += 1) unique = `${key}${suffix}`;
    taken.add(unique);
    return unique;
}

// The provider's source with its configuration written in. JSON escapes every quote, so the configuration can never
// close the raw string it sits in.
export function providerSource(template, configuration) {
    const marker = "CONFIGURATION = json.loads(r'''{}''')";
    if (!template.includes(marker)) throw new Error('The mechanism provider has no configuration line to fill.');
    const json = JSON.stringify(configuration).replaceAll("'", '\\u0027');
    return template.replace(marker, () => `CONFIGURATION = json.loads(r'''${json}''')`);
}

// The unit of each quantity of MuJoCo's model a parameter may set, for a hinge (a slide's damping is N·s/m).
const parameterUnits = { bodyMass: 'kg', jointDamping: 'N·m·s/rad', jointFrictionloss: 'N·m', jointArmature: 'kg·m²' };

const units = { hinge: { position: 'rad', speed: 'rad/s', effort: 'N·m' }, slide: { position: 'm', speed: 'm/s', effort: 'N' } };

// Places one mechanism's node (and the shared parameters its joints' torques come from) into `document`, taking ids
// from `id()`. `mechanism` is { name, mjcf, joints: [{ name, kind: 'hinge' | 'slide', initialPosition, initialSpeed,
// torque, torqueFrom }] }, where `torque` is { value, control?: { minimum, maximum, step } } and `torqueFrom` is
// { nodeId, stateId }. `mechanism.parameters` are quantities of MuJoCo's model the run may change:
// [{ name, kind: 'bodyMass' | 'jointDamping' | 'jointFrictionloss' | 'jointArmature', target (the body or joint), value,
// unit?, control?, schedule? }], each a shared parameter the provider reads (or `sharedParameterId`, one already in the
// document). `template` is mechanism.py's text; `helpers` are Konjugate's equation helpers (validateEquationLatex is
// used). Returns the node, by joint name its position, speed and effort states, and by name its shared parameters.
export function placeMechanism(document, id, mechanism, { template, helpers }) {
    if (!helpers?.validateEquationLatex) throw new Error('The mechanism builder needs Konjugate\'s equation helpers.');
    const keys = new Set();
    const node = { id: id(), name: mechanism.name ?? 'Mechanism', type: 'mechanism', position: [0, 0, 0], states: [], sourceTerms: [], numerics: { substepsPerGlobalStep: 1 }, appearance: {} };
    document.sharedParameters ??= [];
    const sharedParameters = document.sharedParameters;
    const joints = mechanism.joints.map((joint) => {
        const kind = joint.kind === 'slide' ? 'slide' : 'hinge';
        const key = jointKey(joint.name, keys);
        const state = (role, label) => {
            const created = { id: id(), name: `${joint.name} ${label}`, symbol: `${key}${role[0].toUpperCase()}${role.slice(1)}`, initialValue: 0, unit: units[kind][role] };
            node.states.push(created);
            return created;
        };
        const position = state('position', 'position');
        position.initialValue = joint.initialPosition ?? 0;
        const speed = state('speed', 'speed');
        speed.initialValue = joint.initialSpeed ?? 0;
        if (joint.torque && joint.torqueFrom) throw new Error(`Joint ${joint.name} is given both a torque and a state to read one from.`);
        const effort = joint.torque ? state('effort', joint.kind === 'slide' ? 'force' : 'torque') : null;
        return { ...joint, kind, key, position, speed, effort };
    });
    for (const joint of joints.filter(({ torque }) => torque)) {
        const symbol = `${joint.key}${joint.kind === 'slide' ? 'Force' : 'Torque'}`;
        const live = Boolean(joint.torque.control);
        const shared = { id: id(), name: `${joint.name} ${joint.kind === 'slide' ? 'force' : 'torque'}`, symbol, value: joint.torque.value ?? 0, unit: units[joint.kind].effort, mode: live ? 'live' : 'constant', ...(live ? { control: { ...joint.torque.control } } : {}) };
        sharedParameters.push(shared);
        const parameter = { id: id(), name: shared.name, symbol, value: shared.value, unit: shared.unit, mode: shared.mode, ...(live ? { control: { ...shared.control } } : {}), sharedParameterId: shared.id };
        const latex = `\\mathrm{${symbol}}`;
        joint.effort.initialValue = shared.value;
        const bindings = [{ kind: 'parameter', parameterId: parameter.id, symbol }];
        const validation = helpers.validateEquationLatex(latex, bindings);
        if (!validation.valid) throw new Error(`${joint.name} torque: ${validation.errors.join(' ')}`);
        node.sourceTerms.push({
            id: id(), state: joint.effort.symbol, expression: latex, parameters: [parameter], setsValue: true,
            expressionModel: { latex, bindings, output: { stateId: joint.effort.id }, mathJson: validation.mathJson }
        });
    }
    const configuration = {
        providerId: 'konjugate.robotics.mechanism', name: node.name, mjcf: mechanism.mjcf,
        joints: joints.map(({ name, key, initialPosition = 0, initialSpeed = 0, effort, torqueFrom }) => ({ name, key, initialPosition, initialSpeed, driven: Boolean(effort || torqueFrom) })),
        parameters: []
    };
    node.implementation = {
        kind: 'python', providerApiVersion: 1, source: '', // written once the parameters are in the configuration, below
        bindings: joints.filter(({ effort, torqueFrom }) => effort || torqueFrom).map((joint) => (joint.effort
            ? { key: `${joint.key}Effort`, kind: 'state', nodeId: node.id, stateId: joint.effort.id }
            : { key: `${joint.key}Effort`, kind: 'state', nodeId: joint.torqueFrom.nodeId, stateId: joint.torqueFrom.stateId })),
        outputs: joints.flatMap((joint) => [
            { key: `${joint.key}Position`, stateId: joint.position.id, setsValue: true },
            { key: `${joint.key}Speed`, stateId: joint.speed.id, setsValue: true }
        ])
    };
    const parameters = new Map();
    for (const parameter of mechanism.parameters ?? []) {
        if (!parameterUnits[parameter.kind]) throw new Error(`Parameter ${parameter.name} sets ${parameter.kind}, which the mechanism does not know.`);
        const key = jointKey(parameter.name, keys);
        let shared = parameter.sharedParameterId === undefined ? null : sharedParameters.find(({ id: candidate }) => candidate === parameter.sharedParameterId);
        if (parameter.sharedParameterId !== undefined && !shared) throw new Error(`Parameter ${parameter.name} links to a shared parameter that does not exist.`);
        if (!shared) {
            let symbol = key;
            for (let suffix = 2; sharedParameters.some((candidate) => candidate.symbol === symbol); suffix += 1) symbol = `${key}${suffix}`;
            const mode = parameter.control ? 'live' : 'constant';
            shared = {
                id: id(), name: parameter.name, symbol, value: parameter.value, unit: parameter.unit ?? parameterUnits[parameter.kind], mode,
                ...(parameter.control ? { control: { ...parameter.control } } : {}), ...(parameter.schedule ? { schedule: structuredClone(parameter.schedule) } : {})
            };
            sharedParameters.push(shared);
        }
        node.implementation.bindings.push({ key, kind: 'parameter', sharedParameterId: shared.id });
        configuration.parameters.push({ key, kind: parameter.kind, target: parameter.target });
        parameters.set(parameter.name, shared);
    }
    node.implementation.source = providerSource(template, configuration);
    (document.nodes ??= []).push(node);
    return { node, parameters, joints: new Map(joints.map(({ name, position, speed, effort }) => [name, { position, speed, effort }])) };
}

// A run configuration for `document`, made its active one. Time is in seconds.
export function addRunConfiguration(document, id, { globalTimeStep = 0.01, outputInterval = globalTimeStep, targetTime = 10 } = {}) {
    const runConfigurationId = id();
    document.runConfigurations = [...(document.runConfigurations ?? []), { id: runConfigurationId, name: `${targetTime} s`, targetTime, globalTimeStep, outputInterval }];
    document.activeRunConfigurationId = runConfigurationId;
}

export function emptyDocument() {
    return { format: 'konjugate', version: 1, copyright: 'Copyright © 2026 Zenin Easa Panthakkalakath', metadata: { units: 'SI' }, nodes: [], edges: [], sharedParameters: [] };
}

// A document of one mechanism alone, ids from `firstId`, so a caller can put nodes of its own before it.
export function buildMechanismDocument(mechanism, { template, helpers, globalTimeStep = 0.01, outputInterval = globalTimeStep, targetTime = 10, firstId = 1 }) {
    let nextId = firstId;
    const id = () => nextId++;
    const document = emptyDocument();
    placeMechanism(document, id, mechanism, { template, helpers });
    addRunConfiguration(document, id, { globalTimeStep, outputInterval, targetTime });
    return document;
}
