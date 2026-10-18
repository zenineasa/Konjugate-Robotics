/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Builds a Konjugate project document for one mechanism solved by MuJoCo: one node whose computational-node provider
// (packages/engine/providers/mechanism.py) holds the MuJoCo model. For each joint the node has its position and speed,
// set by the provider at the end of each substep (algebraic, `setsValue`), in radians or metres per the joint.
//
// A joint is in one of three modes, by what it is given:
// - torque: driven by a torque (a force, for a slide), `torque: { value, control? }` (a state of the mechanism's own set
//   from a shared parameter, live with a slider) or `torqueFrom: { nodeId, stateId }` (another node's state, such as a
//   motor's, read from the step's snapshot);
// - servo: driven by the model's own position actuator on the joint (a servo's inner loop, run by MuJoCo at its own
//   timestep) to a setpoint, `setpoint` (which may follow a stored schedule, a move) or `setpointFrom` in the same two
//   ways; the node also gets the torque the servo applies, as a state, and with `servoCurrent: { torqueConstant,
//   idleCurrent }` (N·m per A at its output, A) the current it draws, each constant a shared parameter;
// - free: neither, and any actuator the model has on the joint is switched off, so nothing drives it that the model
//   does not say.
// The provider reads a torque or setpoint at the start of each substep.
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

// A model's files as the provider carries them, by path relative to the model's folder: base64, as the project is JSON.
// No dependencies, so it runs in the window as in Node.
export function base64(bytes) {
    let binary = '';
    for (let start = 0; start < bytes.length; start += 0x8000) binary += String.fromCharCode(...bytes.subarray(start, start + 0x8000));
    return btoa(binary);
}

// Of a model folder's files, the ones a run reads (its MJCF and what that uses: meshes, textures, height fields), and
// apart from them the notices that travel with them (its licence, readme and changelog, and our provenance record).
const notices = /^(LICENSE|COPYING|NOTICE)(\.[a-z]+)?$|\.(md|txt)$|^provenance\.json$/i;
export function splitModelFiles(files) {
    const run = {};
    const kept = {};
    for (const [path, bytes] of Object.entries(files)) (notices.test(path.split('/').pop()) && !path.includes('/') ? kept : run)[path] = bytes;
    return { run, notices: kept };
}

// A body's pose as states: position (m, z up), then orientation as a unit quaternion, as Pose Visualizer reads them.
const poseParts = ['X', 'Y', 'Z', 'Qw', 'Qx', 'Qy', 'Qz'];
const poseLabels = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'];

// The unit of each quantity of MuJoCo's model a parameter may set, for a hinge (a slide's damping is N·s/m).
const parameterUnits = {
    bodyMass: 'kg', jointDamping: 'N·m·s/rad', jointFrictionloss: 'N·m', jointArmature: 'kg·m²',
    servoTorqueLimit: 'N·m', servoTorqueConstant: 'N·m/A', servoIdleCurrent: 'A'
};

const units = {
    hinge: { position: 'rad', speed: 'rad/s', effort: 'N·m', setpoint: 'rad', command: 'rad', servoTorque: 'N·m', servoCurrent: 'A' },
    slide: { position: 'm', speed: 'm/s', effort: 'N', setpoint: 'm', command: 'm', servoTorque: 'N', servoCurrent: 'A' }
};

// Places one mechanism's node (and the shared parameters its joints' torques come from) into `document`, taking ids
// from `id()`. `mechanism` is { name, mjcf or model, joints: [{ name, kind: 'hinge' | 'slide', initialPosition,
// initialSpeed, torque or torqueFrom or setpoint or setpointFrom }] }, where `torque` and `setpoint` are
// { value, control?: { minimum, maximum, step } } and `torqueFrom` and `setpointFrom` are { nodeId, stateId }. A model
// is given as MJCF text, or as `model`: { main (the path of its MJCF), files ({ path: bytes }, what a run reads, as
// splitModelFiles gives them), provenance (where it came from: source, repository, folder, commit, licence), licence
// (the licence's text) }. The project then carries the model, so it runs wherever it is opened, and carries its licence
// and provenance with it. `mechanism.bodies` are the bodies whose place in the world the node reports, as seven states
// each (<body>X, Y, Z in metres and Qw, Qx, Qy, Qz): [{ name, frame, shape }], `frame` their place at the reference
// pose, { position, quaternion }, as describeModel gives it, and `shape` how they look, { mesh, color }, as bodyShapes
// gives it: kept as the node's appearance parts, by the same prefix as its pose states, for Pose Visualizer to draw. `mechanism.parameters` are quantities of MuJoCo's model the run may change:
// [{ name, kind: 'bodyMass' | 'jointDamping' | 'jointFrictionloss' | 'jointArmature', target (the body or joint), value,
// unit?, control?, schedule? }], each a shared parameter the provider reads (or `sharedParameterId`, one already in the
// document). `template` is mechanism.py's text; `helpers` are Konjugate's equation helpers (validateEquationLatex is
// used). Returns the node, by joint name its mode and its position, speed, input and servo torque states, and by
// name its shared parameters.
export function placeMechanism(document, id, mechanism, { template, helpers }) {
    if (!helpers?.validateEquationLatex) throw new Error('The mechanism builder needs Konjugate\'s equation helpers.');
    const keys = new Set();
    const node = {
        id: id(), name: mechanism.name ?? 'Mechanism', type: 'mechanism', position: [0, 0, 0], states: [], sourceTerms: [], numerics: { substepsPerGlobalStep: 1 },
        appearance: { type: 'primitive', shape: 'box', color: '#5b8db8' }
    };
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
        const given = ['torque', 'torqueFrom', 'setpoint', 'setpointFrom'].filter((field) => joint[field] !== undefined);
        if (given.length > 1) throw new Error(`Joint ${joint.name} is given ${given.join(' and ')}; it takes one.`);
        const mode = joint.torque || joint.torqueFrom ? 'torque' : joint.setpoint || joint.setpointFrom ? 'servo' : 'free';
        // The input the provider reads, as a state of the mechanism's own when it comes from a shared parameter. Named
        // apart from that parameter (a setpoint's state is its "command", a torque's its "effort"): the canvas binds a
        // node's every state to each of its source terms, as its editor does, and a state of the parameter's name would
        // take the name from it, so the term would read its own output.
        const role = mode === 'servo' ? 'command' : 'effort';
        const own = joint.torque ?? joint.setpoint;
        const input = own ? state(role, mode === 'servo' ? 'setpoint command' : kind === 'slide' ? 'force' : 'torque') : null;
        const servoTorque = mode === 'servo' ? state('servoTorque', kind === 'slide' ? 'servo force' : 'servo torque') : null;
        if (joint.servoCurrent && mode !== 'servo') throw new Error(`Joint ${joint.name} is given a servo's current but is not in servo mode.`);
        // The datasheet's constants, apart from the state of the current they give.
        const servoConstants = joint.servoCurrent ?? null;
        const servoCurrent = servoConstants ? state('servoCurrent', 'servo current') : null;
        return { ...joint, kind, key, mode, role, own, input, from: joint.torqueFrom ?? joint.setpointFrom, position, speed, servoTorque, servoCurrent, servoConstants };
    });
    for (const joint of joints.filter(({ own }) => own)) {
        const symbol = `${joint.key}${joint.mode === 'servo' ? 'Setpoint' : joint.kind === 'slide' ? 'Force' : 'Torque'}`;
        const live = Boolean(joint.own.control);
        const shared = {
            id: id(), name: `${joint.name} ${joint.mode === 'servo' ? 'setpoint' : joint.kind === 'slide' ? 'force' : 'torque'}`, symbol, value: joint.own.value ?? 0,
            unit: units[joint.kind][joint.mode === 'servo' ? 'setpoint' : 'effort'], mode: live ? 'live' : 'constant', ...(live ? { control: { ...joint.own.control } } : {}),
            // A move, as a stored schedule: { interpolation: 'linear' | 'hold', samples: [[time, value], ...] }.
            ...(joint.own.schedule ? { schedule: structuredClone(joint.own.schedule) } : {})
        };
        sharedParameters.push(shared);
        const parameter = { id: id(), name: shared.name, symbol, value: shared.value, unit: shared.unit, mode: shared.mode, ...(live ? { control: { ...shared.control } } : {}), sharedParameterId: shared.id };
        const latex = `\\mathrm{${symbol}}`;
        joint.input.initialValue = shared.value;
        const bindings = [{ kind: 'parameter', parameterId: parameter.id, symbol }];
        const validation = helpers.validateEquationLatex(latex, bindings);
        if (!validation.valid) throw new Error(`${joint.name} ${joint.role}: ${validation.errors.join(' ')}`);
        node.sourceTerms.push({
            id: id(), state: joint.input.symbol, expression: latex, parameters: [parameter], setsValue: true,
            expressionModel: { latex, bindings, output: { stateId: joint.input.id }, mathJson: validation.mathJson }
        });
    }
    if ((mechanism.mjcf === undefined) === (mechanism.model === undefined)) throw new Error('A mechanism is given as MJCF text or as a model\'s files, one of the two.');
    if (mechanism.model && !(mechanism.model.main in mechanism.model.files)) throw new Error(`The model has no file ${mechanism.model.main}.`);
    const configuration = {
        providerId: 'konjugate.robotics.mechanism', name: node.name,
        ...(mechanism.mjcf !== undefined ? { mjcf: mechanism.mjcf } : {
            model: {
                main: mechanism.model.main,
                files: Object.fromEntries(Object.entries(mechanism.model.files).map(([path, bytes]) => [path, base64(bytes)])),
                provenance: mechanism.model.provenance ?? null,
                licence: mechanism.model.licence ?? null
            }
        }),
        joints: joints.map(({ name, key, initialPosition = 0, initialSpeed = 0, mode, servoCurrent }) => ({ name, key, initialPosition, initialSpeed, mode, current: Boolean(servoCurrent) })),
        parameters: []
    };
    const inputKey = (joint) => `${joint.key}${joint.mode === 'servo' ? 'Setpoint' : 'Effort'}`;
    node.implementation = {
        kind: 'python', providerApiVersion: 1, source: '', // written once the parameters are in the configuration, below
        bindings: joints.filter(({ input, from }) => input || from).map((joint) => (joint.input
            ? { key: inputKey(joint), kind: 'state', nodeId: node.id, stateId: joint.input.id }
            : { key: inputKey(joint), kind: 'state', nodeId: joint.from.nodeId, stateId: joint.from.stateId })),
        outputs: joints.flatMap((joint) => [
            { key: `${joint.key}Position`, stateId: joint.position.id, setsValue: true },
            { key: `${joint.key}Speed`, stateId: joint.speed.id, setsValue: true },
            ...(joint.servoTorque ? [{ key: `${joint.key}ServoTorque`, stateId: joint.servoTorque.id, setsValue: true }] : []),
            ...(joint.servoCurrent ? [{ key: `${joint.key}ServoCurrent`, stateId: joint.servoCurrent.id, setsValue: true }] : [])
        ])
    };
    // Each body to report, by its name in the model, with its place at the reference pose (describeModel's frames) as
    // its states' starting values: the provider sets them at the end of every step, not before the first.
    const bodyKeys = new Set();
    configuration.bodies = [];
    const bodies = new Map();
    for (const body of mechanism.bodies ?? []) {
        if (!body.name) continue;
        const key = jointKey(body.name, bodyKeys);
        const pose = [...(body.frame?.position ?? [0, 0, 0]), ...(body.frame?.quaternion ?? [1, 0, 0, 0])];
        const states = poseParts.map((part, index) => {
            const created = { id: id(), name: `${body.name} ${poseLabels[index]}`, symbol: `${key}${part}`, initialValue: pose[index], unit: index < 3 ? 'm' : '' };
            node.states.push(created);
            node.implementation.outputs.push({ key: created.symbol, stateId: created.id, setsValue: true });
            return created;
        });
        configuration.bodies.push({ name: body.name, key });
        bodies.set(body.name, { key, states });
        if (body.shape?.mesh?.position?.length) {
            node.appearance.parts ??= [];
            node.appearance.parts.push({ prefix: key, mesh: body.shape.mesh, ...(body.shape.color ? { color: body.shape.color } : {}) });
        }
    }

    // A servo's current reads its torque constant and idle current, each a shared parameter of its own.
    const servoParameters = joints.filter(({ servoConstants }) => servoConstants).flatMap((joint) => {
        if (!(joint.servoConstants.torqueConstant > 0) || !(joint.servoConstants.idleCurrent >= 0)) {
            throw new Error(`Joint ${joint.name}'s servo needs a positive torque constant and an idle current of zero or more.`);
        }
        return [
            { name: `${joint.name} servo torque constant`, kind: 'servoTorqueConstant', target: joint.name, value: joint.servoConstants.torqueConstant },
            { name: `${joint.name} servo idle current`, kind: 'servoIdleCurrent', target: joint.name, value: joint.servoConstants.idleCurrent }
        ];
    });
    const parameters = new Map();
    for (const parameter of [...(mechanism.parameters ?? []), ...servoParameters]) {
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
    return { node, parameters, bodies, joints: new Map(joints.map(({ name, mode, position, speed, input, servoTorque, servoCurrent }) => [name, { mode, position, speed, input, servoTorque, servoCurrent }])) };
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
