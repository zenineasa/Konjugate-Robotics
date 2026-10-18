/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The robot importer, in the two steps the window drives. Its files are a robot's model, as kept in Konjugate's store
// of downloaded files (binary files of the `model` role, each named for its path in the model's folder; see
// lib/catalog.mjs), so every step reads them again:
//
// - describe: what the model says of itself (its joints, the servos on them, its bodies) and its licence, for the
//   window to show and to choose each joint's mode by;
// - build: a mechanism from the joint modes chosen, carrying the model with its licence and where it came from
//   (lib/mechanismModel.mjs), as a project document. And, as asked: a move for its servos (`move`, lib/moves.mjs);
//   its servos' current, held to their datasheet's torque (`servo`, lib/servos.mjs); a battery and the servos'
//   windings (`supply`: { voltage, capacity }); a payload on a body (`payload`: { body, mass }).

import { pathOfStoredName } from '../lib/catalog.mjs';
import { DocumentBuilder } from '../lib/documentBuilder.mjs';
import { placeServoSupply } from '../lib/driveModel.mjs';
import { bodyFramesAt, bodyShapes, describeModel } from '../lib/mjcf.mjs';
import { addRunConfiguration, emptyDocument, placeMechanism, splitModelFiles } from '../lib/mechanismModel.mjs';
import { jointSchedule, moveFor } from '../lib/moves.mjs';
import { servoById } from '../lib/servos.mjs';

const failure = (message) => ({ ok: false, report: { errors: [message], warnings: [] } });

// The model's files by path, from the files given: a name's path is the window's (options.paths, from the stored
// file's details) or, failing that, read from the name.
function modelFiles(files, options) {
    const model = {};
    for (const file of files.filter((candidate) => candidate.role === 'model' && candidate.data)) {
        model[options.paths?.[file.name] ?? pathOfStoredName(file.name)] = file.data;
    }
    return model;
}

const decodeText = (bytes) => (bytes ? new TextDecoder().decode(bytes) : null);

export default async function importRobot({ files, helpers, options = {} }) {
    const all = modelFiles(files, options);
    if (!Object.keys(all).length) return failure('Choose a robot kept on this computer first.');
    const main = options.main ?? 'scene.xml';
    const { run, notices } = splitModelFiles(all);
    let description;
    try {
        description = describeModel(run, main);
    } catch (error) {
        return failure(`The robot's model could not be read: ${error.message}`);
    }
    if (options.step === 'describe') {
        return { ok: true, data: { step: 'describe', description, licence: decodeText(notices.LICENSE) }, report: { errors: [], warnings: [] } };
    }
    if (options.step !== 'build') return failure(`The robot importer has no step "${options.step}".`);

    // Each hinge or slide joint in the mode the window chose (free when it chose none); a ball or free joint (a
    // floating base, say) is MuJoCo's alone.
    const chosen = new Map((options.joints ?? []).map((joint) => [joint.name, joint]));
    const warnings = [];
    const notes = [];
    const move = options.move ? moveFor(options.folder, options.move) : null;
    if (options.move && !move) return failure(`There is no move "${options.move}" for this robot.`);
    const servo = options.servo ? servoById(options.servo) : null;
    if (options.servo && !servo) return failure(`There is no servo "${options.servo}".`);
    if (options.supply && !servo) return failure('A supply needs the servos\' datasheet, to know the current they draw.');
    const joints = description.joints.filter(({ kind, name }) => (kind === 'hinge' || kind === 'slide') && name).map((joint) => {
        const choice = chosen.get(joint.name) ?? { mode: 'free' };
        const range = joint.range ?? (joint.kind === 'hinge' ? [-Math.PI, Math.PI] : [-1, 1]);
        const control = { minimum: range[0], maximum: range[1], step: joint.kind === 'hinge' ? 0.01 : 0.001 };
        const base = { name: joint.name, kind: joint.kind, initialPosition: joint.initialPosition };
        if (choice.mode === 'servo') {
            const servos = description.actuators.filter((actuator) => actuator.joint === joint.name);
            if (servos.length !== 1 || servos[0].kind !== 'position') {
                warnings.push(`${joint.name} has no position servo of its own in the model, so it is left free.`);
                return base;
            }
            const value = Math.min(Math.max(Number(choice.value ?? joint.initialPosition), range[0]), range[1]);
            // A move plays as the setpoint's schedule, and the joint starts where the move does.
            const moved = move ? jointSchedule(move, joint.name) : null;
            return {
                ...base, ...(moved ? { initialPosition: moved.start } : {}),
                setpoint: { value: moved ? moved.start : value, control, ...(moved ? { schedule: moved.schedule } : {}) },
                ...(servo ? { servoCurrent: { torqueConstant: servo.torqueConstant, idleCurrent: servo.idleCurrent } } : {})
            };
        }
        if (choice.mode === 'torque') return { ...base, torque: { value: Number(choice.value ?? 0) } };
        return base;
    });
    if (!joints.length) return failure('The robot has no hinge or slide joint to drive.');

    const { source: template } = await helpers.readPackageJson('engine/mechanismProvider.json');
    let shapes;
    try {
        shapes = bodyShapes(run, main);
    } catch (error) {
        // A robot that cannot be drawn still runs: it is built without its shapes, and the report says why.
        shapes = new Map();
        warnings.push(`The robot's shapes could not be read, so it is built without them: ${error.message}`);
    }
    const document = emptyDocument();
    let nextId = 1;
    const id = () => nextId++;
    // Each body starts where its joints do.
    const frames = bodyFramesAt(description, Object.fromEntries(joints.map(({ name, initialPosition }) => [name, initialPosition])));
    const servoJoints = joints.filter(({ setpoint }) => setpoint);
    const parameters = [];
    if (servo) {
        // Live, so a fork can take a servo's torque away (a worn or a smaller one): from a tenth of its rating to twice it.
        for (const joint of servoJoints) {
            parameters.push({
                name: `${joint.name} servo torque limit`, kind: 'servoTorqueLimit', target: joint.name, value: servo.stallTorque,
                control: { minimum: Math.round(servo.stallTorque * 10) / 100, maximum: Math.round(servo.stallTorque * 200) / 100, step: 0.01 }
            });
        }
        notes.push(`Each servo is held to the ${servo.name}'s ${servo.stallTorque.toFixed(2)} N·m at stall, from its datasheet; the model's own limit is not used.`);
    }
    if (options.payload) {
        const index = description.bodies.indexOf(options.payload.body);
        const own = index >= 0 ? description.tree[index].mass : null;
        if (own === null) return failure(`A payload goes on a body whose mass the model gives; ${options.payload.body} has none given.`);
        parameters.push({
            name: `${options.payload.body} mass with payload`, kind: 'bodyMass', target: options.payload.body, value: own + options.payload.mass,
            control: { minimum: own, maximum: own + 1, step: 0.01 }
        });
        notes.push(`The payload is ${options.payload.mass} kg added to ${options.payload.body}'s own ${own} kg, at its centre of mass: an approximation of a payload held in the gripper.`);
    }
    let placed;
    try {
        placed = placeMechanism(document, id, {
            name: options.name ?? 'Robot', joints, parameters,
            // Every named body reports where it is, and carries how it looks, for Pose Visualizer to show the robot moving.
            bodies: description.bodies.map((name, index) => ({ name, frame: frames[index], shape: shapes.get(name) ?? null })).filter(({ name }) => name),
            model: { main, files: run, provenance: options.provenance ?? null, licence: decodeText(notices.LICENSE) }
        }, { template, helpers });
    } catch (error) {
        return failure(error.message);
    }
    if (options.supply) {
        // A battery, and each servo's winding heated by its current and drawing on the battery.
        const templates = new Map();
        for (const name of ['battery', 'servoWinding', 'servoSupply']) templates.set(name, await helpers.readPackageJson(`engine/components/${name}.json`));
        const builder = new DocumentBuilder(document, templates, helpers);
        const battery = builder.placeNode('battery', { name: 'Battery', initialValues: { voltage: options.supply.voltage ?? servo.voltage } });
        for (const joint of servoJoints) {
            placeServoSupply(builder, {
                battery, mechanism: placed.node, current: placed.joints.get(joint.name).servoCurrent, name: joint.name,
                values: { resistance: servo.windingResistance, capacity: options.supply.capacity ?? 7200 }
            });
        }
        nextId = builder.nextId;
        notes.push(`The windings' resistance (${servo.windingResistance} Ω) is worked out from the datasheet; their heat capacity and their resistance to ambient are assumed.`);
    }
    addRunConfiguration(document, id, { globalTimeStep: options.timeStep ?? 0.01, targetTime: options.runTime ?? (move ? move.keyframes.at(-1)[0] + 1 : 10) });
    const drivenCount = joints.filter((joint) => joint.setpoint || joint.torque).length;
    // What a scenario may change, by what it is of: each servo's torque limit by its joint, and the payload by the body
    // it is on.
    const parameterIndex = [];
    const index = (key, entity, shared) => parameterIndex.push({
        key, entity, name: shared.name, symbol: shared.symbol, sharedParameterId: shared.id, scope: 'instance', value: shared.value, unit: shared.unit,
        live: shared.mode === 'live', minimum: shared.control?.minimum ?? null, maximum: shared.control?.maximum ?? null
    });
    if (servo) for (const joint of servoJoints) index('servoTorqueLimit', joint.name, placed.parameters.get(`${joint.name} servo torque limit`));
    if (options.payload) index('payloadMass', options.payload.body, placed.parameters.get(`${options.payload.body} mass with payload`));
    return {
        ok: true, document, parameterIndex,
        data: {
            step: 'build', joints: joints.length, driven: drivenCount,
            // For a scenario's comparison: the mechanism, its servo joints by their states' prefix, their rating and the payload.
            mechanism: options.name ?? 'Robot',
            servos: servo ? servoJoints.map(({ name }) => ({ name, key: placed.joints.get(name).servoTorque.symbol.replace(/ServoTorque$/, '') })) : [],
            stallTorque: servo?.stallTorque ?? null,
            payload: options.payload ? { body: options.payload.body, own: placed.parameters.get(`${options.payload.body} mass with payload`).value - options.payload.mass, mass: options.payload.mass } : null,
            runTime: document.runConfigurations.at(-1).targetTime
        },
        report: { errors: [], warnings, notes, summary: `${options.name ?? 'The robot'}: ${joints.length} joints, ${drivenCount} driven` }
    };
}
