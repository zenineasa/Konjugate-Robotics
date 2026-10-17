/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The robot importer, in the two steps the window drives. Its files are a robot's model, as kept in Konjugate's store
// of downloaded files (binary files of the `model` role, each named for its path in the model's folder; see
// lib/catalog.mjs), so every step reads them again:
//
// - describe: what the model says of itself (its joints, the servos on them, its bodies) and its licence, for the
//   window to show and to choose each joint's mode by;
// - build: a mechanism from the joint modes chosen, carrying the model with its licence and where it came from
//   (lib/mechanismModel.mjs), as a project document.

import { pathOfStoredName } from '../lib/catalog.mjs';
import { describeModel } from '../lib/mjcf.mjs';
import { addRunConfiguration, emptyDocument, placeMechanism, splitModelFiles } from '../lib/mechanismModel.mjs';

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
            return { ...base, setpoint: { value, control } };
        }
        if (choice.mode === 'torque') return { ...base, torque: { value: Number(choice.value ?? 0) } };
        return base;
    });
    if (!joints.length) return failure('The robot has no hinge or slide joint to drive.');

    const { source: template } = await helpers.readPackageJson('engine/mechanismProvider.json');
    const document = emptyDocument();
    let nextId = 1;
    const id = () => nextId++;
    try {
        placeMechanism(document, id, {
            name: options.name ?? 'Robot', joints,
            model: { main, files: run, provenance: options.provenance ?? null, licence: decodeText(notices.LICENSE) }
        }, { template, helpers });
    } catch (error) {
        return failure(error.message);
    }
    addRunConfiguration(document, id, { globalTimeStep: options.timeStep ?? 0.01, targetTime: options.runTime ?? 10 });
    const drivenCount = joints.filter((joint) => joint.setpoint || joint.torque).length;
    return {
        ok: true, document,
        data: { step: 'build', joints: joints.length, driven: drivenCount },
        report: { errors: [], warnings, summary: `${options.name ?? 'The robot'}: ${joints.length} joints, ${drivenCount} driven` }
    };
}
