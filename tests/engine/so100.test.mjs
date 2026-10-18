/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A real robot from MuJoCo Menagerie, The Robot Studio's SO-ARM100, fetched at a pinned commit as the toolbox fetches
// one at a user's request (kept on this computer, never shipped), carried in the project with its licence and where it
// came from, as an importer reads it from Konjugate's store, and run in Konjugate's engine with its own servos:
// each joint's position actuator, run by MuJoCo at its own timestep, drives the joint to a setpoint from a shared
// parameter. It runs as MuJoCo alone runs the same model file, positions and servo torques alike. Skipped, saying why,
// when the model is not on this computer and cannot be fetched.

import assert from 'node:assert/strict';
import { mkdtemp, readdir, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { menagerieModel } from '../../scripts/fetchMenagerieModel.mjs';
import { buildMechanismDocument, splitModelFiles } from '../../packages/toolbox/lib/mechanismModel.mjs';
import { describeModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { helpers, providerTemplate, runDocument, runReference } from './harness.mjs';

// Menagerie's main branch on 17 October 2026.
const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';

let model = null;
let unavailable = null;
try {
    model = await menagerieModel('trs_so_arm100', { commit });
} catch (error) {
    unavailable = `the SO-ARM100 is not on this computer and could not be fetched (${error.message})`;
}

// Five joints held by their servos at setpoints within their ranges; the wrist's roll left free, its servo switched off.
const setpoints = { Rotation: 0.5, Pitch: -1.0, Elbow: 1.2, Wrist_Pitch: 0.3, Jaw: 0.4 };
const servoLimit = 3.5;
const targetTime = 2;

test('the SO-ARM100 is fetched at a pinned commit with its licence and where it came from', { skip: unavailable ?? false }, () => {
    assert.equal(model.provenance.source, 'MuJoCo Menagerie');
    assert.equal(model.provenance.commit, commit);
    assert.equal(model.provenance.licence.name, 'Apache-2.0');
    assert.ok(model.provenance.files.some(({ path }) => path === 'so_arm100.xml'));
    assert.ok(!model.provenance.files.some(({ path }) => /\.png$/.test(path)), 'Menagerie\'s own pictures are left out.');
});

// The fetched folder's files, by path relative to it, as an importer is given them from the store.
async function folderFiles(directory) {
    const entries = await readdir(directory, { recursive: true, withFileTypes: true });
    const files = {};
    for (const entry of entries.filter((candidate) => candidate.isFile())) {
        const path = join(entry.parentPath, entry.name);
        files[path.slice(directory.length + 1).split('\\').join('/')] = new Uint8Array(await readFile(path));
    }
    return files;
}

test('the SO-ARM100, driven by its own servos to setpoints, runs in Konjugate as MuJoCo alone runs it', { skip: unavailable ?? false }, async () => {
    const { run: files, notices } = splitModelFiles(await folderFiles(model.directory));
    assert.ok('scene.xml' in files && 'so_arm100.xml' in files && 'assets/Base.stl' in files);
    assert.deepEqual(Object.keys(notices).sort(), ['CHANGELOG.md', 'LICENSE', 'README.md', 'provenance.json']);
    const joints = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw'].map((name) => (name in setpoints
        ? { name, setpoint: { value: setpoints[name], control: { minimum: -3.3, maximum: 3.2, step: 0.01 } } }
        : { name }));
    const licence = new TextDecoder().decode(notices.LICENSE);
    const description = describeModel(files, 'scene.xml');
    const bodies = description.bodies.map((name, index) => ({ name, frame: description.frames[index] }));
    const document = buildMechanismDocument({ name: 'SO-ARM100', model: { main: 'scene.xml', files, provenance: model.provenance, licence }, joints, bodies },
        { template: providerTemplate, helpers, globalTimeStep: 0.01, targetTime });
    // The project carries the robot, and its licence and provenance with it.
    const source = document.nodes[0].implementation.source;
    assert.ok(source.includes('"licence":"') && source.includes(model.provenance.commit) && source.includes('Apache License'));
    const modelPath = join(model.directory, 'scene.xml');
    const directory = await mkdtemp(join(tmpdir(), 'robotics-so100-'));
    const run = await runDocument(directory, 'so100', document);
    const reference = await runReference(directory, { modelPath, joints: joints.map(({ name }) => ({ name, initialPosition: 0 })), setpoints, times: run.times, bodies: description.bodies }, 'servoReference.py');

    const largest = (actual, expected) => Math.max(...actual.map((value, index) => Math.abs(value - expected[index])));
    for (const { name } of joints) {
        const key = name.replace(/_(.)/g, (match, letter) => letter.toUpperCase()).replace(/^./, (letter) => letter.toLowerCase());
        const position = largest(run.series(`SO-ARM100.${key}Position`), reference[name].position);
        assert.ok(position < 1e-9, `${name}: Konjugate and MuJoCo alone differ by up to ${position} rad.`);
        if (!(name in setpoints)) continue;
        // The servo torque recorded at 0 is the state's initial value; from the first step on, the servo's.
        const torques = run.series(`SO-ARM100.${key}ServoTorque`);
        const torque = largest(torques.slice(1), reference[name].servoTorque.slice(1));
        assert.ok(torque < 1e-9, `${name}: servo torques differ by up to ${torque} N·m.`);
        // Will these servos do it? Each stays within its limit, and has brought its joint most of the way.
        assert.ok(Math.max(...torques.map(Math.abs)) <= servoLimit + 1e-9, `${name}: its servo went past ${servoLimit} N·m.`);
        const final = run.series(`SO-ARM100.${key}Position`).at(-1);
        assert.ok(Math.abs(final - setpoints[name]) < 0.5 * Math.abs(setpoints[name]), `${name} ended at ${final} rad, far from ${setpoints[name]}.`);
    }
    // Each body where MuJoCo alone has it, from the start (its reference pose, read from the model) to the end.
    const bodyKey = (name) => name.replace(/_(.)/g, (match, letter) => letter.toUpperCase()).replace(/^./, (letter) => letter.toLowerCase());
    for (const name of description.bodies) {
        const series = ['X', 'Y', 'Z', 'Qw', 'Qx', 'Qy', 'Qz'].map((part) => run.series(`SO-ARM100.${bodyKey(name)}${part}`));
        const largestPose = Math.max(...run.times.map((time, sample) => Math.max(...series.map((values, part) => Math.abs(values[sample] - reference.bodies[name][sample][part])))));
        assert.ok(largestPose < 1e-9, `${name}: its pose differs from MuJoCo alone's by up to ${largestPose}.`);
    }
});
