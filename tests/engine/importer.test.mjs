/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The robot importer as the window drives it, without the app: a robot's files handed over as Konjugate's store hands
// them to an importer (binary files of the `model` role, each named for its path with / as ^), with the helpers the
// host gives an importer, then the project it builds run in Konjugate's engine. The SO-ARM100 on its servos holds its
// pose; the Go2, a floating base on motors left free, falls to the floor and stays on it. Skipped, saying why, for a
// robot not fetched to this computer (npm run fetch:model <folder>).

import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdtemp, readdir, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import importRobot from '../../packages/toolbox/importers/robot.mjs';
import { robotByFolder, storedName } from '../../packages/toolbox/lib/catalog.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { helpers, providerTemplate, runDocument } from './harness.mjs';

const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';
const directory = await mkdtemp(join(tmpdir(), 'robotics-importer-'));

// The helpers the host gives an importer: Konjugate's equation helpers and a reader of its own package's JSON files,
// here the provider's source as the build packages it.
const importerHelpers = {
    ...helpers,
    async readPackageJson(path) {
        assert.equal(path, 'engine/mechanismProvider.json');
        return { source: providerTemplate };
    }
};

// A fetched robot's files as the store hands them to the importer: { role, name, data }.
async function storedFiles(folder) {
    const root = join(roboticsRoot, 'out', 'models', 'menagerie', commit, folder);
    const files = [];
    for (const entry of await readdir(root, { recursive: true, withFileTypes: true })) {
        if (!entry.isFile() || entry.name === 'provenance.json') continue;
        const path = join(entry.parentPath, entry.name).slice(root.length + 1).split('\\').join('/');
        files.push({ role: 'model', name: storedName(path), data: new Uint8Array(await readFile(join(root, path))) });
    }
    return files;
}

const fetched = (folder) => existsSync(join(roboticsRoot, 'out', 'models', 'menagerie', commit, folder, 'scene.xml'));
const skipUnless = (folder) => !fetched(folder) && `${folder} not fetched (npm run fetch:model ${folder} -- --commit ${commit})`;

test('the SO-ARM100: described with its servos and licence, built with its servos holding it, and it holds', { skip: skipUnless('trs_so_arm100') }, async () => {
    const files = await storedFiles('trs_so_arm100');
    const described = await importRobot({ files, helpers: importerHelpers, options: { step: 'describe', main: 'scene.xml' } });
    assert.ok(described.ok, JSON.stringify(described.report));
    assert.equal(described.data.description.actuators.filter(({ kind }) => kind === 'position').length, 6);
    assert.match(described.data.licence, /Apache License/);

    const joints = described.data.description.joints.map(({ name }) => ({ name, mode: 'servo', value: 0 }));
    const provenance = { source: 'MuJoCo Menagerie', folder: 'trs_so_arm100', commit };
    const built = await importRobot({ files, helpers: importerHelpers, options: { step: 'build', main: 'scene.xml', name: robotByFolder('trs_so_arm100').name, joints, provenance, runTime: 1 } });
    assert.ok(built.ok, JSON.stringify(built.report));
    assert.equal(built.data.driven, 6);
    assert.ok(built.document.nodes[0].implementation.source.includes(commit), 'The project carries where the robot came from.');

    const run = await runDocument(directory, 'so100', built.document);
    // Held at its starting pose by its servos, against gravity: each joint within a few hundredths of a radian.
    for (const key of ['rotation', 'pitch', 'elbow', 'wristPitch', 'wristRoll', 'jaw']) {
        const drift = Math.max(...run.series(`SO-ARM100.${key}Position`).map(Math.abs));
        assert.ok(drift < 0.05, `${key} drifted ${drift} rad from where its servo holds it.`);
    }
});

test('the Go2: a floating base on motors, left free, falls to the floor and the run carries on with it there', { skip: skipUnless('unitree_go2') }, async () => {
    const files = await storedFiles('unitree_go2');
    const described = await importRobot({ files, helpers: importerHelpers, options: { step: 'describe', main: 'scene.xml' } });
    assert.ok(described.ok, JSON.stringify(described.report));
    assert.equal(described.data.description.joints.filter(({ kind }) => kind === 'free').length, 1);
    const built = await importRobot({ files, helpers: importerHelpers, options: { step: 'build', main: 'scene.xml', name: 'Go2', joints: [], runTime: 1 } });
    assert.ok(built.ok, JSON.stringify(built.report));
    assert.equal(built.data.joints, 12, 'Twelve leg joints; the floating base is MuJoCo\'s alone.');
    assert.equal(built.data.driven, 0);
    const run = await runDocument(directory, 'go2', built.document);
    assert.equal(run.times.at(-1), 1);
});

test('a step the importer does not have, or no robot, is refused with the reason', async () => {
    assert.match((await importRobot({ files: [], helpers: importerHelpers, options: { step: 'describe' } })).report.errors[0], /Choose a robot/);
    assert.match((await importRobot({ files: [{ role: 'model', name: 'scene.xml', data: new TextEncoder().encode('<mujoco/>') }], helpers: importerHelpers, options: { step: 'walk' } })).report.errors[0], /no step "walk"/);
});
