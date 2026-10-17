/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Running a built model through the real engine CLI, with the Python that has MuJoCo as the providers' interpreter.
// The engine is Konjugate's own build (out/engine/konjugateEngine in the checkout, or the first argument); the
// interpreter is this repository's .venv (or ROBOTICS_PYTHON).

import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { readFile, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { konjugateModule, roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const { encodeProjectFile } = await import(pathToFileURL(konjugateModule('src/projectFile.mjs')));
const { decodeResultFile } = await import(pathToFileURL(konjugateModule('src/engineProtocol.mjs')));
const { decodeValidationReport } = await import(pathToFileURL(konjugateModule('src/reportProtocol.mjs')));
export const helpers = await import(pathToFileURL(konjugateModule('src/equationModel.mjs')));

export const executable = process.env.KONJUGATE_ENGINE_PATH
    ?? konjugateModule(join('out', 'engine', process.platform === 'win32' ? 'konjugateEngine.exe' : 'konjugateEngine'));
export const python = process.env.ROBOTICS_PYTHON
    ?? join(roboticsRoot, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
export const providerTemplate = await readFile(join(roboticsRoot, 'packages', 'engine', 'providers', 'mechanism.py'), 'utf8');

// This toolbox's component templates, and core's that it places too, by id.
export async function loadTemplates(coreIds = []) {
    const plugin = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'engine', 'plugin.json'), 'utf8'));
    const templates = new Map();
    for (const { entry } of plugin.contributes) {
        const template = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'engine', entry), 'utf8'));
        templates.set(template.id, template);
    }
    for (const id of coreIds) templates.set(id, JSON.parse(await readFile(konjugateModule(join('assets', 'componentLibrary', `${id}.json`)), 'utf8')));
    return templates;
}

if (!existsSync(executable)) throw new Error(`No Konjugate engine at ${executable}; build Konjugate or set KONJUGATE_ENGINE_PATH.`);
if (!existsSync(python)) throw new Error(`No Python at ${python}; run "uv venv .venv --python 3.12 && uv pip install --python .venv/bin/python mujoco", or set ROBOTICS_PYTHON.`);

// `capture` keeps standard output, `captureErrors` standard error (otherwise passed through).
export function execute(command, args, { capture = false, captureErrors = false } = {}) {
    return new Promise((resolve, reject) => {
        const child = spawn(command, args, { stdio: ['ignore', capture ? 'pipe' : 'ignore', captureErrors ? 'pipe' : 'inherit'] });
        let output = '';
        let errors = '';
        child.stdout?.on('data', (chunk) => { output += chunk; });
        child.stderr?.on('data', (chunk) => { errors += chunk; });
        child.once('error', reject);
        child.once('close', (code) => resolve({ code, output, errors }));
    });
}

// Validates and runs `document` for its run configuration's target time (or `targetTime`), in `directory`, from the
// start or from `startCheckpoint` (one of a result's checkpoints); returns { times, series(key) } of "Node.state"
// values at every output, and the raw result. With `expectFailure`, the run must fail; returns { errors }, what the
// engine wrote to standard error.
export async function runDocument(directory, name, document, { targetTime, startCheckpoint, expectFailure = false } = {}) {
    const inputPath = join(directory, `${name}.kjt`);
    const reportPath = join(directory, `${name}.report`);
    const outputPath = join(directory, `${name}.kjr`);
    const configurationPath = join(directory, `${name}.json`);
    const run = document.runConfigurations[0];
    await writeFile(inputPath, await encodeProjectFile(JSON.stringify(document)));
    await writeFile(configurationPath, JSON.stringify({
        name, targetTime: targetTime ?? run.targetTime, globalTimeStep: run.globalTimeStep, outputInterval: run.outputInterval,
        providers: { python: { interpreter: python, sdkPath: konjugateModule(join('engine', 'sdk', 'python')) } },
        ...(startCheckpoint ? { startCheckpoint } : {})
    }));
    assert.equal((await execute(executable, ['validate', inputPath, '--report', reportPath])).code, 0, `${name}: validation failed to run.`);
    const report = decodeValidationReport(await readFile(reportPath));
    assert.equal(report.valid, true, `${name}: ${JSON.stringify(report.issues ?? report)}`);
    const args = ['run', inputPath, '--configuration', configurationPath, '--output', outputPath];
    if (expectFailure) {
        const { code, errors } = await execute(executable, args, { captureErrors: true });
        assert.notEqual(code, 0, `${name}: the run should have failed.`);
        return { errors };
    }
    assert.equal((await execute(executable, args)).code, 0, `${name}: run failed.`);
    const result = decodeResultFile(await readFile(outputPath));
    const stateIds = new Map(document.nodes.flatMap((node) => node.states.map((state) => [`${node.name}.${state.symbol}`, state.id])));
    const series = (key) => {
        if (!stateIds.has(key)) throw new Error(`${name}: no state ${key}.`);
        const stateId = stateIds.get(key);
        return result.samples.map((sample) => sample.states.find((state) => state.stateId === stateId).value);
    };
    return { name, document, result, times: result.samples.map((sample) => sample.time), series };
}

// MuJoCo on its own, for comparison: `script` (in tests/engine) reads `request` and prints its result as JSON.
// reference.py runs the same MJCF with the given joints, constant generalised forces and output times, and returns
// each joint's positions; armReference.py drives the joints with motors (see it).
export async function runReference(directory, request, script = 'reference.py') {
    const requestPath = join(directory, `reference-${Math.random().toString(36).slice(2)}.json`);
    await writeFile(requestPath, JSON.stringify(request));
    const { code, output } = await execute(python, ['-I', join(roboticsRoot, 'tests', 'engine', script), requestPath], { capture: true });
    assert.equal(code, 0, 'The MuJoCo reference run failed.');
    return JSON.parse(output);
}
