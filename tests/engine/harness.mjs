/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Running a built model through the real engine CLI. The engine is Konjugate's own build (out/engine/konjugateEngine in
// the checkout, or KONJUGATE_ENGINE_PATH). Nothing else runs the robot: no provider, no interpreter.
//
// MuJoCo, run on its own as a second opinion, needs a Python with MuJoCo: this repository's .venv (npm run
// setup:python) or ROBOTICS_PYTHON. Without one, the tests that compare with it are skipped and say so.

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

export const executable = process.env.KONJUGATE_ENGINE_PATH
    ?? konjugateModule(join('out', 'engine', process.platform === 'win32' ? 'konjugateEngine.exe' : 'konjugateEngine'));
export const python = process.env.ROBOTICS_PYTHON
    ?? join(roboticsRoot, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');

if (!existsSync(executable)) throw new Error(`No Konjugate engine at ${executable}; build Konjugate or set KONJUGATE_ENGINE_PATH.`);

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

// Why MuJoCo cannot be run here, or null when it can.
export async function mujocoMissing() {
    if (!existsSync(python)) return `no Python at ${python} (npm run setup:python, or set ROBOTICS_PYTHON)`;
    const { code } = await execute(python, ['-I', '-c', 'import mujoco'], { captureErrors: true }).catch(() => ({ code: 1 }));
    return code === 0 ? null : `the Python at ${python} has no MuJoCo`;
}

// The canvas binds a node's every state to each of its source terms, as its node editor does, ahead of the term's own
// parameters: a parameter of a state's name would lose its name to the state, and the term would read the state. A
// document built headless must not depend on the canvas never seeing it.
export function checkCanvasBindings(document) {
    for (const node of document.nodes) {
        const states = new Set(node.states.map(({ symbol }) => symbol));
        for (const term of node.sourceTerms ?? []) {
            for (const parameter of term.parameters ?? []) {
                assert.ok(!states.has(parameter.symbol), `${node.name}: the parameter ${parameter.symbol} of the source term on ${term.state} has a state's name, which the canvas would bind instead.`);
            }
        }
    }
}

// Validates and runs `document` for its run configuration's target time (or `targetTime`), in `directory`, from the
// start or from `startCheckpoint` (one of a result's checkpoints); returns { times, series(key) } of "Node.state"
// values at every output, and the raw result.
export async function runDocument(directory, name, document, { targetTime, startCheckpoint } = {}) {
    checkCanvasBindings(document);
    const inputPath = join(directory, `${name}.kjt`);
    const reportPath = join(directory, `${name}.report`);
    const outputPath = join(directory, `${name}.kjr`);
    const configurationPath = join(directory, `${name}.json`);
    const run = document.runConfigurations[0];
    await writeFile(inputPath, await encodeProjectFile(JSON.stringify(document)));
    await writeFile(configurationPath, JSON.stringify({
        name, targetTime: targetTime ?? run.targetTime, globalTimeStep: run.globalTimeStep, outputInterval: run.outputInterval,
        ...(startCheckpoint ? { startCheckpoint } : {})
    }));
    assert.equal((await execute(executable, ['validate', inputPath, '--report', reportPath])).code, 0, `${name}: validation failed to run.`);
    const report = decodeValidationReport(await readFile(reportPath));
    assert.equal(report.valid, true, `${name}: ${JSON.stringify(report.issues ?? report)}`);
    const { code, errors } = await execute(executable, ['run', inputPath, '--configuration', configurationPath, '--output', outputPath], { captureErrors: true });
    assert.equal(code, 0, `${name}: run failed. ${errors}`);
    const result = decodeResultFile(await readFile(outputPath));
    const stateIds = new Map(document.nodes.flatMap((node) => node.states.map((state) => [`${node.name}.${state.symbol}`, state.id])));
    const series = (key) => {
        if (!stateIds.has(key)) throw new Error(`${name}: no state ${key}.`);
        const stateId = stateIds.get(key);
        return result.samples.map((sample) => sample.states.find((state) => state.stateId === stateId).value);
    };
    return { name, document, result, times: result.samples.map((sample) => sample.time), series };
}

// MuJoCo on its own, for comparison: reference.py runs an MJCF with the given joints, controls and output times, and
// returns each joint's positions and the named bodies' places.
export async function runReference(directory, request) {
    const requestPath = join(directory, `reference-${Math.random().toString(36).slice(2)}.json`);
    await writeFile(requestPath, JSON.stringify(request));
    const { code, output } = await execute(python, ['-I', join(roboticsRoot, 'tests', 'engine', 'reference.py'), requestPath], { capture: true });
    assert.equal(code, 0, 'The MuJoCo reference run failed.');
    return JSON.parse(output);
}
