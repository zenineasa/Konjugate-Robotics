/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The toolbox's what-if forks, run as Konjugate's launcher host runs them: the importer's parameter index and the
// manifest's scenarios resolved by the host's own resolveInterventions, then the baseline forked at a time by the
// host's own runScenarioBranches, on the engine. A fork from a time must carry on exactly as the robot with that change
// made in the drawing, built again and resumed from the baseline's checkpoint at that time: the change reaches the
// motion through the live shared parameter, and nothing else differs.

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';
import { batteryOf, massOf, updateBattery, updateJoint, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { buildModel, inverseWeights } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { envelope, sizingSignals, summarize, whatIf } from '../../packages/toolbox/lib/sizing.mjs';
import { konjugateDir, konjugateModule, roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { runDocument } from './harness.mjs';

const { composeBranchSamples, extractSeries, resolveInterventions, runScenarioBranches } = await import(pathToFileURL(konjugateModule('src/launcherHost.mjs')));
const { startEngineRun } = await import(pathToFileURL(konjugateModule('src/engineAdapter.mjs')));
const engineOptions = { applicationPath: konjugateDir, resourcesPath: '', packaged: false };
const { contributes: { scenarios } } = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'toolbox', 'addon.json'), 'utf8'));
const directory = await mkdtemp(join(tmpdir(), 'robotics-forks-'));

const runTime = 3;
const forkAt = 1;
const drawing = exampleDrawing('pickAndPlace');
const built = buildModel(drawing, { timeStep: 0.001, runTime });
const content = JSON.stringify(built.document);
const config = { name: 'Baseline', targetTime: runTime, globalTimeStep: 0.001, outputInterval: built.document.runConfigurations[0].outputInterval };
const baselineRun = await startEngineRun(content, config, engineOptions, { retainResult: true });
const baseline = { result: await baselineRun.completion, resultPath: baselineRun.resultPath, cleanup: baselineRun.cleanup };
test.after(() => baseline.cleanup?.());

// A value held from the fork to the end of the run, as the window supplies it.
const held = (value) => [[0, value], [runTime - forkAt, value]];

// Forks the baseline with `scenarioId`, the window's `supplied` data, and checks it against `changed` (the drawing with
// the same change made), built and resumed from the fork's checkpoint. Returns the fork's and the baseline's samples.
// The changed robot keeps the joints' inverse inertia where drawn as the robot was built (lib/modelBuilder.mjs): a
// constant of the model, which a fork leaves as it was.
async function forkMatches(scenarioId, supplied, changed) {
    const scenario = { ...scenarios.find((candidate) => candidate.scenarioId === scenarioId), forkAt, runTime };
    const interventions = resolveInterventions(scenario, built.parameterIndex, null, supplied);
    const { forkTime, child } = await runScenarioBranches({ content, config, scenario, interventions, baseline, engineOptions });
    try {
        assert.equal(forkTime, forkAt);
        const checkpoint = baseline.result.checkpoints.find((candidate) => Math.abs(candidate.time - forkAt) < 1e-9);
        const expected = await runDocument(directory, scenarioId, buildModel(changed, { timeStep: 0.001, runTime, inverseWeights: inverseWeights(drawing) }).document, { startCheckpoint: checkpoint });
        assert.equal(child.result.samples.length, expected.result.samples.length);
        let largest = 0;
        child.result.samples.forEach((sample, index) => {
            const wanted = new Map(expected.result.samples[index].states.map(({ stateId, value }) => [stateId, value]));
            assert.ok(Math.abs(sample.time - expected.result.samples[index].time) < 1e-9);
            for (const { stateId, value } of sample.states) {
                if (!wanted.has(stateId)) continue;
                largest = Math.max(largest, Math.abs(value - wanted.get(stateId)) / Math.max(1, Math.abs(value)));
            }
        });
        assert.ok(largest < 1e-9, `${scenarioId}: the fork differs from the changed robot resumed at ${forkAt} s by ${largest}.`);
        return child.result.samples;
    } finally {
        await child.cleanup?.();
    }
}

const stateId = (key) => {
    const [nodeName, symbol] = key.split('.');
    return built.document.nodes.find(({ name }) => name === nodeName).states.find((state) => state.symbol === symbol).id;
};
const finalValue = (samples, key) => samples.at(-1).states.find((state) => state.stateId === stateId(key)).value;

test('a heavier part, from a time in the cycle, moves the arm as the robot drawn with it would, and loads the shoulder more', async () => {
    const part = drawing.solids.find(({ name }) => name === 'Part');
    const mass = 4 * massOf(part);
    const samples = await forkMatches('heavierSolid', { byParameter: { solidMass: { entities: ['Part'], samples: { Part: held(mass) } } } },
        updateSolid(drawing, part.id, { mass: { by: 'mass', value: mass } }));
    assert.ok(Math.abs(finalValue(samples, 'Shoulder motor.current')) > Math.abs(finalValue(baseline.result.samples, 'Shoulder motor.current')),
        'Holding a heavier part, the shoulder\'s motor should draw more current.');
});

test('a weaker motor, from a time in the cycle, runs as the robot drawn with it would', async () => {
    const shoulder = drawing.joints.find(({ name }) => name === 'Shoulder');
    const torqueConstant = 0.6 * shoulder.drive.motor.torqueConstant;
    await forkMatches('weakerMotor', { byParameter: { torqueConstant: { entities: ['Shoulder'], samples: { Shoulder: held(torqueConstant) } } } },
        updateJoint(drawing, shoulder.id, { drive: { motor: { torqueConstant } } }));
});

test('a tired battery, from a time in the cycle, gives every motor fewer volts, as the robot drawn with it would', async () => {
    const { fullVoltage, emptyVoltage } = batteryOf(drawing);
    const drop = 1.5;
    const samples = await forkMatches('tiredBattery', {
        byParameter: {
            batteryFullVoltage: { entities: ['Battery'], samples: { Battery: held(fullVoltage - drop) } },
            batteryEmptyVoltage: { entities: ['Battery'], samples: { Battery: held(emptyVoltage - drop) } }
        }
    }, updateBattery(drawing, { fullVoltage: fullVoltage - drop, emptyVoltage: emptyVoltage - drop }));
    assert.ok(Math.abs(finalValue(baseline.result.samples, 'Battery.voltage') - finalValue(samples, 'Battery.voltage') - drop) < 1e-3,
        'The battery should give the drop fewer volts.');
});

// The sizing view's sums, on runs the engine made, as the window reads them: the host's own extractSeries thins them.
test('sized from the engine\'s run, every motor works inside its band, and a much weaker shoulder spends more of the cycle at full duty', async () => {
    const asDrawn = summarize({ series: extractSeries(baseline.result.samples, built.document, [...sizingSignals]) }, drawing);
    // Inside the band at the battery's voltage at that moment: the duty is held to ±1.
    const voltages = new Map(baseline.result.samples.map((sample) => [sample.time.toFixed(9), sample.states.find((state) => state.stateId === stateId('Battery.voltage')).value]));
    for (const motor of asDrawn.motors) {
        assert.ok(motor.points.length > 100, `${motor.name}: its run should be read back`);
        for (const [speed, torque, time] of motor.points) {
            const band = envelope(motor.motor, voltages.get(time.toFixed(9)));
            assert.ok(torque <= band.most(speed) * (1 + 1e-9) + 1e-12 && torque >= band.least(speed) * (1 + 1e-9) - 1e-12, `${motor.name} at ${time} s: ${torque} outside its band.`);
        }
    }
    assert.ok(asDrawn.battery.chargeUsed > 0);
    // The shoulder at a twentieth of its torque per ampere from the fork, run as the window asks for it.
    const asked = whatIf(drawing, { scenarioId: 'weakerMotor', forkAt, runTime, motor: 'Shoulder', share: 0.05 });
    const scenario = { ...scenarios.find((candidate) => candidate.scenarioId === 'weakerMotor'), forkAt, runTime };
    const interventions = resolveInterventions(scenario, built.parameterIndex, null, asked.options.supplied);
    const { forkTime, child } = await runScenarioBranches({ content, config, scenario, interventions, baseline, engineOptions });
    try {
        const weaker = summarize({ series: extractSeries(composeBranchSamples(baseline.result.samples, child.result.samples, forkTime), built.document, [...sizingSignals]) }, asked.changed);
        const shoulder = (summary) => summary.motors.find(({ name }) => name === 'Shoulder');
        assert.ok(shoulder(weaker).atFullDuty > shoulder(asDrawn).atFullDuty, `A weaker shoulder should spend more of the cycle at full duty (${shoulder(weaker).atFullDuty} against ${shoulder(asDrawn).atFullDuty}).`);
        assert.ok(shoulder(weaker).stallTorque < 0.051 * shoulder(asDrawn).stallTorque, 'and its band is a twentieth as tall');
    } finally {
        await child.cleanup?.();
    }
});
