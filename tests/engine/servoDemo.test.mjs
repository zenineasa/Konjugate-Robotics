/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Will these servos do it? The SO-ARM100 built as the window builds it, with its pick-and-place move, its servos held
// to the Feetech STS3215's datasheet (current for torque, and its 1.91 N·m at stall in place of the model's 3.5) and a
// battery and a winding for each servo. Checked against MuJoCo alone with the same move (positions and servo torques),
// against the datasheet's current for that torque, and against the same heat and charge equations integrated beside it;
// then the same move with a payload in the gripper, which the servos meet as they should: harder, longer at their limit,
// hotter, and the battery lower. Skipped, saying why, when the SO-ARM100 is not fetched to this computer.

import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdtemp, readdir, readFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import importRobot from '../../packages/toolbox/importers/robot.mjs';
import { storedName } from '../../packages/toolbox/lib/catalog.mjs';
import { jointSchedule, moveFor } from '../../packages/toolbox/lib/moves.mjs';
import { servoById } from '../../packages/toolbox/lib/servos.mjs';
import { konjugateModule, roboticsRoot } from '../../scripts/konjugatePaths.mjs';
import { helpers, providerTemplate, runDocument, runReference } from './harness.mjs';

const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';
const folder = 'trs_so_arm100';
const modelDirectory = join(roboticsRoot, 'out', 'models', 'menagerie', commit, folder);
const skip = !existsSync(join(modelDirectory, 'scene.xml')) && `the SO-ARM100 is not fetched (npm run fetch:model ${folder} -- --commit ${commit})`;

const servo = servoById('sts3215');
const move = moveFor(folder, 'pickAndPlace');
const jointNames = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw'];
const key = (name) => name.replace(/_(.)/g, (match, letter) => letter.toUpperCase()).replace(/^./, (letter) => letter.toLowerCase());
const step = 0.01;

// The helpers the host gives an importer, reading the package's files as the build packages them.
const importerHelpers = {
    ...helpers,
    async readPackageJson(path) {
        if (path === 'engine/mechanismProvider.json') return { source: providerTemplate };
        const component = path.match(/^engine\/components\/(\w+)\.json$/)?.[1];
        if (component === 'battery') return JSON.parse(await readFile(konjugateModule(join('assets', 'componentLibrary', 'battery.json')), 'utf8'));
        if (component) return JSON.parse(await readFile(join(roboticsRoot, 'packages', 'engine', 'components', `${component}.json`), 'utf8'));
        throw new Error(`The importer asked for ${path}.`);
    }
};

async function storedFiles() {
    const files = [];
    for (const entry of await readdir(modelDirectory, { recursive: true, withFileTypes: true })) {
        if (!entry.isFile() || entry.name === 'provenance.json') continue;
        const path = join(entry.parentPath, entry.name).slice(modelDirectory.length + 1).split('\\').join('/');
        files.push({ role: 'model', name: storedName(path), data: new Uint8Array(await readFile(join(modelDirectory, path))) });
    }
    return files;
}

async function build(payload = null) {
    const built = await importRobot({
        files: await storedFiles(), helpers: importerHelpers,
        options: {
            step: 'build', folder, main: 'scene.xml', name: 'SO-ARM100', move: 'pickAndPlace', servo: 'sts3215', supply: { voltage: 7.4, capacity: 7200 },
            joints: jointNames.map((name) => ({ name, mode: 'servo' })), timeStep: step, ...(payload ? { payload } : {})
        }
    });
    assert.ok(built.ok, JSON.stringify(built.report));
    return built;
}

const directory = await mkdtemp(join(tmpdir(), 'robotics-servo-demo-'));
const largest = (values) => Math.max(...values.map(Math.abs));

test('the pick-and-place move, on servos held to their datasheet, runs as MuJoCo alone runs it, and draws and heats as its datasheet says', { skip }, async () => {
    const built = await build();
    assert.ok(built.report.notes.some((note) => /1\.91 N·m at stall, from its datasheet/.test(note)), 'The report says the servos are held to their datasheet.');
    const run = await runDocument(directory, 'move', built.document);
    assert.equal(run.times.at(-1), 9);

    // Positions and servo torques as MuJoCo alone has them, with the same move and the datasheet's torque limit.
    const schedules = Object.fromEntries(jointNames.map((name) => [name, jointSchedule(move, name).schedule.samples]));
    const reference = await runReference(directory, {
        modelPath: join(modelDirectory, 'scene.xml'), step, times: run.times, schedules,
        joints: jointNames.map((name) => ({ name, initialPosition: jointSchedule(move, name).start })),
        torqueLimits: Object.fromEntries(jointNames.map((name) => [name, servo.stallTorque]))
    }, 'servoReference.py');
    for (const name of jointNames) {
        const positions = run.series(`SO-ARM100.${key(name)}Position`);
        const torques = run.series(`SO-ARM100.${key(name)}ServoTorque`);
        const position = Math.max(...positions.map((value, index) => Math.abs(value - reference[name].position[index])));
        const torque = Math.max(...torques.slice(1).map((value, index) => Math.abs(value - reference[name].servoTorque[index + 1])));
        assert.ok(position < 1e-9, `${name}: positions differ from MuJoCo alone's by up to ${position} rad.`);
        assert.ok(torque < 1e-9, `${name}: servo torques differ from MuJoCo alone's by up to ${torque} N·m.`);
        assert.ok(largest(torques) <= servo.stallTorque + 1e-9, `${name}: its servo went past its datasheet's ${servo.stallTorque} N·m.`);

        // The current the datasheet gives for that torque.
        const currents = run.series(`SO-ARM100.${key(name)}ServoCurrent`);
        const current = Math.max(...currents.slice(1).map((value, index) => Math.abs(value - (servo.idleCurrent + Math.abs(torques[index + 1]) / servo.torqueConstant))));
        assert.ok(current < 1e-12, `${name}: its current is not its datasheet's for its torque (up to ${current} A off).`);

        // The winding's heat, integrated beside the run from the same current (the engine's explicit Euler step).
        const temperatures = run.series(`${name} winding.windingTemperature`);
        let temperature = temperatures[0];
        let worst = 0;
        for (let index = 1; index < temperatures.length; index += 1) {
            temperature += step * (servo.windingResistance * currents[index - 1] ** 2 / 10 - (temperature - 293.15) / (15 * 10));
            worst = Math.max(worst, Math.abs(temperature - temperatures[index]));
        }
        assert.ok(worst < 1e-9, `${name}: its winding's temperature is off by up to ${worst} K.`);
    }
    // The battery, drawn on by all six.
    const charge = run.series('Battery.stateOfCharge');
    const currents = jointNames.map((name) => run.series(`SO-ARM100.${key(name)}ServoCurrent`));
    let expected = charge[0];
    let worst = 0;
    for (let index = 1; index < charge.length; index += 1) {
        expected -= step * currents.reduce((sum, series) => sum + 100 * series[index - 1] / 7200, 0);
        worst = Math.max(worst, Math.abs(expected - charge[index]));
    }
    assert.ok(worst < 1e-9, `The battery's charge is off by up to ${worst} %.`);
});

test('with a payload in the gripper, the servos meet the move harder: longer at their limit, hotter, and the battery lower', { skip }, async () => {
    const atLimit = (run, name) => run.series(`SO-ARM100.${key(name)}ServoTorque`).filter((value) => Math.abs(value) > servo.stallTorque - 1e-6).length * step;
    const summary = (run) => Object.fromEntries(jointNames.map((name) => [name, {
        peakTorque: largest(run.series(`SO-ARM100.${key(name)}ServoTorque`)),
        secondsAtLimit: atLimit(run, name),
        peakWinding: Math.max(...run.series(`${name} winding.windingTemperature`)) - 273.15
    }]));
    const empty = await runDocument(directory, 'empty', (await build()).document);
    const loaded = await runDocument(directory, 'loaded', (await build({ body: 'Fixed_Jaw', mass: 0.3 })).document);
    const [before, after] = [summary(empty), summary(loaded)];
    const used = (run) => 100 - run.series('Battery.stateOfCharge').at(-1);
    console.log(JSON.stringify({ empty: before, loaded: after, batteryUsed: { empty: used(empty), loaded: used(loaded) } }, null, 1));
    // The shoulder carries the arm and the payload: it is at its limit for longer, and its winding gets hotter.
    assert.ok(after.Pitch.secondsAtLimit > before.Pitch.secondsAtLimit, 'The shoulder should spend longer at its limit with the payload.');
    assert.ok(after.Pitch.peakWinding > before.Pitch.peakWinding, 'The shoulder\'s winding should get hotter with the payload.');
    assert.ok(used(loaded) > used(empty), 'The battery should give more with the payload.');
});
