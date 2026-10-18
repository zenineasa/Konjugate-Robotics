/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics Toolbox window in the real app, offline: the main process's fetch is replaced so GitHub's API and
// Menagerie's raw files are answered from the SO-ARM100 fetched to this computer (npm run fetch:model trs_so_arm100).
// Through the window:
//   - nothing is downloaded before the licence is shown; the licence is the robot's, Apache-2.0
//   - the robot is downloaded into Konjugate's store of downloaded files, every file of its folder but its picture
//   - used, its six joints are listed, each with its own servo; one is set free, one given a setpoint
//   - built, the mechanism is in the canvas: the project carries the robot with its licence and commit, and its
//     servos' setpoints are live shared parameters, the one set in the window at its value
//   - built again with its pick-and-place move, its servos as their datasheet gives them, a battery and a payload:
//     the canvas has the battery, a winding for each servo, the move as each setpoint's schedule, each servo held to
//     its datasheet's torque, and the payload as a slider
//   - run in the app (with this repository's Python, which has MuJoCo, as the providers' Python) and shown in Pose
//     Visualizer, once a naming convention for poses by quaternion is made: each of its seven links is a body of the
//     one mechanism node, drawn with its own mesh at its true size, where the run put it
// Uses Playwright from the Konjugate checkout, as the logistics toolbox's interaction tests do; Pose Visualizer is
// installed from a sibling checkout's build (../konjugate-posevisualizer, npm run build), as Konjugate's own tests do.
// ROBOTICS_SCREENSHOT=<path> keeps a picture of Pose Visualizer showing the robot.

import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdir, mkdtemp, readdir, readFile, rm, writeFile } from 'node:fs/promises';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { installBuiltPackages } from '../../scripts/installDev.mjs';
import { konjugateDir, konjugateModule, roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const require = createRequire(join(konjugateDir, 'package.json'));
const { _electron: electron } = require('playwright');
const electronPath = require('electron');
const { encodeProjectFile } = await import(pathToFileURL(konjugateModule('src/projectFile.mjs')));
const { installInteractionAddons } = await import(pathToFileURL(konjugateModule('scripts/interactionAddons.mjs')));
const python = process.env.ROBOTICS_PYTHON ?? join(roboticsRoot, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');

const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';
const folder = 'trs_so_arm100';
const modelDirectory = join(roboticsRoot, 'out', 'models', 'menagerie', commit, folder);
if (!existsSync(join(modelDirectory, 'scene.xml'))) {
    console.error(`The SO-ARM100 is not on this computer: npm run fetch:model ${folder} -- --commit ${commit}`);
    process.exit(1);
}

// What GitHub would answer, from the fetched folder: the commit, each directory's listing, each file's bytes (as
// base64, to cross into the main process). Menagerie's picture is among them, as it is upstream.
async function githubAnswers() {
    const files = {};
    const directories = { [folder]: [] };
    for (const entry of await readdir(modelDirectory, { recursive: true, withFileTypes: true })) {
        if (!entry.isFile() || entry.name === 'provenance.json') continue;
        const path = join(entry.parentPath, entry.name).slice(modelDirectory.length + 1).split('\\').join('/');
        files[`${folder}/${path}`] = (await readFile(join(modelDirectory, path))).toString('base64');
    }
    files[`${folder}/so_arm100.png`] = Buffer.from('not a real picture').toString('base64');
    for (const path of Object.keys(files)) {
        const parts = path.split('/');
        for (let depth = 1; depth < parts.length; depth += 1) {
            const parent = parts.slice(0, depth).join('/');
            const child = parts.slice(0, depth + 1).join('/');
            directories[parent] ??= [];
            const isFile = depth === parts.length - 1;
            if (!directories[parent].some((entry) => entry.path === child)) {
                directories[parent].push({ name: parts[depth], path: child, type: isFile ? 'file' : 'dir', size: isFile ? Buffer.from(files[path], 'base64').length : 0 });
            }
        }
    }
    return { commit, files, directories };
}

const env = { ...process.env };
delete env.ELECTRON_RUN_AS_NODE;
const extraArgs = (process.env.KONJUGATE_ELECTRON_ARGS ?? '').split(' ').filter(Boolean);
const scratch = await mkdtemp(join(tmpdir(), 'konjugate-robotics-'));
const userData = join(scratch, 'userData');

try {
    await installBuiltPackages(userData);
    const addons = await installInteractionAddons(konjugateDir, userData);
    assert.ok(addons.some((name) => name.startsWith('konjugate.poseVisualizer')), 'Pose Visualizer is not built: npm run build in ../konjugate-posevisualizer.');
    // The providers' Python: this repository's, which has MuJoCo.
    await mkdir(join(userData, 'providers'), { recursive: true });
    await writeFile(join(userData, 'providers', 'toolchains.json'), JSON.stringify({ version: 1, cpp: { compilerPath: '' }, python: { interpreterPath: python }, executionMode: '' }));
    const projectPath = join(scratch, 'empty.kjt');
    await writeFile(projectPath, await encodeProjectFile(JSON.stringify({
        format: 'konjugate', version: 1, metadata: { units: 'SI' }, nodes: [], edges: [], sharedParameters: [],
        runConfigurations: [{ id: 1, name: 'Default', globalTimeStep: 0.01, outputInterval: 0.01 }], activeRunConfigurationId: 1
    })));
    const app = await electron.launch({ executablePath: electronPath, args: [konjugateDir, ...extraArgs, `--user-data-dir=${userData}`, projectPath], env });
    try {
        await app.evaluate((_electron, answers) => {
            globalThis.roboticsRequests = [];
            globalThis.fetch = async (url) => {
                const address = new URL(url);
                globalThis.roboticsRequests.push(address.href);
                const json = (value) => new Response(JSON.stringify(value), { status: 200, headers: { 'content-type': 'application/json' } });
                if (address.hostname === 'api.github.com' && address.pathname.endsWith('/commits/main')) return json({ sha: answers.commit });
                const contents = address.pathname.match(/\/repos\/google-deepmind\/mujoco_menagerie\/contents\/(.+)$/);
                if (address.hostname === 'api.github.com' && contents && address.searchParams.get('ref') === answers.commit) {
                    const listing = answers.directories[decodeURIComponent(contents[1])];
                    return listing ? json(listing) : new Response('not found', { status: 404 });
                }
                const raw = address.pathname.match(/^\/google-deepmind\/mujoco_menagerie\/([0-9a-f]{40})\/(.+)$/);
                if (address.hostname === 'raw.githubusercontent.com' && raw && raw[1] === answers.commit) {
                    const file = answers.files[decodeURIComponent(raw[2])];
                    if (file) {
                        const bytes = Buffer.from(file, 'base64');
                        return new Response(bytes, { status: 200, headers: { 'content-length': String(bytes.length) } });
                    }
                }
                return new Response('not found', { status: 404 });
            };
        }, await githubAnswers());

        const project = await app.firstWindow();
        await project.waitForLoadState('domcontentloaded');
        const tool = '.addonTool[data-addon-id="konjugate.robotics.toolbox"][data-command-id="openRoboticsToolbox"]';
        await project.waitForSelector(tool, { timeout: 30000 });
        await project.click(tool);
        let toolbox = null;
        for (let attempt = 0; attempt < 150 && !toolbox; attempt += 1) {
            toolbox = app.windows().find((candidate) => candidate.url().includes('konjugate.robotics.toolbox'));
            if (!toolbox) await new Promise((resolve) => setTimeout(resolve, 100));
        }
        assert.ok(toolbox, 'The Robotics toolbox window did not open.');
        const log = [];
        toolbox.on('console', (message) => log.push(`${message.type()}: ${message.text().slice(0, 300)}`));
        toolbox.on('pageerror', (error) => log.push(`pageerror: ${error.message}`));
        const fail = async (error) => {
            const shown = await toolbox.evaluate(() => ({ robot: document.querySelector('#robotStatus')?.innerText, build: document.querySelector('#buildStatus')?.innerText })).catch((problem) => problem.message);
            throw new Error(`${error.message}\nThe window shows: ${JSON.stringify(shown)}\nToolbox window log:\n${log.join('\n')}`);
        };
        await toolbox.waitForLoadState('domcontentloaded');
        const robot = toolbox.locator('.robot', { hasText: 'SO-ARM100' });

        // 1. The licence first, nothing downloaded; then the download.
        await robot.getByRole('button', { name: 'Licence' }).click();
        await robot.locator('.licence').waitFor({ timeout: 20000 }).catch(fail);
        assert.match(await robot.locator('.licence p').textContent(), /Licence: Apache-2\.0\..*folder trs_so_arm100, at commit 62ab00f/);
        assert.ok(!(await app.evaluate(() => globalThis.roboticsRequests)).some((url) => url.includes('/assets/')), 'Nothing of the robot is downloaded before its licence is shown.');
        await robot.getByRole('button', { name: 'Download' }).click();
        await toolbox.waitForFunction(() => /is kept on this computer/.test(document.querySelector('#robotStatus')?.textContent ?? ''), null, { timeout: 60000 }).catch(fail);
        assert.match(await robot.locator('p.small').textContent(), /On this computer: Menagerie at 62ab00f, 23 files/);
        assert.ok(!(await app.evaluate(() => globalThis.roboticsRequests)).some((url) => url.endsWith('.png')), 'Menagerie\'s picture is not downloaded.');

        // 2. Used: its joints, each with its own servo; one set free, one given a setpoint.
        await robot.getByRole('button', { name: 'Use' }).click();
        await toolbox.waitForSelector('#stepJoints:not([hidden])', { timeout: 30000 }).catch(fail);
        assert.equal(await toolbox.locator('#joints tr').count(), 6);
        assert.match(await toolbox.locator('#jointsIntro').textContent(), /6 joints; 6 with a servo/);
        await toolbox.selectOption('select[aria-label="Mode of Wrist_Roll"]', 'free');
        await toolbox.fill('input[aria-label="Setpoint or torque of Elbow"]', '1.2');
        await toolbox.dispatchEvent('input[aria-label="Setpoint or torque of Elbow"]', 'change');

        // 3. Built into the canvas.
        await toolbox.click('#buildButton');
        await toolbox.waitForFunction(() => /now in the canvas/.test(document.querySelector('#buildStatus')?.textContent ?? ''), null, { timeout: 60000 }).catch(fail);
        assert.match(await toolbox.locator('#buildStatus').textContent(), /SO-ARM100: 6 joints, 5 driven/);
        let saved = null;
        for (let attempt = 0; attempt < 100; attempt += 1) {
            saved = await project.evaluate(() => window.__debugProject?.serialize());
            if (saved?.nodes?.length) break;
            await new Promise((resolve) => setTimeout(resolve, 100));
        }
        if (process.env.ROBOTICS_SAVED) await writeFile(process.env.ROBOTICS_SAVED, JSON.stringify(saved));
        const mechanism = saved.nodes.find((node) => node.name === 'SO-ARM100');
        assert.ok(mechanism?.implementation, 'The canvas has the mechanism, with its provider.');
        const source = mechanism.implementation.source;
        assert.ok(source.includes(commit) && source.includes('Apache License'), 'The project carries the robot with its licence and commit.');
        assert.ok(!source.includes('so_arm100.png'), 'Menagerie\'s picture is not in the project.');
        const setpoints = saved.sharedParameters.filter(({ symbol }) => symbol.endsWith('Setpoint'));
        assert.equal(setpoints.length, 5);
        assert.ok(setpoints.every(({ mode }) => mode === 'live'), 'Each servo\'s setpoint is a live slider.');
        assert.equal(setpoints.find(({ symbol }) => symbol === 'elbowSetpoint').value, 1.2);
        assert.ok(!mechanism.implementation.bindings.some(({ key }) => key === 'wristRollSetpoint'), 'The joint set free has no setpoint.');
        // 4. Will these servos do it? Built again with the move, the servos' datasheet, a battery and a payload.
        await toolbox.check('#useMove');
        await toolbox.check('#useServo');
        await toolbox.check('#useSupply');
        await toolbox.check('#usePayload');
        assert.equal(await toolbox.inputValue('#runTime'), '9', 'The run is the move\'s length and a second more.');
        await toolbox.click('#buildButton');
        await toolbox.waitForFunction(() => /now in the canvas\..*following the move/.test(document.querySelector('#buildStatus')?.textContent ?? ''), null, { timeout: 60000 }).catch(fail);
        assert.match(await toolbox.locator('#buildStatus').textContent(), /held to the Feetech STS3215.*1\.91 N·m at stall/);
        let demo = null;
        for (let attempt = 0; attempt < 100; attempt += 1) {
            demo = await project.evaluate(() => window.__debugProject?.serialize());
            if (demo?.nodes?.some(({ name }) => name === 'Battery')) break;
            await new Promise((resolve) => setTimeout(resolve, 100));
        }
        assert.ok(demo.nodes.some(({ name }) => name === 'Battery'), 'The canvas has the battery.');
        assert.equal(demo.nodes.filter(({ name }) => / winding$/.test(name)).length, 5, 'A winding for each of the five servos.');
        const shared = (symbol) => demo.sharedParameters.find((parameter) => parameter.symbol === symbol);
        assert.ok(shared('elbowSetpoint')?.schedule?.samples?.length > 2, 'The elbow\'s setpoint follows the move.');
        assert.ok(Math.abs(shared('pitchServoTorqueLimit')?.value - 1.9123) < 1e-3, 'The shoulder\'s servo is held to its datasheet.');
        assert.equal(shared('fixedJawMassWithPayload')?.mode, 'live', 'The payload is a slider.');

        // 4b. Compare: the run as built and a fork of it with a heavier payload, servo by servo.
        await toolbox.bringToFront();
        await toolbox.waitForSelector('#stepCompare:not([hidden])', { timeout: 10000 }).catch(fail);
        assert.equal(await toolbox.locator('#scenarioServo option').count(), 5, 'Each servo can be the weaker one.');
        await toolbox.selectOption('#scenarioSelect', 'heavierPayload');
        await toolbox.fill('#forkAt', '2.5');
        await toolbox.fill('#scenarioPayload', '0.6');
        await toolbox.click('#compareButton');
        await toolbox.waitForFunction(() => document.querySelectorAll('#compareResult .comparison tbody tr').length === 5 || /error/.test(document.querySelector('#compareStatus .notice')?.className ?? ''), null, { timeout: 180000 }).catch(fail);
        const compared = await toolbox.evaluate(() => ({
            status: document.querySelector('#compareStatus')?.innerText,
            headline: document.querySelector('#compareResult .headline')?.textContent,
            rows: [...document.querySelectorAll('#compareResult .comparison tbody tr')].map((row) => [...row.cells].map((cell) => cell.textContent)),
            battery: [...document.querySelectorAll('#compareResult p')].at(-1)?.textContent
        }));
        assert.equal(compared.rows.length, 5, `The comparison has a row for each servo: ${JSON.stringify(compared)}`);
        const pitch = compared.rows.find(([name]) => name === 'Pitch');
        const [before, after] = pitch[3].split(' → ').map(Number);
        assert.ok(after > before, `The shoulder's winding gets hotter with the heavier payload: ${pitch[3]}.`);
        const [usedBefore, usedAfter] = compared.battery.match(/[\d.]+(?= %)/g).map(Number);
        assert.ok(usedAfter > usedBefore, `The battery gives more with the heavier payload: ${compared.battery}`);
        assert.ok(!/^The servos meet it as they met/.test(compared.headline), `The headline says what changed: ${compared.headline}`);
        assert.ok(!(await toolbox.locator('#showCompareButton').isDisabled()), 'The fork can be shown in the canvas.');

        // 5. Run in the app, and shown in Pose Visualizer.
        await project.bringToFront();
        await project.waitForFunction(() => !document.querySelector('#runButton').disabled, null, { timeout: 30000 });
        await project.click('#runButton');
        await project.waitForFunction(() => document.querySelector('#runLaunchDialog').open, null, { timeout: 10000 });
        await project.evaluate(() => { document.querySelector('#runTargetTime').value = '9'; document.querySelector('#startRun').click(); });
        await project.waitForFunction(() => document.querySelector('.resultMode b')?.textContent === 'Results', null, { timeout: 120000 });
        const poseTool = '.addonTool[data-addon-id="konjugate.poseVisualizer"][data-command-id="openPoseVisualizer"]:not([hidden])';
        // Found and clicked as Konjugate's own tests do: the toolstrip may hold it out of sight.
        await project.waitForFunction((selector) => Boolean(document.querySelector(selector)), poseTool, { timeout: 30000 }).catch(async (error) => {
            const tools = await project.evaluate(() => [...document.querySelectorAll('.addonTool')].map((tool) => `${tool.dataset.addonId}/${tool.dataset.commandId}${tool.hidden ? ' (hidden)' : ''}`));
            const status = await project.evaluate(() => ({ mode: document.querySelector('.resultMode')?.innerText, banner: document.querySelector('#runStatus, .runStatus, #statusBar, .statusBar')?.innerText, validation: document.querySelector('#validationSummary')?.innerText, toasts: [...document.querySelectorAll('.toast, [role="alert"]')].map((node) => node.innerText) }));
            throw new Error(`${error.message}\nToolstrip: ${tools.join(', ')}\nInstalled: ${addons.join(', ')}\nProject window: ${JSON.stringify(status)}`);
        });
        const known = new Set(app.windows());
        await project.evaluate((selector) => document.querySelector(selector).click(), poseTool);
        let pose = null;
        for (let attempt = 0; attempt < 150 && !pose; attempt += 1) {
            pose = app.windows().find((candidate) => !known.has(candidate) && candidate.url().includes('konjugate.poseVisualizer'));
            if (!pose) await new Promise((resolve) => setTimeout(resolve, 100));
        }
        assert.ok(pose, 'Pose Visualizer did not open.');
        await pose.waitForLoadState('domcontentloaded');
        // Nothing is guessed from state names: a convention for poses by quaternion, named as the toolbox names them.
        await pose.evaluate(() => { document.querySelector('.conventionPanel').open = true; });
        await pose.click('#newConvention');
        const form = '#conventionForm';
        await pose.fill(`${form} [name="name"]`, 'Quaternion');
        await pose.selectOption(`${form} [name="orientation"]`, 'quaternion');
        for (const role of ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']) await pose.fill(`${form} [name="${role}"]`, role);
        await pose.click(`${form} button[type="submit"]`);
        await pose.waitForFunction(() => (window.__debugPoseVisualizer?.bodies() ?? []).length === 7, null, { timeout: 20000 });
        const shown = await pose.evaluate(() => window.__debugPoseVisualizer.bodies());
        assert.deepEqual(shown.map(({ id }) => id.split(':')[1]), ['base', 'rotationPitch', 'upperArm', 'lowerArm', 'wristPitchRoll', 'fixedJaw', 'movingJaw']);
        assert.ok(shown.every(({ part, vertices }) => part && vertices > 100), 'Each link is drawn with its own mesh.');
        // At its true size: the arm is a few tens of centimetres across, its links apart.
        const spread = Math.max(...[0, 1, 2].map((axis) => Math.max(...shown.map(({ position }) => position[axis])) - Math.min(...shown.map(({ position }) => position[axis]))));
        assert.ok(spread > 0.1 && spread < 1, `The links span ${spread} m.`);
        if (process.env.ROBOTICS_SCREENSHOT) {
            await pose.evaluate(() => document.querySelector('[data-nav-action="fit"]')?.click?.());
            await new Promise((resolve) => setTimeout(resolve, 1500));
            await pose.screenshot({ path: process.env.ROBOTICS_SCREENSHOT });
        }
        console.log('Robotics toolbox window: licence, download, joints, build into the canvas, a fork compared, a run and Pose Visualizer all passed.');
        console.log(`The fork: ${compared.headline} Shoulder ${pitch.slice(1).join(', ')}. ${compared.battery}`);
    } finally {
        await app.close();
    }
} finally {
    await rm(scratch, { recursive: true, force: true });
}
