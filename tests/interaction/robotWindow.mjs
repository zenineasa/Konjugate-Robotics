/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics Toolbox window in the real app, offline: the main process's fetch is replaced so GitHub's API and
// Menagerie's raw files are answered from the SO-ARM100 fetched to this computer (npm run fetch:model trs_so_arm100).
// Through the window:
//   - nothing is downloaded before the licence is shown; the licence is the robot's, Apache-2.0
//   - the robot is downloaded into Konjugate's store of downloaded files, every file of its folder but its picture
//   - used, its six joints are listed, each with its own servo; one is set free, one given a setpoint
//   - built, the mechanism is in the canvas: the project carries the robot with its licence and commit, and its
//     servos' setpoints are live shared parameters, the one set in the window at its value
// Uses Playwright from the Konjugate checkout, as the logistics toolbox's interaction tests do.

import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { mkdtemp, readdir, readFile, rm, writeFile } from 'node:fs/promises';
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
        console.log('Robotics toolbox window: licence, download, joints and build into the canvas all passed.');
    } finally {
        await app.close();
    }
} finally {
    await rm(scratch, { recursive: true, force: true });
}
