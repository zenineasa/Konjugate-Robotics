/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics window in the real Konjugate app, as the other toolboxes test theirs: the sibling Konjugate checkout is
// launched in Electron (its own `electron`, driven through Playwright's Electron mode, so no browser is downloaded)
// with a scratch userData holding the freshly built toolbox. Konjugate's own interaction runner cannot be extended
// from outside, so this drives the app as fintech and logistics do. The window is opened from Konjugate's toolstrip
// and driven as a user drives it, by buttons, by clicks in the 3D view and by keys; its session is kept and restored
// by Konjugate's launcher host; Build runs the real importer and opens the model in the canvas, where it is read back
// as the project would save it. Then Konjugate's engine validates those models, when there is a build of it
// (out/engine/konjugateEngine in the Konjugate checkout, or KONJUGATE_ENGINE_PATH).
//
// Run: node tests/interaction/robotWindow.mjs [--other-platform] [--screenshots <directory>]
// --other-platform: the window believes it runs on the other kind of platform (a Mac on Windows or Linux, and the
// other way round), so both sets of keys are tested on one machine.
// Extra Electron switches, for example to run without a GPU: KONJUGATE_ELECTRON_ARGS='--no-sandbox'.

import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { mkdir, mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { installBuiltPackages } from '../../scripts/installDev.mjs';
import { konjugateDir, konjugateModule } from '../../scripts/konjugatePaths.mjs';

const require = createRequire(join(konjugateDir, 'package.json'));
const { _electron: electron } = require('playwright');
const electronPath = require('electron');
const { encodeProjectFile } = await import(pathToFileURL(konjugateModule('src/projectFile.mjs')));

const args = process.argv.slice(2);
const screenshotDirectory = args.includes('--screenshots') ? args[args.indexOf('--screenshots') + 1] : null;
const otherPlatform = args.includes('--other-platform');
const asMac = (process.platform === 'darwin') !== otherPlatform;
const extraArgs = (process.env.KONJUGATE_ELECTRON_ARGS ?? '').split(' ').filter(Boolean);
const env = { ...process.env };
delete env.ELECTRON_RUN_AS_NODE;
const scratch = await mkdtemp(join(tmpdir(), 'konjugate-robotics-'));
const userData = join(scratch, 'userData');

// ---- the app ----------------------------------------------------------------------------------------------------------
await installBuiltPackages(userData);
const projectPath = join(scratch, 'robot.kjt');
await writeFile(projectPath, await encodeProjectFile(JSON.stringify({
    format: 'konjugate', version: 1, metadata: { units: 'SI' }, nodes: [], edges: [], sharedParameters: [],
    runConfigurations: [{ id: 1, name: 'Default', globalTimeStep: 0.001, outputInterval: 0.01 }], activeRunConfigurationId: 1
})));
const app = await electron.launch({ executablePath: electronPath, args: [konjugateDir, ...extraArgs, `--user-data-dir=${userData}`, projectPath], env });
const canvas = await app.firstWindow();
await canvas.waitForLoadState('domcontentloaded');
const toolstripButton = '.addonTool[data-addon-id="konjugate.robotics.toolbox"][data-command-id="openRoboticsToolbox"]';
const problems = [];

// Opens the Robotics window from Konjugate's toolstrip, as a user does, and waits for it to have restored what the
// project keeps for it.
async function openWindow() {
    await canvas.waitForSelector(toolstripButton, { timeout: 30000 });
    const known = new Set(app.windows());
    await canvas.click(toolstripButton);
    let opened = null;
    for (let attempt = 0; attempt < 150 && !opened; attempt += 1) {
        opened = app.windows().find((candidate) => !known.has(candidate) && candidate.url().includes('konjugate.robotics.toolbox'));
        if (!opened) await new Promise((resolve) => setTimeout(resolve, 100));
    }
    assert.ok(opened, 'The Robotics window did not open from the toolstrip.');
    opened.on('pageerror', (error) => problems.push(error.message));
    opened.on('console', (message) => { if (message.type() === 'error') problems.push(message.text()); });
    if (otherPlatform) {
        await opened.addInitScript((mac) => Object.defineProperty(navigator, 'platform', { get: () => (mac ? 'MacIntel' : 'Win32') }), asMac);
        await opened.reload();
    }
    await opened.waitForLoadState('domcontentloaded');
    await opened.waitForFunction(() => window.roboticsWindow && !window.roboticsWindow.state.restoring);
    await opened.setViewportSize?.({ width: 1400, height: 860 }).catch(() => {});
    return opened;
}

// The model open in Konjugate's canvas, as the project would save it.
const inCanvas = () => canvas.evaluate(() => window.__debugProject.serialize());
// Waits until the canvas holds a model that `test` (a function's source, run in the canvas) accepts, and returns it.
async function builtInCanvas(test) {
    await canvas.waitForFunction(`(${test})(window.__debugProject.serialize())`, null, { timeout: 30000 });
    return inCanvas();
}

let page;
try {
    page = await openWindow();
    const shot = async (name) => { if (screenshotDirectory) { await mkdir(screenshotDirectory, { recursive: true }); await page.screenshot({ path: join(screenshotDirectory, `${name}.png`) }); } };
    const command = asMac ? 'Meta' : 'Control';
    const drawing = () => page.evaluate(() => window.roboticsWindow.drawing());
    // A point of the world on the screen, for clicking it in the view.
    const onScreen = (point) => page.evaluate((world) => {
        const { view } = window.roboticsWindow;
        const vector = new view.camera.position.constructor(...world).project(view.camera);
        const rect = view.renderer.domElement.getBoundingClientRect();
        return { x: rect.left + ((vector.x + 1) / 2) * rect.width, y: rect.top + ((1 - vector.y) / 2) * rect.height };
    }, point);
    const clickWorld = async (point, options) => { const { x, y } = await onScreen(point); await page.mouse.click(x, y, options); };
    // A point given in a solid's own frame, in the world, as the drawing has it now.
    const worldOf = (solidId, local) => page.evaluate(async ([id, point]) => {
        const { toWorld } = await import(new URL('lib/math.mjs', location.href).href);
        const solid = window.roboticsWindow.drawing().solids.find((candidate) => candidate.id === id);
        return toWorld({ position: solid.position, rotation: solid.rotation }, point);
    }, [solidId, local]);

    await shot('01-empty');
    assert.match(await page.textContent('#hint'), /Start with a solid/);

    // 1. Add a box by its key, and a capsule by its button.
    await page.keyboard.press('b');
    await page.click('[data-add="capsule"]');
    let current = await drawing();
    assert.deepEqual(current.solids.map(({ shape }) => shape), ['box', 'capsule']);
    const [box, capsule] = current.solids.map(({ id }) => id);
    assert.equal(await page.inputValue('#inspector input[name="name"]'), 'Capsule', 'the new solid is selected, its card open');

    // 2. Rename the box by its card, and make it smaller: one step each.
    await page.click(`#partsTree li[data-id="${box}"]`);
    await page.fill('#inspector input[name="name"]', 'Base');
    await page.press('#inspector input[name="name"]', 'Enter');
    await page.fill('#inspector input[name="size.z"]', '40');
    await page.press('#inspector input[name="size.z"]', 'Enter');
    current = await drawing();
    assert.equal(current.solids[0].name, 'Base');
    assert.equal(current.solids[0].size.z, 0.04);
    await page.click('#viewport', { position: { x: 30, y: 30 } });
    await page.keyboard.press('f');
    await shot('02-two-solids');
    // Joined to nothing, the solids would fall freely, and the checks say so; they still build, each on a free joint.
    assert.match(await page.textContent('#checksList'), /Base is not joined to the Ground, so it falls freely/);
    assert.equal(await page.isDisabled('#buildButton'), false);

    // 3. Fix the base to the Ground: Join as fixed, click the floor, then the base's top (it turns over onto the floor).
    await page.selectOption('#joinKind', 'fixed');
    await page.keyboard.press('j');
    assert.match(await page.textContent('#hint'), /click the point on the first solid/);
    await clickWorld([-0.15, -0.12, 0]);
    assert.match(await page.textContent('#hint'), /Now click the point on the second solid/);
    await clickWorld(await worldOf(box, [0, 0, 0.02]));
    current = await drawing();
    assert.equal(current.joints.length, 1, 'the fixed joint is made');
    assert.equal(current.joints[0].kind, 'fixed');
    const base = current.solids.find(({ id }) => id === box);
    assert.ok(Math.abs(base.position[0] + 0.15) < 1e-9 && Math.abs(base.position[1] + 0.12) < 1e-9 && Math.abs(base.position[2] - 0.02) < 1e-9,
        `the base sits on the floor at the point clicked, not ${base.position}`);

    // 4. Hinge the capsule to the base's side: click the base's +x face, then the capsule's side.
    await page.selectOption('#joinKind', 'hinge');
    await page.keyboard.press('Escape');
    await page.click('[data-tool="join"]');
    await page.keyboard.press('f');
    await clickWorld(await worldOf(box, [0.05, 0, 0]));
    await clickWorld(await worldOf(capsule, [0.02, 0, 0.05]));
    current = await drawing();
    assert.equal(current.joints.length, 2, 'the hinge is made');
    const hinge = current.joints[1];
    assert.equal(hinge.kind, 'hinge');
    const checks = await page.evaluate(() => [...document.querySelectorAll('#checksList li')].map((item) => [item.className, item.textContent]));
    assert.ok(!checks.some(([level]) => level === 'error'), `no errors: ${JSON.stringify(checks)}`);
    await page.keyboard.press('Escape');
    await page.keyboard.press('f');
    await shot('03-joined');

    // 5. Drive the hinge with a servo, from its card; then undo that and redo it.
    await page.click(`#partsTree li[data-id="${hinge.id}"]`);
    await page.selectOption('#inspector select[name="drive.mode"]', 'servo');
    await page.fill('#inspector input[name="drive.setpoint"]', '30');
    await page.press('#inspector input[name="drive.setpoint"]', 'Enter');
    current = await drawing();
    assert.equal(current.joints[1].drive.mode, 'servo');
    assert.ok(Math.abs(current.joints[1].drive.setpoint - Math.PI / 6) < 1e-9);
    await shot('04-servo');
    await page.click('#viewport', { position: { x: 30, y: 30 } });
    await page.keyboard.press(`${command}+z`);
    assert.equal((await drawing()).joints[1].drive.setpoint, 0, 'undo takes the setpoint back');
    assert.match(await page.textContent('#toast'), /Undid: Setpoint of Hinge/);
    await page.keyboard.press(asMac ? 'Meta+Shift+z' : 'Control+y');
    assert.ok(Math.abs((await drawing()).joints[1].drive.setpoint - Math.PI / 6) < 1e-9, 'redo puts it back');

    // 6. Move the capsule by typing its place: the joint's two points part, and the checks say so.
    await page.click(`#partsTree li[data-id="${capsule}"]`);
    const before = (await drawing()).solids[1].position;
    await page.fill('#inspector input[name="position.2"]', String(Math.round(before[2] * 1000) + 30));
    await page.press('#inspector input[name="position.2"]', 'Enter');
    assert.match(await page.textContent('#checksList'), /apart/);
    await page.keyboard.press(`${command}+z`);
    assert.doesNotMatch(await page.textContent('#checksList'), /apart/);

    // 7. Duplicate and delete by keys.
    await page.click(`#partsTree li[data-id="${capsule}"]`);
    await page.click('#viewport', { position: { x: 30, y: 30 }, modifiers: ['Shift'] });
    await page.keyboard.press(`${command}+d`);
    assert.equal((await drawing()).solids.length, 3);
    await page.keyboard.press('Delete');
    assert.equal((await drawing()).solids.length, 2);

    // 8. The session is kept with the project as the drawing changes: closed and opened again from the toolstrip, the
    // window has the drawing back, before anything is built.
    await page.waitForTimeout(1500);
    await page.close();
    page = await openWindow();
    await page.waitForFunction(() => window.roboticsWindow.drawing().joints.length === 2);
    assert.deepEqual((await drawing()).solids.map(({ name }) => name), ['Base', 'Capsule']);

    // 9. Build: the real importer builds the model, and it is opened in the canvas.
    await page.click('#buildButton');
    const document = await builtInCanvas((model) => model.nodes.length === 3 && model.edges.length === 2);
    assert.match(await page.textContent('#buildStatus'), /Built into the canvas: 3 nodes \(the Ground and 2 solids\) and 2 relationships/);
    await shot('05-built');
    assert.deepEqual(document.nodes.map(({ name }) => name), ['Ground', 'Base', 'Capsule']);
    assert.deepEqual(document.edges.map(({ name, implementation }) => [name, implementation]), [['Fixed', undefined], ['Hinge', undefined]]);
    assert.ok(document.edges.every(({ equationModel }) => equationModel.outputs.length > 0), 'each joint carries its equations');

    // 10. Reopened after the build, the window restores the drawing kept with the model it opened.
    await page.close();
    page = await openWindow();
    await page.waitForFunction(() => window.roboticsWindow.drawing().solids.length === 2);
    assert.equal(await page.inputValue('#robotName'), 'Robot');

    // 11. A closed loop and ball joints build too: the four-bar's last pin closes its loop, and a ball joint's card has
    // its damping, friction and armature but no drive.
    await page.selectOption('#exampleSelect', 'fourBar');
    await page.waitForFunction(() => window.roboticsWindow.drawing().name === 'Four-bar linkage');
    assert.match(await page.textContent('#checksList'), /Closing pin closes a loop: Build holds its two solids together at its point, free to turn only about its axis/);
    await page.click('#partsTree li:has-text("Closing pin")');
    assert.match(await page.textContent('#inspector'), /It closes a loop: Build holds it with a force of its own/);
    await page.click('#buildButton');
    const linkage = await builtInCanvas((model) => model.edges.some(({ name }) => name === 'Closing pin'));
    await page.selectOption('#exampleSelect', 'ballPendulum');
    await page.waitForFunction(() => window.roboticsWindow.drawing().name === 'Ball-joint pendulum');
    assert.match(await page.textContent('#checksList'), /Ready to build/);
    await page.click('#partsTree li:has-text("Shoulder")');
    assert.match(await page.textContent('#inspector'), /Damping/);
    // A ball is driven too: as a servo, its setpoint is a turn about each of its parent's axes.
    await page.selectOption('#inspector select[name="drive.mode"]', 'servo');
    await page.fill('#inspector input[name="drive.setpoint.1"]', '20');
    await page.press('#inspector input[name="drive.setpoint.1"]', 'Enter');
    const shoulder = (await drawing()).joints.find(({ name }) => name === 'Shoulder');
    assert.equal(shoulder.drive.mode, 'servo');
    assert.ok(Math.abs(shoulder.drive.setpoint[1] - (20 * Math.PI) / 180) < 1e-9, 'The setpoint turn should be kept in radians.');
    await page.check('#inspector input[name="limited"]');
    assert.match(await page.textContent('#inspector'), /Most turn/);
    assert.match(await page.textContent('#checksList'), /Ready to build/);
    await shot('06-ball');

    // 12. Motors, the battery, the jog panel and keyframes: the pick-and-place arm.
    await page.selectOption('#exampleSelect', 'pickAndPlace');
    await page.waitForFunction(() => window.roboticsWindow.drawing().name === 'Pick and place');
    assert.equal(await page.locator('#posePanel input[name="jog"]').count(), 6, 'a jog slider for each joint a servo or a motor drives');
    assert.equal(await page.locator('#posePanel .keyframes li').count(), 8);
    const handAt = () => page.evaluate(() => { const view = window.roboticsWindow.view; const hand = window.roboticsWindow.drawing().solids.find(({ name }) => name === 'Hand'); return view.meshes.get(hand.id).position.toArray(); });
    const drawnHand = await handAt();
    // Jog the shoulder: the view shows the robot posed; the drawing is unchanged; Escape goes back.
    const shoulderJog = page.locator('#posePanel label.jog', { hasText: 'Shoulder' }).locator('input');
    const jogged = await shoulderJog.evaluate((input) => { input.value = '0.5'; input.dispatchEvent(new Event('input', { bubbles: true })); return Number(input.value); });
    assert.ok(Math.abs(jogged - 0.5) < 0.01, 'the slider moves in steps of half a degree');
    assert.notDeepEqual(await handAt(), drawnHand, 'jogging the shoulder should move the hand in the view');
    assert.equal((await drawing()).solids.find(({ name }) => name === 'Hand').position.join(), (await page.evaluate(() => window.roboticsWindow.drawing().solids.find(({ name }) => name === 'Hand').position.join())));
    await shot('07-jogged');
    await page.click('#viewport', { position: { x: 10, y: 10 } });
    // K records the pose as a keyframe a second after the last; undo takes it back.
    await page.keyboard.press('k');
    let keyframes = (await drawing()).keyframes;
    assert.equal(keyframes.length, 9);
    const added = keyframes.find(({ time }) => time === 7);
    const shoulderId = (await drawing()).joints.find(({ name }) => name === 'Shoulder').id;
    assert.ok(added && Math.abs(added.setpoints[shoulderId] - jogged) < 1e-12, 'the new keyframe keeps the jogged shoulder');
    await page.keyboard.press(`${command}+z`);
    assert.equal((await drawing()).keyframes.length, 8);
    await page.keyboard.press('Escape');
    assert.deepEqual(await handAt(), drawnHand, 'Escape shows the robot as drawn again');
    // Clicking a keyframe shows it; ] shows the next.
    await page.click('#posePanel .keyframes li:nth-child(2)', { position: { x: 120, y: 8 } });
    const second = (await drawing()).keyframes[1];
    assert.equal(await page.evaluate(() => window.roboticsWindow.state.pose.keyframe), second.id);
    await page.click('#viewport', { position: { x: 10, y: 10 } });
    await page.keyboard.press(']');
    assert.equal(await page.evaluate(() => window.roboticsWindow.state.pose.keyframe), (await drawing()).keyframes[2].id);
    await page.keyboard.press('Escape');
    // A motor's card: its controller and its motor; the battery's on the robot's card.
    await page.click('#partsTree li:has-text("Waist")');
    assert.match(await page.textContent('#inspector'), /Winding resistance[\s\S]*Gear ratio/);
    await page.fill('#inspector input[name="motor.gearRatio"]', '120');
    await page.press('#inspector input[name="motor.gearRatio"]', 'Enter');
    assert.equal((await drawing()).joints.find(({ name }) => name === 'Waist').drive.motor.gearRatio, 120);
    await page.click('#viewport', { position: { x: 10, y: 10 } });
    await page.keyboard.press('Escape');
    await page.keyboard.press('Escape');
    assert.match(await page.textContent('#inspector'), /Battery/);
    await page.fill('#inspector input[name="battery.capacity"]', '3');
    await page.press('#inspector input[name="battery.capacity"]', 'Enter');
    assert.equal((await drawing()).battery.capacity, 10800);
    // Built: a motor node per motor, the battery, and each keyframed setpoint scheduled.
    await page.click('#buildButton');
    const pickAndPlace = await builtInCanvas((model) => model.nodes.some(({ name }) => name === 'Battery'));
    assert.ok(['Waist motor', 'Shoulder motor', 'Elbow motor', 'Wrist motor'].every((name) => pickAndPlace.nodes.some((node) => node.name === name)));
    assert.equal(pickAndPlace.sharedParameters.find(({ name }) => name === 'Waist setpoint').schedule.samples.length, 8);
    await shot('08-pick-and-place');

    assert.deepEqual(problems, [], `the page reported problems: ${problems.join(' | ')}`);

    // 13. Konjugate's engine accepts the models, when there is a build of it.
    const engine = process.env.KONJUGATE_ENGINE_PATH ?? konjugateModule(join('out', 'engine', process.platform === 'win32' ? 'konjugateEngine.exe' : 'konjugateEngine'));
    if (existsSync(engine)) {
        const { decodeValidationReport } = await import(pathToFileURL(konjugateModule('src/reportProtocol.mjs')));
        const directory = scratch;
        for (const [name, built] of [['robot', document], ['linkage', linkage], ['pickAndPlace', pickAndPlace]]) {
            await writeFile(join(directory, `${name}.kjt`), await encodeProjectFile(JSON.stringify(built)));
            spawnSync(engine, ['validate', join(directory, `${name}.kjt`), '--report', join(directory, `${name}.report`)]);
            const report = decodeValidationReport(await readFile(join(directory, `${name}.report`)));
            assert.equal(report.valid, true, `${name}: ${JSON.stringify(report.issues)}`);
        }
        console.log('The engine accepts the models the window built.');
    } else {
        console.log(`No engine at ${engine}: the built model was not validated.`);
    }
    console.log(`Robotics window in Konjugate: every step passed, as a ${asMac ? 'Mac' : 'Windows or Linux'} user.`);
} finally {
    await app.close().catch(() => {});
    await rm(scratch, { recursive: true, force: true });
}
