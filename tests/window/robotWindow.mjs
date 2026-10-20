/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics window in a plain browser, no Konjugate build needed: a stand-in for Konjugate's launcher bridge keeps
// the session in memory and runs the real importer, and the page is driven as a user drives it, by buttons, by
// clicks in the 3D view and by keys. Then the model it built is checked by Konjugate's engine (`validate`), when an
// engine build is there (out/engine/konjugateEngine in the Konjugate checkout, or KONJUGATE_ENGINE_PATH).
//
// Run: node tests/window/robotWindow.mjs [--screenshots <directory>] [--mac]

import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { createServer } from 'node:http';
import { tmpdir } from 'node:os';
import { extname, join, normalize } from 'node:path';
import { pathToFileURL } from 'node:url';
import { konjugateModule, roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const { chromium } = await import(pathToFileURL(konjugateModule('node_modules/playwright/index.mjs')));
const args = process.argv.slice(2);
const screenshotDirectory = args.includes('--screenshots') ? args[args.indexOf('--screenshots') + 1] : null;
const asMac = args.includes('--mac');

// ---- serving the package --------------------------------------------------------------------------------------------
const toolbox = join(roboticsRoot, 'packages', 'toolbox');
const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json' };
const server = createServer(async (request, response) => {
    const path = decodeURIComponent(new URL(request.url, 'http://localhost').pathname);
    // Konjugate's window asks for no icon; a browser does.
    if (path === '/favicon.ico') { response.writeHead(204); response.end(); return; }
    const file = normalize(join(toolbox, path === '/' ? 'index.html' : path));
    if (!file.startsWith(toolbox) || !existsSync(file)) { response.writeHead(404); response.end(); return; }
    response.writeHead(200, { 'content-type': types[extname(file)] ?? 'application/octet-stream' });
    response.end(await readFile(file));
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
const origin = `http://127.0.0.1:${server.address().port}`;

// ---- the stand-in bridge ----------------------------------------------------------------------------------------------
// Runs in the page before its scripts: keeps what a window keeps with a project, and runs the importer as the host
// does.
const bridge = ({ asMac }) => {
    if (asMac) Object.defineProperty(navigator, 'platform', { get: () => 'MacIntel' });
    const calls = [];
    let session = window.__session ?? null;
    let built = null;
    window.__bridge = { calls, get session() { return session; }, get built() { return built; } };
    window.konjugateLauncher = {
        restoreSession: async () => { calls.push('restoreSession'); session ??= window.__session ?? null; return { session }; },
        keepSession: async (value) => { calls.push('keepSession'); session = structuredClone(value); return {}; },
        runImport: async (importerId, options) => {
            calls.push(`runImport:${options.step}`);
            const module = await import('/importers/drawing.mjs');
            const result = await module.default({ files: [], options: structuredClone(options) });
            if (!result.ok) return { imported: false, report: result.report };
            built = result.document;
            return { imported: Boolean(result.document), report: result.report, data: result.data };
        },
        openInCanvas: async (scenarioId, options) => { calls.push('openInCanvas'); session = structuredClone(options.session); return {}; }
    };
};

const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH ?? (existsSync('/opt/pw-browsers/chromium') ? '/opt/pw-browsers/chromium' : undefined), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 860 } });
const problems = [];
page.on('pageerror', (error) => problems.push(error.message));
page.on('console', (message) => { if (message.type() === 'error') problems.push(message.text()); });
await page.addInitScript(bridge, { asMac });
await page.goto(origin);
await page.waitForFunction(() => window.roboticsWindow && window.__bridge.calls.includes('restoreSession'));
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
    const { toWorld } = await import('/lib/math.mjs');
    const solid = window.roboticsWindow.drawing().solids.find((candidate) => candidate.id === id);
    return toWorld({ position: solid.position, rotation: solid.rotation }, point);
}, [solidId, local]);

try {
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

    // 8. The session is kept as the drawing changes.
    await page.waitForFunction(() => window.__bridge.session?.drawing?.joints?.length === 2);

    // 9. Build: the real importer builds the model, and it is opened in the canvas.
    await page.click('#buildButton');
    await page.waitForFunction(() => window.__bridge.calls.includes('openInCanvas'));
    assert.match(await page.textContent('#buildStatus'), /Built into the canvas: 3 nodes \(the Ground and 2 solids\) and 2 relationships/);
    await shot('05-built');
    const document = await page.evaluate(() => window.__bridge.built);
    assert.deepEqual(document.nodes.map(({ name }) => name), ['Ground', 'Base', 'Capsule']);
    assert.deepEqual(document.edges.map(({ name, implementation }) => [name, implementation]), [['Fixed', undefined], ['Hinge', undefined]]);
    assert.ok(document.edges.every(({ equationModel }) => equationModel.outputs.length > 0), 'each joint carries its equations');

    // 10. Reopened, the window restores the drawing from the session.
    const session = await page.evaluate(() => window.__bridge.session);
    await page.addInitScript((kept) => { window.__session = kept; }, session);
    await page.reload();
    await page.waitForFunction(() => window.roboticsWindow?.drawing().solids.length === 2);
    assert.equal(await page.inputValue('#robotName'), 'Robot');

    // 11. A closed loop and ball joints build too: the four-bar's last pin closes its loop, and a ball joint's card has
    // its damping, friction and armature but no drive.
    await page.selectOption('#exampleSelect', 'fourBar');
    await page.waitForFunction(() => window.roboticsWindow.drawing().name === 'Four-bar linkage');
    assert.match(await page.textContent('#checksList'), /Closing pin closes a loop: Build holds its two solids together at its point, free to turn only about its axis/);
    await page.click('#partsTree li:has-text("Closing pin")');
    assert.match(await page.textContent('#inspector'), /It closes a loop: Build holds it with a force of its own/);
    await page.click('#buildButton');
    await page.waitForFunction(() => window.__bridge.built?.edges?.some(({ name }) => name === 'Closing pin'));
    const linkage = await page.evaluate(() => window.__bridge.built);
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

    assert.deepEqual(problems, [], `the page reported problems: ${problems.join(' | ')}`);

    // 12. Konjugate's engine accepts the models, when there is a build of it.
    const engine = process.env.KONJUGATE_ENGINE_PATH ?? konjugateModule(join('out', 'engine', process.platform === 'win32' ? 'konjugateEngine.exe' : 'konjugateEngine'));
    if (existsSync(engine)) {
        const { encodeProjectFile } = await import(pathToFileURL(konjugateModule('src/projectFile.mjs')));
        const { decodeValidationReport } = await import(pathToFileURL(konjugateModule('src/reportProtocol.mjs')));
        const directory = join(tmpdir(), `robotics-window-${process.pid}`);
        await mkdir(directory, { recursive: true });
        for (const [name, built] of [['robot', document], ['linkage', linkage]]) {
            await writeFile(join(directory, `${name}.kjt`), await encodeProjectFile(JSON.stringify(built)));
            spawnSync(engine, ['validate', join(directory, `${name}.kjt`), '--report', join(directory, `${name}.report`)]);
            const report = decodeValidationReport(await readFile(join(directory, `${name}.report`)));
            assert.equal(report.valid, true, `${name}: ${JSON.stringify(report.issues)}`);
        }
        console.log('The engine accepts the models the window built.');
    } else {
        console.log(`No engine at ${engine}: the built model was not validated.`);
    }
    console.log(`Robotics window: every step passed${asMac ? ' (as a Mac user)' : ''}.`);
} finally {
    await browser.close();
    server.close();
}
