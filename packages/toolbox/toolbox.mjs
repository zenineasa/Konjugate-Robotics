/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics Toolbox window: a robot from MuJoCo Menagerie, fetched at the user's request after its licence is shown
// and kept in Konjugate's store of downloaded files; its joints, each given a mode; and a mechanism built from it into
// the canvas, the project carrying the robot with its licence and where it came from.
//
// The DOM is built with elements and text, never markup strings, so nothing a server sends is read as markup.

import { robots, robotByFolder, storedName } from './lib/catalog.mjs';
import { latestCommit, licenceName, modelFiles, rawUrl, repository } from './lib/menagerie.mjs';
import { moves } from './lib/moves.mjs';
import { servoById } from './lib/servos.mjs';

const api = window.konjugateLauncher;
const importerId = 'robot';
const $ = (selector) => document.querySelector(selector);

const state = {
    // Robots kept on this computer, by folder: { commit, files: [{ id, name, path, bytes }], fetchedAt }.
    stored: new Map(),
    // A robot's licence, fetched to show before it is downloaded, by folder: { commit, name, text }.
    licences: new Map(),
    downloading: null,
    selected: null,
    description: null,
    licence: null,
    // Each joint's chosen mode and value, by name: { mode: 'servo' | 'torque' | 'free', value }.
    joints: new Map(),
    restoring: false
};

function element(tag, properties = {}, children = []) {
    const node = document.createElement(tag);
    for (const [key, value] of Object.entries(properties)) {
        if (key === 'text') node.textContent = value;
        else if (key === 'class') node.className = value;
        else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
        else if (value !== undefined && value !== null && value !== false) node.setAttribute(key, value === true ? '' : value);
    }
    node.append(...children.filter((child) => child !== null && child !== undefined));
    return node;
}

function notice(kind, text) {
    return element('div', { class: `notice ${kind}`, role: kind === 'error' ? 'alert' : 'status', text });
}

function show(target, ...nodes) {
    $(target).replaceChildren(...nodes);
}

async function call(promise) {
    const answer = await promise;
    if (!answer?.ok) throw new Error(answer?.message ?? 'The request failed.');
    return answer;
}

const fetchJson = async (url) => JSON.parse((await call(api.fetchText(url))).text);
const fetchText = async (url) => (await call(api.fetchText(url))).text;
const shortCommit = (commit) => commit.slice(0, 7);
const megabytes = (bytes) => `${(bytes / 1024 / 1024).toFixed(1)} MB`;

// ---- robots kept on this computer ---------------------------------------------------------------------------

async function refreshStored() {
    const { files } = await call(api.storedFiles());
    const stored = new Map();
    for (const file of files) {
        const { robot, commit, path } = file.details ?? {};
        if (!robotByFolder(robot) || !commit || !path) continue;
        const entry = stored.get(robot);
        // A robot kept at more than one commit: the latest kept is the one used.
        if (entry && entry.commit !== commit) {
            if (file.storedAt <= entry.fetchedAt) continue;
            stored.delete(robot);
        }
        const current = stored.get(robot) ?? { commit, files: [], fetchedAt: file.storedAt };
        current.files.push({ id: file.id, name: file.name, path, bytes: file.bytes });
        if (file.storedAt < current.fetchedAt) current.fetchedAt = file.storedAt;
        stored.set(robot, current);
    }
    state.stored = stored;
}

// ---- the robots list -------------------------------------------------------------------------------------------

function renderRobots() {
    show('#robots', ...robots.map((robot) => {
        const kept = state.stored.get(robot.folder);
        const licence = state.licences.get(robot.folder);
        const downloading = state.downloading?.folder === robot.folder;
        const status = kept
            ? `On this computer: Menagerie at ${shortCommit(kept.commit)}, ${kept.files.length} files, ${megabytes(kept.files.reduce((sum, file) => sum + file.bytes, 0))}.`
            : 'Not on this computer.';
        const actions = element('div', { class: 'actions' }, [
            kept ? element('button', { type: 'button', class: 'primary', text: state.selected === robot.folder ? 'In use' : 'Use', disabled: state.selected === robot.folder || Boolean(state.downloading), onclick: () => useRobot(robot.folder) }) : null,
            element('button', { type: 'button', text: licence ? 'Hide licence' : 'Licence', disabled: Boolean(state.downloading), onclick: () => toggleLicence(robot.folder) }),
            licence && !kept ? element('button', { type: 'button', class: 'primary', text: 'Download', disabled: Boolean(state.downloading), onclick: () => download(robot.folder) }) : null,
            kept ? element('button', { type: 'button', text: 'Remove from this computer', disabled: Boolean(state.downloading), onclick: () => remove(robot.folder) }) : null
        ]);
        const licenceBlock = licence ? element('div', { class: 'licence' }, [
            element('p', { text: `Licence: ${licence.name ?? 'not recognised; read it below'}. From ${repository.url}, folder ${robot.folder}, at commit ${shortCommit(licence.commit)}. ${kept ? '' : 'Downloading it keeps its files on this computer; a project built from it carries the robot with this licence.'}` }),
            element('pre', { text: licence.text })
        ]) : null;
        const progress = downloading ? element('div', {}, [
            element('progress', { max: state.downloading.total, value: state.downloading.done }),
            element('p', { class: 'muted small', text: state.downloading.message })
        ]) : null;
        return element('li', { class: `robot${state.selected === robot.folder ? ' selected' : ''}` }, [
            element('h3', {}, [document.createTextNode(`${robot.name} `), element('span', { class: 'maker', text: robot.maker })]),
            element('p', { text: robot.what }),
            element('p', { class: 'muted small', text: status }),
            actions, licenceBlock, progress
        ]);
    }));
}

// The licence of the robot at Menagerie's current commit, which a download then pins to.
async function toggleLicence(folder) {
    if (state.licences.has(folder)) {
        state.licences.delete(folder);
        renderRobots();
        return;
    }
    show('#robotStatus', notice('', 'Fetching the licence…'));
    try {
        const kept = state.stored.get(folder);
        const commit = kept?.commit ?? await latestCommit(fetchJson);
        const text = await fetchText(rawUrl(commit, `${folder}/LICENSE`));
        state.licences.set(folder, { commit, name: licenceName(text), text });
        show('#robotStatus');
    } catch (error) {
        show('#robotStatus', notice('error', `The licence could not be fetched: ${error.message}`));
    }
    renderRobots();
}

// Every file of the robot's folder at the licence's commit, into the store, each with where it belongs.
async function download(folder) {
    const licence = state.licences.get(folder);
    if (!licence) return;
    const downloadId = `${folder}-${Date.now()}`;
    state.downloading = { folder, downloadId, done: 0, total: 1, message: 'Listing the robot\'s files…' };
    renderRobots();
    const kept = [];
    let outcome = null;
    try {
        const files = await modelFiles(fetchJson, licence.commit, folder);
        state.downloading.total = files.reduce((sum, file) => sum + (file.size ?? 0), 0) || files.length;
        let before = 0;
        for (const [index, file] of files.entries()) {
            state.downloading.message = `${index + 1} of ${files.length}: ${file.path}`;
            state.downloading.base = before;
            renderRobots();
            const { file: stored } = await call(api.downloadFile(file.url, {
                name: storedName(file.path), server: 'MuJoCo Menagerie on GitHub', downloadId,
                details: { robot: folder, commit: licence.commit, path: file.path, licence: licence.name }
            }));
            kept.push(stored.id);
            before += file.size ?? 1;
            state.downloading.done = before;
        }
        outcome = notice('ok', `${robotByFolder(folder).name} is kept on this computer: ${files.length} files from Menagerie at ${shortCommit(licence.commit)}.`);
    } catch (error) {
        // A robot half downloaded is no robot: what was kept of it is removed.
        for (const id of kept) await api.removeStoredFile(id);
        outcome = notice(/cancelled/.test(error.message) ? 'warning' : 'error', `The download did not finish: ${error.message}`);
    } finally {
        state.downloading = null;
        await refreshStored();
        // The licence was shown to decide on the download; kept, the robot's row says what is kept instead.
        if (state.stored.has(folder)) state.licences.delete(folder);
        renderRobots();
        // Said once the list shows it.
        show('#robotStatus', outcome);
    }
}

async function remove(folder) {
    const kept = state.stored.get(folder);
    if (!kept || !window.confirm(`Remove ${robotByFolder(folder).name} from this computer? Projects built from it keep their copy.`)) return;
    for (const file of kept.files) await call(api.removeStoredFile(file.id));
    if (state.selected === folder) clearSelection();
    await refreshStored();
    renderRobots();
}

api?.onProgress?.((progress) => {
    if (progress?.stage !== 'download' || progress.downloadId !== state.downloading?.downloadId) return;
    state.downloading.done = (state.downloading.base ?? 0) + (progress.done ?? 0);
    const bar = document.querySelector('.robot progress');
    if (bar) bar.value = state.downloading.done;
});

// ---- using a robot: its joints --------------------------------------------------------------------------------

function clearSelection() {
    state.selected = null;
    state.description = null;
    $('#stepJoints').hidden = true;
    $('#stepBuild').hidden = true;
}

// The importer's options that name the robot's files: its main MJCF and each stored name's path.
function importOptions(folder) {
    const kept = state.stored.get(folder);
    return { main: robotByFolder(folder).main, paths: Object.fromEntries(kept.files.map((file) => [file.name, file.path])) };
}

async function useRobot(folder, { joints = null, name = null } = {}) {
    const kept = state.stored.get(folder);
    if (!kept) return;
    show('#robotStatus', notice('', `Reading ${robotByFolder(folder).name}…`));
    try {
        await call(api.useStoredFiles(importerId, 'model', kept.files.map((file) => file.id)));
        const answer = await call(api.runImport(importerId, { step: 'describe', useBinaryFiles: true, ...importOptions(folder) }));
        if (answer.report?.errors?.length) throw new Error(answer.report.errors.join(' '));
        state.selected = folder;
        state.description = answer.data.description;
        state.licence = answer.data.licence;
        state.joints = new Map();
        for (const joint of drivableJoints()) {
            const saved = joints?.[joint.name];
            state.joints.set(joint.name, saved ?? { mode: servoOf(joint) ? 'servo' : 'free', value: joint.initialPosition });
        }
        $('#modelName').value = name ?? robotByFolder(folder).name;
        show('#robotStatus');
        renderJoints();
        keepSessionSoon();
    } catch (error) {
        show('#robotStatus', notice('error', error.message));
    }
    renderRobots();
}

const drivableJoints = () => (state.description?.joints ?? []).filter(({ kind, name }) => (kind === 'hinge' || kind === 'slide') && name);
const servoOf = (joint) => {
    const servos = state.description.actuators.filter((actuator) => actuator.joint === joint.name);
    return servos.length === 1 && servos[0].kind === 'position' ? servos[0] : null;
};

function renderJoints() {
    const joints = drivableJoints();
    const floating = state.description.joints.filter(({ kind }) => kind === 'free' || kind === 'ball').length;
    $('#jointsIntro').textContent = `${joints.length} joints${floating ? `, and ${floating} ${floating === 1 ? 'joint' : 'joints'} MuJoCo moves on its own (a floating base, say)` : ''}; ${joints.filter(servoOf).length} with a servo of the model's own.`;
    show('#joints', ...joints.map((joint) => {
        const choice = state.joints.get(joint.name);
        const servo = servoOf(joint);
        const mode = element('select', { 'aria-label': `Mode of ${joint.name}` }, [
            servo ? element('option', { value: 'servo', text: `Servo (gain ${servo.gain}${servo.forceRange ? `, up to ${servo.forceRange[1]} N·m` : ''})` }) : null,
            element('option', { value: 'torque', text: 'Torque' }),
            element('option', { value: 'free', text: 'Free' })
        ]);
        mode.value = choice.mode;
        const value = element('input', { type: 'number', step: joint.kind === 'hinge' ? '0.01' : '0.001', 'aria-label': `Setpoint or torque of ${joint.name}`, disabled: choice.mode === 'free' });
        value.value = String(choice.mode === 'free' ? '' : choice.value);
        mode.addEventListener('change', () => {
            const next = { mode: mode.value, value: mode.value === 'servo' ? joint.initialPosition : 0 };
            state.joints.set(joint.name, next);
            renderJoints();
            keepSessionSoon();
        });
        value.addEventListener('change', () => {
            const number = Number(value.value);
            if (!Number.isFinite(number)) return;
            state.joints.set(joint.name, { ...state.joints.get(joint.name), value: number });
            keepSessionSoon();
        });
        const range = joint.range ? `${joint.range[0].toFixed(2)} to ${joint.range[1].toFixed(2)} ${joint.kind === 'hinge' ? 'rad' : 'm'}` : 'unlimited';
        return element('tr', {}, [element('td', { text: joint.name }), element('td', { class: 'muted', text: range }), element('td', {}, [mode]), element('td', {}, [value])]);
    }));
    $('#stepJoints').hidden = false;
    $('#stepBuild').hidden = false;
    renderServoOptions();
}

// ---- will these servos do it? ------------------------------------------------------------------------------------

// The robot's move and its servos' datasheet, where the toolbox has them: offered beside the build.
const robotMove = () => Object.entries(moves[state.selected] ?? {})[0] ?? null;
const robotServo = () => servoById(robotByFolder(state.selected)?.servo);

function renderServoOptions() {
    const move = robotMove();
    const servo = robotServo();
    $('#servoOptions').hidden = !move && !servo;
    $('#moveOption').hidden = !move;
    if (move) $('#moveLabel').textContent = `${move[1].name}: ${move[1].what}`;
    $('#servoLabel').textContent = servo
        ? `Servos as their datasheet gives them: ${servo.name}, drawing current for their torque, held to ${servo.stallTorque.toFixed(2)} N·m at stall`
        : 'No datasheet for this robot\'s servos yet';
    $('#useServo').disabled = !servo;
    const servoOn = $('#useServo').checked && Boolean(servo);
    for (const id of ['useSupply', 'supplyVoltage', 'supplyCapacity', 'usePayload', 'payloadMass']) $(`#${id}`).disabled = !servoOn;
    $('#servoNote').textContent = servoOn
        ? 'The windings\' resistance is worked out from the datasheet; their heat capacity and resistance to ambient are assumed. The payload is added to the gripper\'s own mass.'
        : '';
}

for (const id of ['useMove', 'useServo', 'useSupply', 'usePayload', 'supplyVoltage', 'supplyCapacity', 'payloadMass']) {
    $(`#${id}`).addEventListener('change', () => {
        // A move runs for its length and a second more, unless a run length has been set since.
        if (id === 'useMove' && $('#useMove').checked && robotMove()) $('#runTime').value = String(robotMove()[1].keyframes.at(-1)[0] + 1);
        renderServoOptions();
        keepSessionSoon();
    });
}

// The importer's options for what is ticked.
function servoBuildOptions() {
    const servoOn = $('#useServo').checked && Boolean(robotServo());
    return {
        folder: state.selected,
        ...($('#useMove').checked && robotMove() ? { move: robotMove()[0] } : {}),
        ...(servoOn ? { servo: robotByFolder(state.selected).servo } : {}),
        ...(servoOn && $('#useSupply').checked ? { supply: { voltage: Number($('#supplyVoltage').value) || 7.4, capacity: (Number($('#supplyCapacity').value) || 2) * 3600 } } : {}),
        ...(servoOn && $('#usePayload').checked ? { payload: { body: robotByFolder(state.selected).gripper, mass: Number($('#payloadMass').value) || 0 } } : {})
    };
}

// ---- building it into the canvas -------------------------------------------------------------------------------

function provenance(folder) {
    const kept = state.stored.get(folder);
    return {
        source: 'MuJoCo Menagerie', repository: repository.url, folder, commit: kept.commit, fetchedAt: kept.fetchedAt,
        licence: { name: licenceName(state.licence ?? ''), file: state.licence === null ? null : 'LICENSE' }
    };
}

$('#buildButton').addEventListener('click', async () => {
    const folder = state.selected;
    if (!folder) return;
    const button = $('#buildButton');
    button.disabled = true;
    show('#buildStatus', notice('', 'Building…'));
    try {
        const kept = state.stored.get(folder);
        await call(api.useStoredFiles(importerId, 'model', kept.files.map((file) => file.id)));
        const answer = await call(api.runImport(importerId, {
            step: 'build', useBinaryFiles: true, ...importOptions(folder),
            name: $('#modelName').value.trim() || robotByFolder(folder).name,
            joints: [...state.joints].map(([name, choice]) => ({ name, ...choice })),
            provenance: provenance(folder),
            timeStep: Number($('#timeStep').value) || 0.01,
            runTime: Number($('#runTime').value) || 10,
            ...servoBuildOptions()
        }));
        if (!answer.imported) throw new Error(answer.report?.errors?.join(' ') || 'The robot could not be built.');
        await call(api.openInCanvas(null, { focus: true, silent: true, session: sessionState() }));
        show('#buildStatus', notice('ok', `${answer.report.summary}, now in the canvas. Its servos' setpoints are live sliders there${$('#useMove').checked ? ', following the move until a slider is moved' : ''}.`),
            ...(answer.report.notes ?? []).map((text) => notice('', text)),
            ...(answer.report.warnings ?? []).map((text) => notice('warning', text)));
    } catch (error) {
        show('#buildStatus', notice('error', error.message));
    } finally {
        button.disabled = false;
    }
});

// ---- the session kept with the project -------------------------------------------------------------------------

function sessionState() {
    return {
        version: 1, robot: state.selected, commit: state.stored.get(state.selected)?.commit ?? null,
        joints: Object.fromEntries(state.joints), name: $('#modelName').value, timeStep: $('#timeStep').value, runTime: $('#runTime').value,
        options: Object.fromEntries(['useMove', 'useServo', 'useSupply', 'usePayload', 'supplyVoltage', 'supplyCapacity', 'payloadMass']
            .map((id) => [id, $(`#${id}`).type === 'checkbox' ? $(`#${id}`).checked : $(`#${id}`).value]))
    };
}

let keepTimer = null;
function keepSessionSoon() {
    if (!api?.keepSession || state.restoring || !state.selected) return;
    clearTimeout(keepTimer);
    keepTimer = setTimeout(() => api.keepSession(sessionState()).catch?.(() => {}), 800);
}

async function restore() {
    const answer = await api.restoreSession?.();
    const session = answer?.ok ? answer.session : null;
    if (!session?.robot) return;
    $('#timeStep').value = session.timeStep ?? '0.01';
    $('#runTime').value = session.runTime ?? '10';
    for (const [id, value] of Object.entries(session.options ?? {})) {
        const input = document.getElementById(id);
        if (!input) continue;
        if (input.type === 'checkbox') input.checked = Boolean(value); else input.value = String(value);
    }
    const kept = state.stored.get(session.robot);
    if (!kept) {
        show('#robotStatus', notice('warning', `This project was built from ${robotByFolder(session.robot)?.name ?? session.robot} (Menagerie at ${shortCommit(session.commit ?? '')}), which is not on this computer. The project carries its copy; download it here to change how it is built.`));
        return;
    }
    if (session.commit && kept.commit !== session.commit) {
        show('#robotStatus', notice('warning', `This project was built from Menagerie at ${shortCommit(session.commit)}; this computer keeps ${shortCommit(kept.commit)}.`));
    }
    state.restoring = true;
    try {
        await useRobot(session.robot, { joints: session.joints, name: session.name });
    } finally {
        state.restoring = false;
    }
}

// ---- start ---------------------------------------------------------------------------------------------------------

async function start() {
    if (!api) {
        show('#robotStatus', notice('error', 'This window runs inside Konjugate.'));
        return;
    }
    try {
        await refreshStored();
    } catch (error) {
        show('#robotStatus', notice('error', error.message));
    }
    renderRobots();
    await restore();
}

start();
