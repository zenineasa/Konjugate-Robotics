/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics window: draw a robot from solids, join them at points chosen explicitly, and build it into the canvas,
// where each solid is a node and each joint an edge. The drawing (lib/drawing.mjs) is the window's whole state worth
// keeping; it is kept with the project and restored when the window opens again.
//
// Every action has a button or field (for a newcomer), a gesture in the view (for a regular user) and a key (for a
// power user), works on the selection and can be undone, on Windows, Linux and macOS alike (docs/interaction.md).

import {
    addFrameAt, addJoint, addKeyframe, addSolid, batteryOf, checkDrawing, createDrawing, defaultMotor, driveLabels, driveModes, drivenJoints, duplicateSolids,
    frameById, groundId, jointById, jointKinds, jointLabels, keyframesOf, massOf, materials, moveSolids, nextKeyframeTime, poseOf, readDrawing, removeJoints,
    removeKeyframe, removeSolids, reverseJoint, solidById, structure, treeOrder, turnSolids, updateBattery, updateFrame, updateJoint, updateKeyframe, updateSolid
} from './lib/drawing.mjs';
import { jogRange, posedDrawing } from './lib/pose.mjs';
import { envelope, motorsOf, sizingSignals, summarize, whatIf } from './lib/sizing.mjs';
import { robotByFolder, robots, storedName } from './lib/catalog.mjs';
import { latestCommit, licenceName, modelFilesFromModel, rawUrl, repository } from './lib/menagerie.mjs';
import { exampleDrawing, examples } from './lib/examples.mjs';
import { History } from './lib/history.mjs';
import { catalogMotorOf, motorById, motorCatalog, motorFromDatasheet, servosToMotors, supplyCatalog, supplyOf } from './lib/motors.mjs';
import { simulationLimits } from './lib/modelBuilder.mjs';
import { conjugate, fromRollPitchYaw, multiply, normalizeQuaternion, subtract, toRollPitchYaw } from './lib/math.mjs';
import { addsToSelection, commandHeld, platformKeys } from './lib/platform.mjs';
import { shapeKinds, shapeLabels, sizeFields, volume } from './lib/shapes.mjs';
import { DrawingView } from './lib/view.mjs';

// Konjugate's launcher bridge, each request's refusal (an answer { ok: false, message }) turned into an error with
// Konjugate's own message, so every caller hears why a request failed rather than reading a field that is not there.
function bridge(launcher) {
    if (!launcher) return null;
    return Object.fromEntries(Object.entries(launcher).map(([name, method]) => [name, typeof method !== 'function' ? method : (...args) => {
        const result = method(...args);
        if (!result || typeof result.then !== 'function') return result;
        return result.then((answer) => {
            if (answer && answer.ok === false) throw new Error(answer.message ?? 'Konjugate refused the request.');
            return answer;
        });
    }]));
}
const api = bridge(window.konjugateLauncher ?? null);
const keys = platformKeys();
const $ = (selector, root = document) => root.querySelector(selector);
const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
const escapeHtml = (text) => String(text).replace(/[&<>"']/g, (character) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[character]);

// Lengths are shown in millimetres and angles in degrees; the drawing keeps metres and radians.
// Shown to a hundredth of a millimetre and a thousandth of a degree; typing a figure keeps exactly what is typed.
const toMillimetres = (metres) => Math.round(metres * 1e5) / 1e2;
const fromMillimetres = (millimetres) => millimetres / 1000;
const toDegrees = (radians) => Math.round((radians * 180) / Math.PI * 1000) / 1000;
const fromDegrees = (degrees) => (degrees * Math.PI) / 180;

const state = {
    history: new History(createDrawing('Robot')),
    selection: [],
    tool: 'select',
    joinFirst: null,
    hover: null,
    settings: { runTime: 10, timeStep: 0.001 },
    // The last Build: what it made (`data`), and the drawing and settings it was made from (`from`), so the status says
    // so only while the window still shows that robot. `failure`: the last Build that failed, and what it was given.
    built: null,
    failure: null,
    building: false,
    restoring: false,
    // The jog panel's pose: each driven joint's value (radians or metres from where it is drawn), whether the view shows
    // the robot posed, and the keyframe last shown.
    pose: { active: false, values: {}, keyframe: null },
    // The sizing panel: whether it is open, the run it shows (`from`: what Build was given for it; `asDrawn` and
    // `whatIf`: lib/sizing.mjs summaries), and a run under way.
    sizing: { open: false, running: false, status: '', error: null, from: null, asDrawn: null, whatIf: null },
    // Imported solids' looks by key ({ mesh, color }), from the importer: kept with the project, not in the drawing.
    looks: {},
    // The robot panel's choice of motor and supply for turning its servos into motors.
    servosToMotors: { motor: motorCatalog[0].id, supply: 'mains5' }
};
const drawing = () => state.history.current;

const view = new DrawingView($('#viewport'), {
    onClick: (pick, event) => clicked(pick, event),
    onHover: (pick) => hovered(pick),
    onTransform: (solidId, pose) => transforming(solidId, pose),
    onTransformEnd: (solidId, move) => transformed(solidId, move)
});
view.setSnapping($('#snapToggle').checked);

// ---- changing the drawing ---------------------------------------------------------------------------------------------

// Makes `next` the drawing, as one undoable step called `label`.
function change(next, label, coalesce = null) {
    state.history.change(next, label, coalesce);
    const ids = new Set([...next.solids, ...next.joints].map(({ id }) => id));
    state.selection = state.selection.filter((id) => ids.has(id));
    render();
    keepSessionSoon();
}

// Runs `action` and turns a refusal into a message where the user is looking.
function attempt(action) {
    try {
        return action();
    } catch (error) {
        toast(error.message, 'error');
        return null;
    }
}

function select(ids, { add = false } = {}) {
    const list = Array.isArray(ids) ? ids : [ids];
    if (add) {
        const next = new Set(state.selection);
        for (const id of list) { if (next.has(id)) next.delete(id); else next.add(id); }
        state.selection = [...next];
    } else {
        state.selection = list.filter(Boolean);
    }
    render();
}

const selectedSolids = () => state.selection.filter((id) => solidById(drawing(), id));
const selectedJoints = () => state.selection.filter((id) => jointById(drawing(), id));

// A free place on the floor for a new solid: along x, clear of every solid's middle by 15 cm.
function freePlace() {
    const taken = drawing().solids.map(({ position }) => position);
    for (let index = 0; ; index += 1) {
        const x = (index % 6) * 0.15;
        const y = -Math.floor(index / 6) * 0.15;
        if (!taken.some((position) => Math.hypot(position[0] - x, position[1] - y) < 0.1)) return [x, y];
    }
}

function addShape(shape) {
    const [x, y] = freePlace();
    const result = attempt(() => addSolid(drawing(), shape));
    if (!result) return;
    const solid = solidById(result.drawing, result.id);
    const placed = updateSolid(result.drawing, result.id, { position: [x, y, solid.position[2]] });
    state.selection = [result.id];
    setTool('select');
    change(placed, `Add ${solid.name}`);
}

function deleteSelection() {
    const solids = selectedSolids();
    const joints = selectedJoints();
    if (!solids.length && !joints.length) { toast('Select a solid or a joint to delete.'); return; }
    let next = drawing();
    if (joints.length) next = removeJoints(next, joints);
    if (solids.length) next = removeSolids(next, solids);
    const names = [...solids.map((id) => solidById(drawing(), id).name), ...joints.map((id) => jointById(drawing(), id).name)];
    state.selection = [];
    change(next, `Delete ${names.join(', ')}`);
}

function duplicateSelection() {
    const solids = selectedSolids();
    if (!solids.length) { toast('Select a solid to duplicate.'); return; }
    const result = duplicateSolids(drawing(), solids);
    state.selection = result.ids;
    change(result.drawing, `Duplicate ${solids.map((id) => solidById(drawing(), id).name).join(', ')}`);
}

function undo() {
    const step = state.history.undo();
    if (!step) { toast('Nothing to undo.'); return; }
    afterHistory(`Undid: ${step.label}`);
}

function redo() {
    const step = state.history.redo();
    if (!step) { toast('Nothing to redo.'); return; }
    afterHistory(`Redid: ${step.label}`);
}

function afterHistory(message) {
    const ids = new Set([...drawing().solids, ...drawing().joints].map(({ id }) => id));
    state.selection = state.selection.filter((id) => ids.has(id));
    state.joinFirst = null;
    view.showMarker('chosen', null);
    render();
    keepSessionSoon();
    toast(message);
}

// ---- tools ------------------------------------------------------------------------------------------------------------

function setTool(tool) {
    // Moving, turning and joining work on the robot as drawn.
    if (tool !== 'select') state.pose.active = false;
    state.tool = tool;
    if (tool !== 'join') { state.joinFirst = null; view.showMarker('chosen', null); }
    view.showMarker('hover', null);
    render();
}

// The solids that move with `solidId`: it and every solid further from the Ground along its joints.
function carriedBy(solidId) {
    const built = structure(drawing());
    const children = new Map();
    for (const [child, { parent }] of built.parentJoint) {
        if (!children.has(parent)) children.set(parent, []);
        children.get(parent).push(child);
    }
    const result = [];
    const visit = (id) => { for (const child of children.get(id) ?? []) { result.push(child); visit(child); } };
    visit(solidId);
    return result;
}

function clicked(pick, event) {
    if (state.tool === 'join') { joinClick(pick, event); return; }
    if (!pick) { if (!addsToSelection(event, keys)) select([]); return; }
    const id = pick.joint ?? (pick.solid === groundId ? null : pick.solid);
    if (!id) { if (!addsToSelection(event, keys)) select([]); return; }
    select(id, { add: addsToSelection(event, keys) });
}

function hovered(pick) {
    state.hover = pick;
    if (state.tool === 'join') view.showMarker('hover', pick?.solid ? pick : null);
    const canvas = $('.viewCanvas');
    if (canvas) canvas.style.cursor = state.tool === 'join' ? (pick?.solid ? 'crosshair' : 'default') : (pick?.solid && pick.solid !== groundId) || pick?.joint ? 'pointer' : 'default';
}

function joinClick(pick, event) {
    if (!pick?.solid) { toast('Click a solid, or the floor for the Ground.'); return; }
    if (!state.joinFirst) {
        state.joinFirst = pick;
        view.showMarker('chosen', pick);
        render();
        return;
    }
    const first = state.joinFirst;
    if (pick.solid === first.solid) { toast('Click a point on a different solid.'); return; }
    if (pick.solid === groundId) { toast('The Ground is the first of a joint, never the second: start the joint on the floor, then click the solid.'); return; }
    const kind = $('#joinKind').value;
    const mate = !event.shiftKey;
    const result = attempt(() => {
        const parent = addFrameAt(drawing(), first.solid, first.point, first.normal);
        const child = addFrameAt(parent.drawing, pick.solid, pick.point, pick.normal);
        return addJoint(child.drawing, parent.id, child.id, { kind, mate });
    });
    state.joinFirst = null;
    view.showMarker('chosen', null);
    view.showMarker('hover', null);
    if (!result) { render(); return; }
    const joint = jointById(result.drawing, result.id);
    state.selection = [result.id];
    change(result.drawing, `Join with ${joint.name}`);
    toast(`${joint.name}: ${nameOf(frameById(result.drawing, joint.parent).solid)} and ${nameOf(frameById(result.drawing, joint.child).solid)} joined${mate ? '' : ', where they were'}. Join another pair, or Escape to stop.`);
}

const nameOf = (solidId) => (solidId === groundId ? 'the Ground' : solidById(drawing(), solidId)?.name ?? solidId);

function transforming(solidId, pose) {
    if (state.tool === 'move') {
        const solid = solidById(drawing(), solidId);
        state.liveReadout = `${solid.name}: x ${toMillimetres(pose.position[0])}, y ${toMillimetres(pose.position[1])}, z ${toMillimetres(pose.position[2])} mm`;
    } else {
        const [roll, pitch, yaw] = toRollPitchYaw(pose.rotation).map(toDegrees);
        state.liveReadout = `${solidById(drawing(), solidId).name}: roll ${roll.toFixed(1)}°, pitch ${pitch.toFixed(1)}°, yaw ${yaw.toFixed(1)}°`;
    }
    $('#hint').textContent = state.liveReadout;
}

function transformed(solidId, move) {
    state.liveReadout = null;
    const ids = [solidId, ...carriedBy(solidId)];
    const turn = normalizeQuaternion(multiply(move.to.rotation, conjugate(move.from.rotation)));
    let next = turnSolids(drawing(), ids, turn, move.from.position);
    next = moveSolids(next, ids, subtract(move.to.position, move.from.position));
    const solid = solidById(drawing(), solidId);
    change(next, `${state.tool === 'turn' ? 'Turn' : 'Move'} ${solid.name}${ids.length > 1 ? ' and what it carries' : ''}`);
}

// ---- drawing the window -------------------------------------------------------------------------------------------------

function render() {
    const current = drawing();
    view.setDrawing(state.pose.active ? posedDrawing(current, state.pose.values) : current);
    view.setSelection(state.selection);
    const solids = selectedSolids();
    view.setTool(state.tool, solids.length === 1 ? carriedBy(solids[0]) : []);
    renderToolbar();
    renderTree();
    renderChecks();
    renderInspector();
    renderPose();
    renderHint();
    renderBuildStatus();
    renderSizing();
}

// What Build was given: the drawing and the run settings.
const buildInput = () => JSON.stringify({ drawing: drawing(), settings: state.settings });

// The status under Build describes the robot the window shows: what was built from it, why it was not built, or, when it
// has changed since (an edit, an example, a drawing opened), that the canvas still holds the robot as last built.
function renderBuildStatus() {
    if (state.building) return;
    const status = $('#buildStatus');
    const input = buildInput();
    if (state.failure?.from === input) { status.textContent = `Not built: ${state.failure.message}`; return; }
    if (!state.built) { status.textContent = ''; return; }
    const { nodes, solids, edges } = state.built.data;
    status.textContent = state.built.from === input
        ? `Built into the canvas: ${nodes} nodes (the Ground and ${solids} solids) and ${edges} relationships (the joints). Run it there; its sliders are on the joints.`
        : `The canvas holds the robot as last built (${solids} solids, ${edges} joints), not as drawn now. Build again to bring the changes in.`;
}

function renderToolbar() {
    for (const button of $$('[data-tool]')) button.setAttribute('aria-pressed', String(button.dataset.tool === state.tool));
    $('#undoButton').disabled = !state.history.canUndo;
    $('#redoButton').disabled = !state.history.canRedo;
    $('#undoButton').title = state.history.canUndo ? `Undo ${state.history.undoLabel} (${keys.undo})` : `Undo (${keys.undo})`;
    $('#redoButton').title = state.history.canRedo ? `Redo ${state.history.redoLabel} (${keys.redo})` : `Redo (${keys.redo})`;
    $('#deleteButton').disabled = !state.selection.length;
    $('#deleteButton').title = `Delete what is selected (${keys.deleteKeys})`;
    $('#duplicateButton').disabled = !selectedSolids().length;
    $('#duplicateButton').title = `Duplicate the selected solids (${keys.duplicate})`;
    $('#buildButton').title = `Build the robot into the canvas (${keys.build})`;
    $('#openButton').hidden = !api;
    if (document.activeElement !== $('#robotName')) $('#robotName').value = drawing().name;
}

function renderTree() {
    const current = drawing();
    const built = structure(current);
    const list = $('#partsTree');
    const items = ['<li class="ground" data-id="ground" role="treeitem"><span class="swatch" style="background:#7b8794"></span>Ground</li>'];
    const jointsOf = new Map();
    for (const [child, { joint }] of built.parentJoint) jointsOf.set(child, joint);
    for (const { id, depth } of treeOrder(current, built)) {
        const solid = solidById(current, id);
        const joint = jointsOf.get(id);
        const indent = `padding-left:${8 + Math.max(0, depth) * 14}px`;
        if (joint) {
            items.push(`<li class="jointItem${state.selection.includes(joint.id) ? ' selected' : ''}" data-id="${joint.id}" role="treeitem" style="${indent}">⊕ ${escapeHtml(joint.name)} <span class="kind">${jointLabels[joint.kind].toLowerCase()}</span></li>`);
        }
        items.push(`<li class="${state.selection.includes(id) ? 'selected' : ''}" data-id="${id}" role="treeitem" style="padding-left:${16 + Math.max(0, depth) * 14}px"><span class="swatch" style="background:${escapeHtml(solid.color)}"></span>${escapeHtml(solid.name)}${built.roots.includes(id) ? ' <span class="kind">floats</span>' : ''}</li>`);
    }
    for (const joint of built.loops) {
        items.push(`<li class="jointItem${state.selection.includes(joint.id) ? ' selected' : ''}" data-id="${joint.id}" role="treeitem">⊕ ${escapeHtml(joint.name)} <span class="kind">closes a loop</span></li>`);
    }
    list.innerHTML = items.join('');
}

function renderChecks() {
    const { errors, warnings, notes } = checkDrawing(drawing());
    const items = [
        ...errors.map((item) => ({ ...item, level: 'error' })),
        ...warnings.map((item) => ({ ...item, level: 'warning' })),
        ...notes.map((item) => ({ ...item, level: 'note' }))
    ];
    // What the drawing may hold but Build cannot simulate yet: a loop closed by a fixed joint, and a second loop.
    // Each says why it cannot be built, in place of what the drawing would otherwise say of that joint.
    const limits = errors.length ? [] : simulationLimits(drawing());
    const limited = new Set(limits.map(({ id }) => id));
    items.splice(0, items.length, ...limits.map((item) => ({ ...item, level: 'error' })), ...items.filter((item) => item.level === 'error' || !limited.has(item.id)));
    if (!items.length) items.push({ level: 'ok', message: 'Ready to build.' });
    $('#checksList').innerHTML = items.map((item) => `<li class="${item.level}"${item.id ? ` data-id="${item.id}"` : ''}>${escapeHtml(item.message)}</li>`).join('');
    $('#buildButton').disabled = errors.length + limits.length > 0;
}

function renderHint() {
    if (state.liveReadout) return;
    const current = drawing();
    let hint = '';
    if (state.tool === 'join') {
        hint = state.joinFirst
            ? `Now click the point on the second solid. It turns and moves so the two points meet, faces together. Hold Shift as you click to leave it where it is. Escape stops.`
            : `Join: click the point on the first solid (or on the floor, for the Ground) where the ${jointLabels[$('#joinKind').value].toLowerCase()} goes. Its axis is the arrow, straight out of the surface; points snap to face centres and edge middles (green).`;
    } else if (state.pose.active) {
        hint = `Posed by the jog sliders${state.pose.keyframe ? `, as the keyframe at ${keyframesOf(current).find(({ id }) => id === state.pose.keyframe)?.time ?? '?'} s` : ''}: the drawing is unchanged. Add a keyframe (K), or go back to the robot as drawn (Escape).`;
    } else if (!current.solids.length) {
        hint = 'Start with a solid: Box (B), Cylinder (C), Sphere (S) or Capsule (P). Then join solids with Join (J).';
    } else if (state.tool === 'move' || state.tool === 'turn') {
        const solids = selectedSolids();
        hint = solids.length === 1
            ? `Drag the handles to ${state.tool} ${solidById(current, solids[0]).name}${carriedBy(solids[0]).length ? ' and the solids it carries' : ''}. Or type its place and turn on the right.`
            : `Select one solid to ${state.tool} it.`;
    } else if (!current.joints.length && current.solids.length > 1) {
        hint = 'Join two solids: Join (J), then click a point on each. Start on the floor to fix a solid to the Ground.';
    } else if (!state.selection.length) {
        hint = `Click a solid or a joint to change it. Drag to turn the view, right-drag to move it, scroll to zoom. ${keys.addClick} to select several.`;
    }
    $('#hint').textContent = hint;
}

// ---- the inspector ----------------------------------------------------------------------------------------------------

function field(label, input, unit = '') {
    return `<label class="field"><span>${label}</span><span class="row">${input}${unit ? ` <span class="unit">${unit}</span>` : ''}</span></label>`;
}

const numberInput = (name, value, { step = 'any', min = null, title = '' } = {}) =>
    `<input type="number" name="${name}" value="${value}" step="${step}"${min !== null ? ` min="${min}"` : ''}${title ? ` title="${escapeHtml(title)}"` : ''}>`;

function renderInspector() {
    const panel = $('#inspector');
    // Leave a field being typed in alone; it is redrawn once the change lands.
    if (panel.contains(document.activeElement) && document.activeElement.matches('input[type="number"], input[type="text"]') && panel.dataset.for === state.selection.join(',')) return;
    panel.dataset.for = state.selection.join(',');
    const current = drawing();
    const solids = selectedSolids();
    const joints = selectedJoints();
    if (solids.length === 1 && !joints.length) { panel.innerHTML = solidInspector(solidById(current, solids[0])); return; }
    if (joints.length === 1 && !solids.length) { panel.innerHTML = jointInspector(jointById(current, joints[0])); return; }
    if (state.selection.length > 1) {
        panel.innerHTML = `<h3>${state.selection.length} selected</h3><p class="note">Delete or duplicate them together, or select one to change it.</p>
            <div class="buttons"><button type="button" class="tool" data-action="duplicate">Duplicate</button><button type="button" class="tool" data-action="delete">Delete</button></div>`;
        return;
    }
    const built = structure(current);
    panel.innerHTML = `<h3>${escapeHtml(current.name || 'Robot')}</h3>
        <p class="note">${current.solids.length} solids, ${current.joints.length} joints${built.loops.length ? `, ${built.loops.length} closing a loop` : ''}.</p>
        <p class="note">Select a solid or a joint to change it. Build turns the drawing into the canvas: each solid a node, each joint a relationship between two of them. The joints' equations move the robot; nothing outside the canvas solves it.</p>
        <h2>Run</h2>
        ${field('Run time', numberInput('runTime', state.settings.runTime, { min: 0.01 }), 's')}
        ${field('Step', numberInput('timeStep', state.settings.timeStep * 1000, { min: 0.1, step: 0.1, title: 'The engine\'s step. Stiff servos and light solids need a short one; a millisecond suits most drawings.' }), 'ms')}
        ${current.joints.some((joint) => joint.drive?.mode === 'motor') ? batteryFields(current) : ''}
        ${servoFields(current)}`;
}

// Turning the robot's servos into catalog motors, to size them: shown while a hinge has a servo.
function servoFields(current) {
    const servos = drivenJoints(current).filter((joint) => joint.drive.mode === 'servo' && joint.kind === 'hinge');
    if (!servos.length) return '';
    const choice = state.servosToMotors;
    const entry = motorById(choice.motor);
    return `<h2>Servos to motors</h2>
        ${field('Motor', `<select name="servosToMotors.motor">${motorCatalog.filter(({ kind }) => kind === 'hinge').map((item) => `<option value="${item.id}"${item.id === choice.motor ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}</select>`)}
        ${field('On', `<select name="servosToMotors.supply"><option value=""${choice.supply ? '' : ' selected'}>The battery as it is</option>${supplyCatalog.map((item) => `<option value="${item.id}"${item.id === choice.supply ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}</select>`)}
        <p class="note">${escapeHtml(entry.what)} Its ${servos.length} servo${servos.length === 1 ? '' : 's'} become${servos.length === 1 ? 's' : ''} this motor, each holding as stiffly and with the damping it had, so Size can say whether the real ones will do it.</p>
        <div class="buttons"><button type="button" class="tool" data-action="servosToMotors">Make them motors</button></div>`;
}

// The battery every motor draws from.
function batteryFields(current) {
    const battery = batteryOf(current);
    const supply = supplyOf(battery);
    return `<h2>Battery</h2>
        ${field('Supply', `<select name="battery.supply"><option value=""${supply ? '' : ' selected'}>My own</option>${supplyCatalog.map((item) => `<option value="${item.id}"${supply?.id === item.id ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}</select>`)}
        ${field('Full', numberInput('battery.fullVoltage', battery.fullVoltage, { min: 0 }), 'V')}
        ${field('Empty', numberInput('battery.emptyVoltage', battery.emptyVoltage, { min: 0 }), 'V')}
        ${field('Capacity', numberInput('battery.capacity', Math.round((battery.capacity / 3600) * 1000) / 1000, { min: 0 }), 'Ah')}
        <p class="note">Every motor draws from it: its voltage falls in a straight line from full to empty with its charge, a node in the canvas. A supply on the mains is full and empty at the same voltage.</p>`;
}

function solidInspector(solid) {
    const pose = poseOf(drawing(), solid.id);
    const [roll, pitch, yaw] = toRollPitchYaw(pose.rotation).map(toDegrees);
    const material = solid.mass.by === 'mass' ? 'mass' : solid.mass.material ?? 'other';
    const imported = solid.shape === 'mesh';
    const sizeInputs = imported ? '' : sizeFields[solid.shape].map(([key, label]) => field(label, numberInput(`size.${key}`, toMillimetres(solid.size[key]), { min: 0.001 }), 'mm')).join('');
    const frames = drawing().frames.filter((frame) => frame.solid === solid.id);
    const inertiaFrom = { inertial: 'as the model gives it', geoms: 'worked out from its geoms, as MuJoCo works it out', meshes: 'worked out from its meshes\' volumes' }[solid.source?.inertia] ?? '';
    const shapeField = imported
        ? `<p class="note">Imported from ${solid.source?.body ? `body ${escapeHtml(solid.source.body)} of ` : ''}a model: its look is the model's, its place its centre of mass and its turn its axes of inertia.</p>`
        : field('Shape', `<select name="shape">${shapeKinds.map((kind) => `<option value="${kind}"${kind === solid.shape ? ' selected' : ''}>${shapeLabels[kind]}</option>`).join('')}</select>`);
    const massFields = imported
        ? `${field('Mass', numberInput('massValue', solid.mass.value, { min: 0 }), 'kg')}
        <p class="note">Its inertia about its axes is ${solid.unitInertia.map((value) => (value * massOf(solid) * 1e4).toPrecision(3)).join(', ')} kg·cm²${inertiaFrom ? `, ${inertiaFrom}` : ''}; it grows with its mass.</p>`
        : `${field('Made of', `<select name="material">${materials.map((item) => `<option value="${item.id}"${item.id === material ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}<option value="other"${material === 'other' ? ' selected' : ''}>Another density</option><option value="mass"${material === 'mass' ? ' selected' : ''}>A mass I give</option></select>`)}
        ${solid.mass.by === 'mass'
        ? field('Mass', numberInput('massValue', solid.mass.value, { min: 0 }), 'kg')
        : field('Density', numberInput('density', solid.mass.density, { min: 0, step: 10 }), 'kg/m³')}
        <p class="note">${solid.mass.by === 'mass' ? `Its density works out at ${Math.round(solid.mass.value / volume(solid.shape, solid.size))} kg/m³.` : `Its mass works out at ${massOf(solid).toFixed(3)} kg, from its density and its ${(volume(solid.shape, solid.size) * 1e6).toFixed(1)} cm³.`}</p>`;
    return `<h3>${escapeHtml(solid.name)}</h3>
        ${field('Name', `<input type="text" name="name" value="${escapeHtml(solid.name)}" spellcheck="false">`)}
        ${shapeField}
        ${sizeInputs}
        <h2>Place</h2>
        <div class="fieldRow"><span>Position</span>${['x', 'y', 'z'].map((axis, index) => `<label>${axis}${numberInput(`position.${index}`, toMillimetres(pose.position[index]), { step: 1 })}</label>`).join('')}</div>
        <p class="unit">Millimetres, from the middle of the floor; z is up.</p>
        <div class="fieldRow"><span>Turn</span>${[['roll', roll], ['pitch', pitch], ['yaw', yaw]].map(([name, value]) => `<label>${name[0]}${numberInput(`turn.${name}`, value, { step: 5 })}</label>`).join('')}</div>
        <p class="unit">Degrees: yaw about z, then pitch about y, then roll about x.</p>
        <h2>Mass</h2>
        ${massFields}
        ${field('Colour', `<input type="color" name="color" value="${escapeHtml(solid.color)}">`)}
        ${frames.length ? `<p class="note">${frames.length} joint point${frames.length === 1 ? '' : 's'} on it, which move with it.</p>` : ''}
        <div class="buttons">
            <button type="button" class="tool" data-action="move">Move</button>
            <button type="button" class="tool" data-action="turn">Turn</button>
            <button type="button" class="tool" data-action="join">Join it…</button>
            <button type="button" class="tool" data-action="duplicate">Duplicate</button>
            <button type="button" class="tool" data-action="delete">Delete</button>
        </div>`;
}

// A motor's fields: its setpoint and controller gains, then the motor and gearbox.
function motorFieldsOf(joint, show, unit) {
    const linear = joint.kind === 'slider';
    const drive = joint.drive;
    const motor = { ...defaultMotor(joint.kind), ...drive.motor };
    const listed = catalogMotorOf(motor);
    const choices = motorCatalog.filter(({ kind }) => kind === joint.kind);
    return [
        field('Setpoint', numberInput('drive.setpoint', show(drive.setpoint ?? 0)), unit),
        field('Proportional', numberInput('drive.proportional', drive.proportional ?? 0, { min: 0, title: 'Duty cycle per unit of the setpoint left to reach' }), linear ? '/m' : '/rad'),
        field('Derivative', numberInput('drive.derivative', drive.derivative ?? 0, { min: 0, title: 'Duty cycle taken off per unit of the joint\'s speed' }), linear ? 's/m' : 's/rad'),
        '<h2>Motor</h2>',
        ...(choices.length ? [field('Motor', `<select name="motor.catalog"><option value=""${listed ? '' : ' selected'}>My own</option>${choices.map((item) => `<option value="${item.id}"${listed?.id === item.id ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}</select>`)] : []),
        ...(listed ? [`<p class="note">${escapeHtml(listed.what)} From its maker: ${escapeHtml(listed.source)}.</p>`] : []),
        field('Winding resistance', numberInput('motor.resistance', motor.resistance, { min: 0 }), 'Ω'),
        field('Torque constant', numberInput('motor.torqueConstant', motor.torqueConstant, { min: 0, title: 'Torque per ampere at the motor, and back-EMF volts per radian per second' }), 'N·m/A'),
        field('Gear ratio', numberInput('motor.gearRatio', motor.gearRatio, { min: 0, title: linear ? 'Motor radians per metre of travel, as a lead screw\'s' : 'Motor turns per joint turn' }), linear ? 'rad/m' : ':1'),
        field('Gear efficiency', numberInput('motor.gearEfficiency', Math.round(motor.gearEfficiency * 1e4) / 100, { min: 0, title: 'The share of the motor\'s torque its gears pass on to the joint' }), '%'),
        field('Rotor inertia', numberInput('motor.rotorInertia', motor.rotorInertia, { min: 0 }), 'kg·m²'),
        field('Thermal resistance', numberInput('motor.thermalResistance', motor.thermalResistance, { min: 0, title: 'From the winding to the air around it' }), 'K/W'),
        field('Heat capacity', numberInput('motor.thermalCapacitance', motor.thermalCapacitance, { min: 0, title: 'Of the winding' }), 'J/K'),
        field('Ambient', numberInput('motor.ambientTemperature', Math.round((motor.ambientTemperature - 273.15) * 100) / 100), '°C'),
        field('Hottest winding', numberInput('motor.windingLimit', Math.round((motor.windingLimit - 273.15) * 100) / 100, { title: 'The hottest its winding may get (its insulation class): Size holds the run against it' }), '°C')
    ];
}

// A ball joint's motion: its most turn, its damping, friction and armature (each axis alike) and its drive, a torque
// about each of its parent's axes or a servo to a turn about them.
function ballMotion(joint) {
    const drive = joint.drive ?? { mode: 'free' };
    const vector = (name, values, unit, shown) => `<div class="fieldRow"><span>${name === 'drive.torque' ? 'Torque' : 'Setpoint turn'}</span>${['x', 'y', 'z'].map((axis, index) => `<label>${axis}${numberInput(`${name}.${index}`, shown((values ?? [0, 0, 0])[index] ?? 0), { step: name === 'drive.torque' ? 0.01 : 1 })}</label>`).join('')}</div><p class="unit">${unit}, about ${escapeHtml(nameOf(frameById(drawing(), joint.parent).solid))}'s own axes.</p>`;
    const fields = [field('Driven by', `<select name="drive.mode">${driveModes.filter((mode) => mode !== 'motor').map((mode) => `<option value="${mode}"${mode === drive.mode ? ' selected' : ''}>${driveLabels[mode]}</option>`).join('')}</select>`)];
    if (drive.mode === 'torque') fields.push(vector('drive.torque', drive.torque, 'N·m', (value) => value));
    if (drive.mode === 'servo') {
        fields.push(vector('drive.setpoint', drive.setpoint, 'Degrees: a turn about the direction given, by its length', toDegrees));
        fields.push(field('Stiffness', numberInput('drive.gain', drive.gain, { min: 0 }), 'N·m/rad'));
        fields.push(field('Most torque', numberInput('drive.torqueLimit', drive.torqueLimit, { min: 0 }), 'N·m'));
    }
    fields.push(`<p class="note">${{
        free: 'Nothing drives it: it turns freely about its point every way, as gravity and the rest of the robot turn it.',
        torque: 'A constant torque about each axis, each a live slider on the relationship in the canvas.',
        servo: 'A servo turns it to its setpoint, each axis of the turn a live slider on the relationship in the canvas, pushing with up to its most torque.'
    }[drive.mode]}</p>`);
    return `<h2>Motion</h2>
        <label class="field"><span>Limited</span><span><input type="checkbox" name="limited"${joint.limits ? ' checked' : ''}></span></label>
        ${joint.limits ? field('Most turn', numberInput('limits.upper', toDegrees(joint.limits.upper)), '°') : ''}
        ${field('Damping', numberInput('damping', joint.damping, { min: 0 }), 'N·m·s/rad')}
        ${field('Friction', numberInput('friction', joint.friction, { min: 0 }), 'N·m')}
        ${field('Armature', numberInput('armature', joint.armature, { min: 0, title: 'The inertia of what turns with the joint inside it' }), 'kg·m²')}
        ${fields.join('')}`;
}

function jointInspector(joint) {
    const current = drawing();
    const parentFrame = frameById(current, joint.parent);
    const childFrame = frameById(current, joint.child);
    const built = structure(current);
    const loop = built.loops.some(({ id }) => id === joint.id);
    const linear = joint.kind === 'slider';
    const moving = joint.kind === 'hinge' || joint.kind === 'slider';
    const ball = joint.kind === 'ball';
    const unit = linear ? 'mm' : '°';
    const show = (value) => (linear ? toMillimetres(value) : toDegrees(value));
    const anchor = parentFrame.point;
    const axis = parentFrame.axis;
    const drive = joint.drive ?? { mode: 'free' };
    const driveFields = [];
    if (moving && !loop) {
        driveFields.push(field('Driven by', `<select name="drive.mode">${driveModes.map((mode) => `<option value="${mode}"${mode === drive.mode ? ' selected' : ''}>${driveLabels[mode]}</option>`).join('')}</select>`));
        if (drive.mode === 'torque') driveFields.push(field(linear ? 'Force' : 'Torque', numberInput('drive.torque', drive.torque ?? 0), linear ? 'N' : 'N·m'));
        if (drive.mode === 'servo') {
            driveFields.push(field('Setpoint', numberInput('drive.setpoint', show(drive.setpoint ?? 0)), unit));
            driveFields.push(field('Stiffness', numberInput('drive.gain', drive.gain, { min: 0 }), linear ? 'N/m' : 'N·m/rad'));
            driveFields.push(field(linear ? 'Most force' : 'Most torque', numberInput('drive.torqueLimit', drive.torqueLimit, { min: 0 }), linear ? 'N' : 'N·m'));
            driveFields.push(field('Servo damping', numberInput('drive.damping', drive.damping ?? 0, { min: 0, title: 'Its own damping, within its most torque, as a real servo\'s' }), linear ? 'N·s/m' : 'N·m·s/rad'));
        }
        if (drive.mode === 'motor') driveFields.push(...motorFieldsOf(joint, show, unit));
        driveFields.push(`<p class="note">${{
            free: 'Nothing drives it: it moves only as gravity and the rest of the robot move it.',
            torque: `A constant ${linear ? 'force' : 'torque'}, a live slider on the relationship in the canvas. Wire a motor to it there to drive it from a motor instead.`,
            servo: 'A position servo holds it at its setpoint, a live slider on the relationship in the canvas, pushing with up to its most torque.',
            motor: `A DC motor and gearbox on the robot's battery turn it towards its setpoint: its controller sets the duty cycle from how far it has to go and how fast it is going. Build adds the motor and the battery as nodes in the canvas, with the motor's current, ${linear ? 'force' : 'torque'} and winding temperature and the battery's charge.`
        }[drive.mode]}</p>`);
    }
    return `<h3>${escapeHtml(joint.name)}</h3>
        ${field('Name', `<input type="text" name="name" value="${escapeHtml(joint.name)}" spellcheck="false">`)}
        ${field('Kind', `<select name="kind">${jointKinds.map((kind) => `<option value="${kind}"${kind === joint.kind ? ' selected' : ''}>${jointLabels[kind]}</option>`).join('')}</select>`)}
        <p class="note">Joins <strong>${escapeHtml(nameOf(parentFrame.solid))}</strong> (its point and axis) to <strong>${escapeHtml(nameOf(childFrame.solid))}</strong>.${loop ? ' It closes a loop: Build holds it with a force of its own, on its own relationship in the canvas; the force, the gap and the twist left are states of the solid it holds.' : ''}</p>
        <h2>Point and axis</h2>
        <div class="fieldRow"><span>Point</span>${['x', 'y', 'z'].map((name, index) => `<label>${name}${numberInput(`anchor.${index}`, toMillimetres(anchor[index]), { step: 1 })}</label>`).join('')}</div>
        <div class="fieldRow"><span>Axis</span>${['x', 'y', 'z'].map((name, index) => `<label>${name}${numberInput(`axis.${index}`, Math.round(axis[index] * 1000) / 1000, { step: 0.1 })}</label>`).join('')}</div>
        <p class="unit">In ${escapeHtml(nameOf(parentFrame.solid))}'s own frame${parentFrame.solid === groundId ? ' (the world)' : ''}: millimetres from its middle, and a direction.</p>
        <div class="buttons">
            <button type="button" class="tool" data-action="flipAxis" title="Point the axis the other way">Flip axis</button>
            ${parentFrame.solid !== groundId ? '<button type="button" class="tool" data-action="reverse" title="Use the other solid\'s point and axis">Swap solids</button>' : ''}
        </div>
        ${moving && !loop ? `<h2>Motion</h2>
        <label class="field"><span>Limited</span><span><input type="checkbox" name="limited"${joint.limits ? ' checked' : ''}></span></label>
        ${joint.limits ? `${field('From', numberInput('limits.lower', show(joint.limits.lower)), unit)}${field('To', numberInput('limits.upper', show(joint.limits.upper)), unit)}` : ''}
        ${field('Damping', numberInput('damping', joint.damping, { min: 0 }), linear ? 'N·s/m' : 'N·m·s/rad')}
        ${field('Friction', numberInput('friction', joint.friction, { min: 0 }), linear ? 'N' : 'N·m')}
        ${field('Armature', numberInput('armature', joint.armature, { min: 0, title: 'The inertia of what turns with the joint inside it, such as a motor\'s rotor through its gearbox' }), linear ? 'kg' : 'kg·m²')}
        ${driveFields.join('')}` : ''}
        ${ball && !loop ? ballMotion(joint) : ''}
        <div class="buttons"><button type="button" class="tool" data-action="delete">Delete joint</button></div>`;
}

// One field of the inspector changed: apply it as one step (typing in one field is one step).
function inspectorChanged(input) {
    const current = drawing();
    const name = input.name;
    const value = input.type === 'checkbox' ? input.checked : input.value;
    const number = Number(value);
    if (input.type === 'number' && !Number.isFinite(number)) { toast('Type a number.'); render(); return; }
    const solids = selectedSolids();
    const joints = selectedJoints();
    if (!state.selection.length) {
        if (name === 'servosToMotors.motor' || name === 'servosToMotors.supply') {
            state.servosToMotors[name.slice(15)] = value;
            render();
            return;
        }
        if (name === 'battery.supply') {
            const supply = supplyCatalog.find(({ id }) => id === value);
            if (supply) change(updateBattery(current, { fullVoltage: supply.fullVoltage, emptyVoltage: supply.emptyVoltage, capacity: supply.capacity }), 'Battery');
            else render();
            return;
        }
        if (name.startsWith('battery.')) {
            const field = name.slice(8);
            if (!(number > 0)) { toast('This must be more than zero.'); render(); return; }
            change(updateBattery(current, { [field]: field === 'capacity' ? number * 3600 : number }), 'Battery', `battery:${field}`);
            return;
        }
        if (name === 'runTime' && number > 0) state.settings.runTime = number;
        if (name === 'timeStep' && number >= 0.1) state.settings.timeStep = number / 1000;
        keepSessionSoon();
        renderBuildStatus();
        return;
    }
    if (solids.length === 1) {
        const solid = solidById(current, solids[0]);
        const coalesce = `${solid.id}:${name}`;
        const apply = (changes, label) => { const next = attempt(() => updateSolid(current, solid.id, changes)); if (next) change(next, `${label} of ${solid.name}`, coalesce); };
        if (name === 'name') { if (value.trim()) apply({ name: value.trim() }, 'Name'); else render(); return; }
        if (name === 'shape') { apply({ shape: value }, 'Shape'); return; }
        if (name === 'color') { apply({ color: value }, 'Colour'); return; }
        if (name.startsWith('size.')) {
            if (!(number > 0)) { toast('A size must be more than zero.'); render(); return; }
            apply({ size: { [name.slice(5)]: fromMillimetres(number) } }, 'Size');
            return;
        }
        if (name.startsWith('position.')) {
            const position = [...solid.position];
            position[Number(name.slice(9))] = fromMillimetres(number);
            const ids = [solid.id, ...carriedBy(solid.id)];
            change(moveSolids(current, ids, subtract(position, solid.position)), `Move ${solid.name}`, coalesce);
            return;
        }
        if (name.startsWith('turn.')) {
            const angles = toRollPitchYaw(solid.rotation);
            angles[['roll', 'pitch', 'yaw'].indexOf(name.slice(5))] = fromDegrees(number);
            const rotation = fromRollPitchYaw(angles);
            const turn = normalizeQuaternion(multiply(rotation, conjugate(solid.rotation)));
            change(turnSolids(current, [solid.id, ...carriedBy(solid.id)], turn, solid.position), `Turn ${solid.name}`, coalesce);
            return;
        }
        if (name === 'material') {
            if (value === 'mass') apply({ mass: { by: 'mass', value: Math.round(massOf(solid) * 1000) / 1000 } }, 'Mass');
            else if (value === 'other') apply({ mass: { by: 'density', density: solid.mass.density ?? Math.round(massOf(solid) / volume(solid.shape, solid.size)) } }, 'Density');
            else apply({ mass: { by: 'density', density: materials.find(({ id }) => id === value).density, material: value } }, 'Material');
            return;
        }
        if (name === 'density') { if (number > 0) apply({ mass: { by: 'density', density: number } }, 'Density'); else { toast('A density must be more than zero.'); render(); } return; }
        if (name === 'massValue') { if (number > 0) apply({ mass: { by: 'mass', value: number } }, 'Mass'); else { toast('A mass must be more than zero.'); render(); } return; }
        return;
    }
    if (joints.length === 1) {
        const joint = jointById(current, joints[0]);
        const coalesce = `${joint.id}:${name}`;
        const linear = joint.kind === 'slider';
        const fromShown = (shown) => (linear ? fromMillimetres(shown) : fromDegrees(shown));
        const apply = (changes, label) => { const next = attempt(() => updateJoint(current, joint.id, changes)); if (next) change(next, `${label} of ${joint.name}`, coalesce); };
        if (name === 'name') { if (value.trim()) apply({ name: value.trim() }, 'Name'); else render(); return; }
        if (name === 'kind') { apply({ kind: value }, 'Kind'); return; }
        if (name === 'limited') {
            const limits = joint.kind === 'ball' ? { lower: 0, upper: Math.PI / 4 } : linear ? { lower: -0.05, upper: 0.05 } : { lower: -Math.PI / 2, upper: Math.PI / 2 };
            apply({ limits: value ? limits : null }, 'Limits');
            return;
        }
        if (joint.kind === 'ball' && name === 'limits.upper') {
            if (!(number > 0)) { toast('A most turn must be more than zero.'); render(); return; }
            apply({ limits: { lower: 0, upper: fromDegrees(number) } }, 'Most turn');
            return;
        }
        if (/^drive\.(torque|setpoint)\.[012]$/.test(name)) {
            const [, field, index] = name.split('.');
            const values = [...(Array.isArray(joint.drive?.[field]) ? joint.drive[field] : [0, 0, 0])];
            values[Number(index)] = field === 'setpoint' ? fromDegrees(number) : number;
            apply({ drive: { [field]: values } }, field === 'setpoint' ? 'Setpoint' : 'Torque');
            return;
        }
        if (name === 'limits.lower' || name === 'limits.upper') { apply({ limits: { ...joint.limits, [name.slice(7)]: fromShown(number) } }, 'Limits'); return; }
        if (['damping', 'friction', 'armature'].includes(name)) {
            if (number < 0) { toast('This must be zero or more.'); render(); return; }
            apply({ [name]: number }, name[0].toUpperCase() + name.slice(1));
            return;
        }
        if (name === 'drive.mode') { apply({ drive: { mode: value } }, 'Drive'); return; }
        if (name === 'drive.proportional' || name === 'drive.derivative') {
            if (number < 0) { toast('A gain must be zero or more.'); render(); return; }
            apply({ drive: { [name.slice(6)]: number } }, name === 'drive.proportional' ? 'Proportional gain' : 'Derivative gain');
            return;
        }
        if (name === 'motor.catalog') {
            const entry = motorById(value);
            if (entry) apply({ drive: { motor: motorFromDatasheet(entry) } }, 'Motor');
            else render();
            return;
        }
        if (name.startsWith('motor.')) {
            const field = name.slice(6);
            const amount = field === 'ambientTemperature' || field === 'windingLimit' ? number + 273.15 : field === 'gearEfficiency' ? number / 100 : number;
            if (field === 'rotorInertia' ? amount < 0 : !(amount > 0)) { toast(field === 'rotorInertia' ? 'This must be zero or more.' : 'This must be more than zero.'); render(); return; }
            if (field === 'gearEfficiency' && amount > 1) { toast('A gear efficiency is at most 100%.'); render(); return; }
            apply({ drive: { motor: { [field]: amount } } }, 'Motor');
            return;
        }
        if (name === 'drive.torque') { apply({ drive: { torque: number } }, linear ? 'Force' : 'Torque'); return; }
        if (name === 'drive.setpoint') { apply({ drive: { setpoint: fromShown(number) } }, 'Setpoint'); return; }
        if (name === 'drive.damping') {
            if (!(number >= 0)) { toast('This must be zero or more.'); render(); return; }
            apply({ drive: { damping: number } }, 'Servo damping');
            return;
        }
        if (name === 'drive.gain' || name === 'drive.torqueLimit') {
            if (!(number > 0)) { toast('This must be more than zero.'); render(); return; }
            apply({ drive: { [name.slice(6)]: number } }, name === 'drive.gain' ? 'Servo stiffness' : 'Servo limit');
            return;
        }
        if (name.startsWith('anchor.') || name.startsWith('axis.')) {
            const frame = frameById(current, joint.parent);
            if (name.startsWith('anchor.')) {
                const point = [...frame.point];
                point[Number(name.slice(7))] = fromMillimetres(number);
                change(updateFrame(current, frame.id, { point }), `Point of ${joint.name}`, coalesce);
            } else {
                const axis = [...frame.axis];
                axis[Number(name.slice(5))] = number;
                const next = attempt(() => updateFrame(current, frame.id, { axis }));
                if (next) change(next, `Axis of ${joint.name}`, coalesce);
                else render();
            }
        }
    }
}

function inspectorAction(action) {
    const current = drawing();
    if (action === 'delete') { deleteSelection(); return; }
    if (action === 'duplicate') { duplicateSelection(); return; }
    if (action === 'move' || action === 'turn') { setTool(action); return; }
    if (action === 'join') { setTool('join'); return; }
    if (action === 'servosToMotors') { makeServosMotors(); return; }
    const joint = jointById(current, selectedJoints()[0]);
    if (!joint) return;
    if (action === 'flipAxis') {
        const frame = frameById(current, joint.parent);
        change(updateFrame(current, frame.id, { axis: frame.axis.map((value) => -value) }), `Flip the axis of ${joint.name}`);
    }
    if (action === 'reverse') {
        const next = attempt(() => reverseJoint(current, joint.id));
        if (next) change(next, `Swap the solids of ${joint.name}`);
    }
}

// Every servo on a hinge becomes the chosen catalog motor, on the chosen supply, in one step to undo.
function makeServosMotors() {
    const { motor, supply } = state.servosToMotors;
    const done = attempt(() => servosToMotors(drawing(), motor, supply || null));
    if (!done) return;
    if (!done.turned.length) { toast('There is no servo on a hinge to make a motor.'); return; }
    change(done.drawing, `Make the servos ${done.motor.name}`);
    const weaker = done.turned.filter(({ torqueLimit, stallTorque }) => stallTorque < torqueLimit);
    const figure = (value) => String(Number(value.toPrecision(3)));
    toast(`${done.turned.length} servo${done.turned.length === 1 ? ' is' : 's are'} ${done.motor.name} now, on ${figure(done.voltage)} V: each gives up to ${figure(done.turned[0].stallTorque)} N·m held still${weaker.length ? `, less than the most torque ${weaker.length === done.turned.length ? 'they' : weaker.map(({ name }) => name).join(', ')} had` : ''}.${done.left.length ? ` ${done.left.join(', ')} ${done.left.length === 1 ? 'is a slider\'s servo, left' : 'are sliders\' servos, left'} as ${done.left.length === 1 ? 'it was' : 'they were'}.` : ''}`);
}

// ---- messages ---------------------------------------------------------------------------------------------------------

let toastTimer = null;
function toast(message, level = 'info') {
    const element = $('#toast');
    element.textContent = message;
    element.dataset.level = level;
    element.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { element.hidden = true; }, level === 'error' ? 6000 : 3500);
}

// ---- the session and building -----------------------------------------------------------------------------------------

let keepTimer = null;
function keepSessionSoon() {
    if (!api?.keepSession || state.restoring) return;
    clearTimeout(keepTimer);
    keepTimer = setTimeout(() => { api.keepSession(sessionState()).catch?.(() => {}); }, 700);
}

const sessionState = () => ({ drawing: drawing(), settings: state.settings });

async function restore() {
    if (!api?.restoreSession) { render(); view.fit(); return; }
    state.restoring = true;
    try {
        const answer = await api.restoreSession();
        if (answer?.session?.drawing) {
            state.history.reset(readDrawing(answer.session.drawing));
            if (answer.session.settings) state.settings = { ...state.settings, ...answer.session.settings };
        }
    } catch (error) {
        toast(`The drawing kept with this project could not be read: ${error.message}`, 'error');
    } finally {
        state.restoring = false;
    }
    render();
    view.fit();
}

async function build() {
    const { errors } = checkDrawing(drawing());
    const problems = [...errors, ...(errors.length ? [] : simulationLimits(drawing()))];
    if (problems.length) { toast(problems[0].message, 'error'); return; }
    if (!api) { toast('Building needs the Robotics window opened from Konjugate.', 'error'); return; }
    const status = $('#buildStatus');
    const from = buildInput();
    state.building = true;
    $('#buildButton').disabled = true;
    status.textContent = 'Building…';
    try {
        const answer = await api.runImport('drawing', { step: 'build', drawing: drawing(), runTime: state.settings.runTime, timeStep: state.settings.timeStep });
        if (!answer.imported) throw new Error((answer.report?.errors ?? ['The model could not be built.']).join(' '));
        await api.openInCanvas(null, { focus: true, silent: true, session: sessionState() });
        state.built = { data: answer.data, from };
        importedFrom = from;
        state.failure = null;
        toast('Built into the canvas.');
    } catch (error) {
        state.failure = { message: error.message, from };
        toast(error.message, 'error');
    } finally {
        state.building = false;
        renderChecks();
        renderBuildStatus();
    }
}

async function openDrawing() {
    if (!api?.chooseFile) return;
    try {
        const chosen = await api.chooseFile('drawing', 'drawing');
        if (!chosen?.chosen) return;
        const answer = await api.runImport('drawing', { step: 'open' });
        // Read once: the drawing lives in the window from now on, not as a file kept with the project.
        await api.clearFile?.('drawing', 'drawing', chosen.name);
        if (!answer?.data?.drawing) throw new Error((answer?.report?.errors ?? ['The drawing could not be read.']).join(' '));
        state.selection = [];
        forgetPose();
        change(readDrawing(answer.data.drawing), `Open ${answer.data.name}`);
        view.fit();
    } catch (error) {
        toast(error.message, 'error');
    }
}

// ---- the jog panel and the keyframes ------------------------------------------------------------------------------------

const jogShown = (joint, value) => (joint.kind === 'slider' ? `${toMillimetres(value)} mm` : `${toDegrees(value)}°`);
const poseValues = () => Object.fromEntries(drivenJoints(drawing()).map((joint) => [joint.id, state.pose.values[joint.id] ?? 0]));

function renderPose() {
    const panel = $('#posePanel');
    if (panel.contains(document.activeElement) && document.activeElement.matches('input[type="number"]')) return;
    const current = drawing();
    const driven = drivenJoints(current);
    if (!driven.length) {
        panel.innerHTML = '<p class="note">Drive a hinge or a slider by a servo or a motor to pose it here and keyframe its moves.</p>';
        return;
    }
    const keyframes = keyframesOf(current);
    panel.innerHTML = `${driven.map((joint) => {
        const range = jogRange(joint);
        const value = state.pose.values[joint.id] ?? 0;
        const step = joint.kind === 'slider' ? 0.0005 : Math.PI / 360;
        return `<label class="jog"><span>${escapeHtml(joint.name)}</span><input type="range" name="jog" data-joint="${joint.id}" min="${range.lower}" max="${range.upper}" step="${step}" value="${value}" aria-label="Jog ${escapeHtml(joint.name)}"><output>${jogShown(joint, value)}</output></label>`;
    }).join('')}
        <div class="buttons">
            <button type="button" class="tool" data-pose="add" title="Record this pose as a keyframe (K)">Add keyframe</button>
            <button type="button" class="tool" data-pose="setpoints" title="Make this pose the joints' setpoints">Set as setpoints</button>
            <button type="button" class="tool" data-pose="drawn" title="Back to the robot as drawn (Escape)"${state.pose.active ? '' : ' disabled'}>As drawn</button>
        </div>
        <h2>Keyframes</h2>
        ${keyframes.length ? `<ol class="keyframes">${keyframes.map((keyframe) => `<li data-keyframe="${keyframe.id}"${keyframe.id === state.pose.keyframe ? ' class="selected"' : ''}>
            <input type="number" name="keyframeTime" value="${keyframe.time}" min="0" step="0.1" aria-label="Keyframe time"><span class="unit">s</span>
            <button type="button" class="tool" data-keyframe-action="show" title="Pose the robot as this keyframe">Show</button>
            <button type="button" class="tool" data-keyframe-action="replace" title="Make this keyframe the pose now">Replace</button>
            <button type="button" class="tool" data-keyframe-action="remove" title="Remove this keyframe" aria-label="Remove">✕</button>
        </li>`).join('')}</ol>
        <p class="note">Build makes each joint's setpoint follow its keyframes in straight lines, a schedule on its setpoint in the canvas. [ and ] show the keyframe before and after.</p>`
        : '<p class="note">Pose the robot with the sliders, then Add keyframe (K) at each pose it should pass through. Build makes the joints\' setpoints follow them over time.</p>'}`;
}

function jogged(input) {
    const joint = jointById(drawing(), input.dataset.joint);
    if (!joint) return;
    state.pose.values = { ...state.pose.values, [joint.id]: Number(input.value) };
    state.pose.active = true;
    state.pose.keyframe = null;
    input.nextElementSibling.textContent = jogShown(joint, Number(input.value));
    view.setDrawing(posedDrawing(drawing(), state.pose.values));
    $('#posePanel [data-pose="drawn"]').disabled = false;
    renderHint();
}

function addPoseKeyframe() {
    if (!drivenJoints(drawing()).length) { toast('Drive a joint by a servo or a motor first.'); return; }
    const time = nextKeyframeTime(drawing());
    const added = addKeyframe(drawing(), { time, setpoints: poseValues() });
    state.pose.keyframe = added.id;
    change(added.drawing, `Add the keyframe at ${time} s`);
}

function showKeyframe(id) {
    const keyframe = keyframesOf(drawing()).find((candidate) => candidate.id === id);
    if (!keyframe) return;
    state.pose = { active: true, values: { ...keyframe.setpoints }, keyframe: id };
    if (state.tool !== 'select') state.tool = 'select';
    render();
}

function stepKeyframe(direction) {
    const keyframes = keyframesOf(drawing());
    if (!keyframes.length) return;
    const index = keyframes.findIndex(({ id }) => id === state.pose.keyframe);
    const next = index < 0 ? (direction > 0 ? 0 : keyframes.length - 1) : Math.max(0, Math.min(keyframes.length - 1, index + direction));
    showKeyframe(keyframes[next].id);
}

// A drawing put in place of the whole one (an example, a drawing opened, a robot imported): the jog panel starts again
// from the robot as drawn, since its joints are others, whatever their ids.
function forgetPose() {
    state.pose = { active: false, values: {}, keyframe: null };
}

function poseAsDrawn() {
    state.pose = { active: false, values: state.pose.values, keyframe: null };
    render();
}

function poseAction(action) {
    if (action === 'add') { addPoseKeyframe(); return; }
    if (action === 'drawn') { poseAsDrawn(); return; }
    if (action === 'setpoints') {
        let next = drawing();
        for (const [jointId, value] of Object.entries(poseValues())) next = updateJoint(next, jointId, { drive: { setpoint: value } });
        change(next, 'Set the setpoints from the pose');
    }
}

function keyframeAction(id, action) {
    if (action === 'show') { showKeyframe(id); return; }
    if (action === 'replace') { change(updateKeyframe(drawing(), id, { setpoints: poseValues() }), 'Replace the keyframe'); return; }
    if (action === 'remove') {
        if (state.pose.keyframe === id) state.pose.keyframe = null;
        change(removeKeyframe(drawing(), id), 'Remove the keyframe');
    }
}

// ---- sizing: will these motors do it? -----------------------------------------------------------------------------------
// Konjugate runs the built robot (runBaseline, as Run does in the canvas) and the window reads each motor's run back:
// where it worked on its torque and speed band, how much of the run it gave all it could, and how hot its winding got.
// A what-if forks that run from a time with one thing changed (runScenario: a heavier solid, a weaker motor, a tired
// battery), a branch of its own that Show in canvas opens beside it. The sums are lib/sizing.mjs.

const whatIfs = Object.freeze([
    { id: 'heavierSolid', label: 'A heavier solid' },
    { id: 'weakerMotor', label: 'A weaker motor' },
    { id: 'tiredBattery', label: 'A tired battery' }
]);

// The model the host holds for this window: Build's, or one built for a sizing run, from what Build was given.
let importedFrom = null;
let progressHeard = false;

async function ensureImported() {
    const input = buildInput();
    if (importedFrom === input) return;
    const { errors } = checkDrawing(drawing());
    const problems = [...errors, ...(errors.length ? [] : simulationLimits(drawing()))];
    if (problems.length) throw new Error(problems[0].message);
    const answer = await api.runImport('drawing', { step: 'build', drawing: drawing(), runTime: state.settings.runTime, timeStep: state.settings.timeStep });
    if (!answer.imported) throw new Error((answer.report?.errors ?? ['The model could not be built.']).join(' '));
    importedFrom = input;
}

function openSizing() {
    state.sizing.open = true;
    renderSizing();
    // Opened with nothing run for this robot yet, it runs it as drawn.
    if (state.sizing.from !== buildInput() && !state.sizing.running && motorsOf(drawing()).length) runAsDrawn();
}
// What Size says of a robot with no motor: a robot on servos can have them made motors.
const noMotors = () => (drivenJoints(drawing()).some((joint) => joint.drive.mode === 'servo' && joint.kind === 'hinge')
    ? 'Sizing looks at motors: make the servos motors first, under Servos to motors with nothing selected.'
    : 'Sizing looks at motors: drive a hinge or a slider in motor mode, and run it here.');
function closeSizing() { state.sizing.open = false; renderSizing(); }
const toggleSizing = () => (state.sizing.open ? closeSizing() : openSizing());

async function sizingRun(label, work) {
    if (!api?.runBaseline) { toast('Sizing needs the Robotics window opened from Konjugate.', 'error'); return; }
    if (!motorsOf(drawing()).length) { toast(noMotors(), 'error'); return; }
    if (state.sizing.running) return;
    if (!progressHeard && api.onProgress) {
        progressHeard = true;
        api.onProgress((progress) => { if (state.sizing.running && progress?.message) { state.sizing.status = progress.message; renderSizing(); } });
    }
    state.sizing.running = true;
    state.sizing.status = `${label}…`;
    state.sizing.error = null;
    renderSizing();
    try {
        await ensureImported();
        await work();
        state.sizing.status = '';
    } catch (error) {
        state.sizing.error = error.message;
        state.sizing.status = '';
        toast(error.message, 'error');
    } finally {
        state.sizing.running = false;
        renderSizing();
    }
}

function runAsDrawn() {
    return sizingRun('Running the robot as drawn', async () => {
        const from = buildInput();
        const answer = await api.runBaseline({ runTime: state.settings.runTime, signals: [...sizingSignals] });
        state.sizing.from = from;
        state.sizing.asDrawn = summarize(answer.branches[0], drawing());
        state.sizing.whatIf = null;
    });
}

// The what-if form as the user left it, filled in with what this robot has.
function whatIfForm() {
    const form = state.settings.whatIf ?? {};
    const solids = drawing().solids;
    const motors = motorsOf(drawing());
    const solid = solids.some(({ name }) => name === form.solid) ? form.solid : (solids.find(({ name }) => name === 'Part') ?? solids.at(-1))?.name;
    const solidMass = solids.find(({ name }) => name === solid);
    return {
        scenarioId: whatIfs.some(({ id }) => id === form.scenarioId) ? form.scenarioId : 'heavierSolid',
        forkAt: Number.isFinite(form.forkAt) ? form.forkAt : 0,
        solid, mass: Number.isFinite(form.mass) ? form.mass : (solidMass ? 2 * massOf(solidMass) : 1),
        motor: motors.some(({ name }) => name === form.motor) ? form.motor : motors[0]?.name,
        share: Number.isFinite(form.share) ? form.share : 0.7,
        drop: Number.isFinite(form.drop) ? form.drop : 1.5
    };
}

function runWhatIf() {
    let asked;
    try {
        asked = whatIf(drawing(), { ...whatIfForm(), runTime: state.settings.runTime });
    } catch (error) {
        toast(error.message, 'error');
        return null;
    }
    const { scenarioId } = whatIfForm();
    return sizingRun(`Running the robot with ${asked.change}`, async () => {
        const from = buildInput();
        const answer = await api.runScenario(scenarioId, { ...asked.options, retain: true });
        const branch = (id) => answer.branches.find((candidate) => candidate.id === id);
        state.sizing.from = from;
        state.sizing.asDrawn = summarize(branch('baseline'), drawing());
        state.sizing.whatIf = { scenarioId, change: asked.change, forkTime: answer.forkTime, summary: summarize(branch('scenario'), asked.changed) };
    });
}

async function showSizingInCanvas() {
    if (!state.sizing.asDrawn || state.sizing.from !== buildInput()) { toast('Run the robot first: Run as drawn, or a what-if.', 'error'); return; }
    try {
        await ensureImported();
        await api.openInCanvas(state.sizing.whatIf?.scenarioId ?? null, { focus: true, silent: true, session: sessionState() });
        toast(state.sizing.whatIf ? 'Opened in the canvas: the run as drawn, and the what-if as a branch of it.' : 'Opened in the canvas, with its run.');
    } catch (error) {
        toast(error.message, 'error');
    }
}

function setWhatIf(name, value) {
    const form = { ...whatIfForm() };
    if (name === 'scenarioId' || name === 'solid' || name === 'motor') form[name] = value;
    else {
        const number = Number(value);
        if (!Number.isFinite(number)) { renderSizing(); return; }
        if (name === 'mass') form.mass = number / 1000;
        else if (name === 'share') form.share = number / 100;
        else form[name] = number;
    }
    state.settings.whatIf = form;
    keepSessionSoon();
    renderSizing();
}

const units = (entry) => (entry.linear ? { speed: 'm/s', torque: 'N' } : { speed: 'rad/s', torque: 'N·m' });
const figure = (value, digits = 3) => (value === null || value === undefined || !Number.isFinite(value) ? '–' : Number(value.toPrecision(digits)).toString());
const celsius = (kelvin) => `${(kelvin - 273.15).toFixed(0)} °C`;

// A motor's band at its joint, at a full battery (solid) and an empty one (dashed), and where it worked in the run (and,
// in orange, in the what-if, with the band its change gives), torque against speed, either way.
function bandPlot(entry, supply, changed, changedSupply) {
    const width = 250;
    const height = 160;
    const pad = 26;
    const entries = [entry, changed].filter(Boolean);
    const xMost = 1.05 * Math.max(...entries.flatMap((summary) => [summary.noLoadSpeed, summary.peakSpeed]), 1e-9);
    const yMost = 1.05 * Math.max(...entries.flatMap((summary) => [summary.stallTorque, summary.peakTorque]), 1e-9);
    const x = (speed) => (pad + ((speed + xMost) / (2 * xMost)) * (width - 2 * pad)).toFixed(1);
    const y = (torque) => (height - pad - ((torque + yMost) / (2 * yMost)) * (height - 2 * pad)).toFixed(1);
    const edges = (motor, voltage, className) => {
        const band = envelope(motor, voltage);
        return [band.most, band.least].map((edge) => `<line class="${className}" x1="${x(-xMost)}" y1="${y(edge(-xMost))}" x2="${x(xMost)}" y2="${y(edge(xMost))}"/>`).join('');
    };
    const points = (summary, className) => summary.points.map(([speed, torque]) => `<circle class="${className}" cx="${x(speed)}" cy="${y(torque)}" r="1.6"/>`).join('');
    const { speed, torque } = units(entry);
    return `<svg class="band" viewBox="0 0 ${width} ${height}" role="img" aria-label="${escapeHtml(entry.name)}: torque against speed at the joint, within the motor's band">
        <defs><clipPath id="clip-${entry.joint}"><rect x="${pad}" y="${pad}" width="${width - 2 * pad}" height="${height - 2 * pad}"/></clipPath></defs>
        <line class="axis" x1="${x(0)}" y1="${pad}" x2="${x(0)}" y2="${height - pad}"/><line class="axis" x1="${pad}" y1="${y(0)}" x2="${width - pad}" y2="${y(0)}"/>
        <g clip-path="url(#clip-${entry.joint})">
            ${edges(entry.motor, supply.fullVoltage, 'asDrawn edge')}${edges(entry.motor, supply.emptyVoltage, 'asDrawn edge empty')}
            ${changed ? edges(changed.motor, changedSupply.fullVoltage, 'changed edge') : ''}
            ${points(entry, 'asDrawn')}${changed ? points(changed, 'changed') : ''}
        </g>
        <text x="${width - pad}" y="${height - 8}" text-anchor="end">speed, ${speed} (±${figure(xMost, 2)})</text>
        <text x="4" y="12">torque, ${torque} (±${figure(yMost, 2)})</text>
    </svg>`;
}

function renderSizing() {
    const panel = $('#sizingPanel');
    $('#sizeButton').setAttribute('aria-pressed', String(state.sizing.open));
    panel.hidden = !state.sizing.open;
    if (!state.sizing.open) return;
    const motors = motorsOf(drawing());
    const form = whatIfForm();
    const { asDrawn, whatIf: changed } = state.sizing;
    const current = state.sizing.from === buildInput();
    const option = (value, label, chosen) => `<option value="${escapeHtml(value)}"${value === chosen ? ' selected' : ''}>${escapeHtml(label)}</option>`;
    const fields = {
        heavierSolid: `<label>Solid <select data-whatif="solid">${drawing().solids.map(({ name }) => option(name, name, form.solid)).join('')}</select></label>
            <label>Mass <input type="number" data-whatif="mass" min="0" step="any" value="${figure(form.mass * 1000, 6)}"> g</label>`,
        weakerMotor: `<label>Motor <select data-whatif="motor">${motors.map(({ name }) => option(name, name, form.motor)).join('')}</select></label>
            <label>Torque per ampere <input type="number" data-whatif="share" min="1" max="200" step="any" value="${figure(form.share * 100, 4)}"> %</label>`,
        tiredBattery: `<label>Fewer volts <input type="number" data-whatif="drop" min="0" step="any" value="${figure(form.drop, 4)}"> V</label>`
    };
    const verdictWords = { ok: 'Does it', close: 'Close', over: 'Struggles' };
    const card = (entry, index) => {
        const other = changed?.summary.motors[index] ?? null;
        const { speed, torque } = units(entry);
        const row = (label, value, otherValue, of = '') => `<tr><th>${label}</th><td>${value}</td>${changed ? `<td>${otherValue}</td>` : ''}<td class="muted">${of}</td></tr>`;
        const verdict = (summary) => `<span class="verdict ${summary.verdict.level}">${verdictWords[summary.verdict.level]}</span>`;
        return `<article class="motorCard" data-joint="${entry.joint}">
            <h3>${escapeHtml(entry.name)} ${verdict(entry)}${other ? ` <span class="muted">· what if</span> ${verdict(other)}` : ''}</h3>
            ${bandPlot(entry, asDrawn.supply, other, changed?.summary.supply)}
            <table>
                ${changed ? '<tr><th></th><td class="muted">As drawn</td><td class="muted">What if</td><td></td></tr>' : ''}
                ${row('Most torque', `${figure(entry.peakTorque)}`, other ? figure(other.peakTorque) : '', `of ${figure(entry.stallTorque)} ${torque} stall (${(100 * Math.max(entry.peakTorque, other?.peakTorque ?? 0) / entry.stallTorque).toFixed(0)}%)`)}
                ${row('Most speed', `${figure(entry.peakSpeed)}`, other ? figure(other.peakSpeed) : '', `of ${figure(entry.noLoadSpeed)} ${speed} no-load`)}
                ${row('At full duty', `${(100 * entry.atFullDuty).toFixed(0)}%`, other ? `${(100 * other.atFullDuty).toFixed(0)}%` : '', 'of the run')}
                ${row('Hottest winding', celsius(entry.hottest), other ? celsius(other.hottest) : '', `of ${celsius(entry.motor.windingLimit)}`)}
                ${row('Most current', `${figure(entry.peakCurrent)} A`, other ? `${figure(other.peakCurrent)} A` : '', '')}
            </table>
            ${[entry, other].filter(Boolean).flatMap((summary) => summary.verdict.reasons.map((reason) => `<p class="reason ${summary.verdict.level}">${summary === other ? 'What if: ' : ''}${escapeHtml(reason[0].toUpperCase() + reason.slice(1))}.</p>`)).join('')}
        </article>`;
    };
    const battery = asDrawn ? `<article class="motorCard batteryCard"><h3>Battery</h3><table>
        ${changed ? '<tr><th></th><td class="muted">As drawn</td><td class="muted">What if</td></tr>' : ''}
        <tr><th>Charge used</th><td>${figure(asDrawn.battery.chargeUsed, 3)}%</td>${changed ? `<td>${figure(changed.summary.battery.chargeUsed, 3)}%</td>` : ''}</tr>
        <tr><th>Lowest voltage</th><td>${figure(asDrawn.battery.lowestVoltage, 4)} V</td>${changed ? `<td>${figure(changed.summary.battery.lowestVoltage, 4)} V</td>` : ''}</tr>
        </table></article>` : '';
    panel.innerHTML = `<header>
            <h2>Will these motors do it?</h2>
            <button type="button" class="primary" data-sizing="run"${state.sizing.running ? ' disabled' : ''} title="Run the robot as drawn for its run time and size its motors">Run as drawn</button>
            <span class="whatIf">What if
                <select data-whatif="scenarioId" aria-label="What to change">${whatIfs.map(({ id, label }) => option(id, label, form.scenarioId)).join('')}</select>
                ${fields[form.scenarioId]}
                <label>from <input type="number" data-whatif="forkAt" min="0" step="any" value="${figure(form.forkAt, 6)}"> s</label>
                <button type="button" class="tool" data-sizing="whatIf"${state.sizing.running ? ' disabled' : ''} title="Run the robot as drawn and, from that time, with this changed">Run what-if</button>
            </span>
            <button type="button" class="tool" data-sizing="show"${asDrawn && current && !state.sizing.running ? '' : ' disabled'} title="Open the model in the canvas with these runs as its branches">Show in canvas</button>
            <button type="button" class="tool close" data-sizing="close" title="Close (M, or Escape)" aria-label="Close sizing">✕</button>
        </header>
        <p class="status" aria-live="polite">${escapeHtml(state.sizing.status || (state.sizing.error ? `Not run: ${state.sizing.error}` : !motors.length ? noMotors()
            : !asDrawn ? 'Run the robot to see whether its motors do it.' : !current ? 'The robot has changed since this run: run it again to size it as drawn now.'
                : `${state.settings.runTime} s as drawn${changed ? `, and with ${changed.change}` : ''}. Each motor's band is what it can give at its joint (full battery, solid; empty, dashed); a point on its edge is the motor giving all it can.`))}</p>
        <div class="cards${current ? '' : ' stale'}">${asDrawn ? asDrawn.motors.map(card).join('') + battery : ''}</div>`;
}

// ---- importing a robot from a model ---------------------------------------------------------------------------------
// A robot from MuJoCo Menagerie, its licence shown and its files kept on this computer when the user asks, pinned to
// one commit; or a model's files brought in from disk. Importing one replaces the drawing (undo brings it back) and
// says what came in and what was left out; its looks are kept with the project (the importer derives them), so the
// window shows the imported solids again when it is reopened.

const importState = { licences: new Map(), stored: new Map(), busy: null, report: null, error: null, progress: null };
const shortCommit = (commit) => commit.slice(0, 7);
const fetchJson = async (url) => JSON.parse((await api.fetchText(url)).text);

// What is kept on this computer: each Menagerie robot's files by folder, and each batch brought in from disk.
async function refreshStored() {
    const { files } = await api.storedFiles();
    importState.stored = new Map();
    for (const file of files) {
        const key = file.details?.robot ? `menagerie:${file.details.robot}` : file.details?.batch ? `disk:${file.details.batch}` : null;
        if (!key) continue;
        if (!importState.stored.has(key)) importState.stored.set(key, []);
        importState.stored.get(key).push(file);
    }
}

async function openImport() {
    if (!api?.storedFiles) { toast('Importing a robot needs the Robotics window opened from Konjugate.', 'error'); return; }
    importState.error = null;
    $('#importDialog').showModal();
    renderImport();
    try { await refreshStored(); } catch (error) { importState.error = error.message; }
    renderImport();
}

function renderImport() {
    const dialog = $('#importDialog');
    if (!dialog.open) return;
    const busy = Boolean(importState.busy);
    const rows = robots.map((robot) => {
        const kept = importState.stored.get(`menagerie:${robot.folder}`);
        const licence = importState.licences.get(robot.folder);
        const commit = kept?.[0]?.details?.commit;
        return `<li class="robotRow" data-folder="${robot.folder}">
            <div><strong>${escapeHtml(robot.name)}</strong> <span class="muted">by ${escapeHtml(robot.maker)}</span><br><span class="muted small">${escapeHtml(robot.what)}${kept ? ` Kept on this computer (${kept.length} files, Menagerie at ${shortCommit(commit)}${kept[0].details.licence ? `, ${escapeHtml(kept[0].details.licence)}` : ''}).` : ''}</span></div>
            <div class="buttons">
                ${kept ? `<button type="button" class="primary" data-import="import"${busy ? ' disabled' : ''}>Import</button>` : ''}
                ${kept ? '' : `<button type="button" class="tool" data-import="licence"${busy ? ' disabled' : ''}>${licence ? 'Hide licence' : 'Licence'}</button>`}
                ${licence && !kept ? `<button type="button" class="primary" data-import="download"${busy ? ' disabled' : ''}>Download</button>` : ''}
            </div>
            ${licence && !kept ? `<div class="licence"><p>Licence: ${escapeHtml(licence.name ?? 'not recognised; read it below')}. From ${escapeHtml(repository.url)}, folder ${escapeHtml(robot.folder)}, at commit ${shortCommit(licence.commit)}. Downloading keeps its files on this computer; a project built from it carries its look and this licence.</p><pre>${escapeHtml(licence.text)}</pre></div>` : ''}
        </li>`;
    }).join('');
    const disk = [...importState.stored].filter(([key]) => key.startsWith('disk:')).map(([key, files]) => `<li class="robotRow" data-batch="${escapeHtml(key.slice(5))}">
        <div><strong>${escapeHtml(files[0].details.label ?? files[0].name)}</strong><br><span class="muted small">${files.length} files brought in from this computer.</span></div>
        <div class="buttons"><button type="button" class="primary" data-import="import"${busy ? ' disabled' : ''}>Import</button></div>
    </li>`).join('');
    const report = importState.report;
    $('#importBody').innerHTML = `
        <h3>From MuJoCo Menagerie</h3>
        <ul class="robots">${rows}</ul>
        <h3>From this computer</h3>
        <p class="muted small">A MuJoCo model: its MJCF (.xml) and the STL or OBJ meshes it reads, chosen together.</p>
        <ul class="robots">${disk}</ul>
        <div class="buttons"><button type="button" class="tool" data-import="disk"${busy ? ' disabled' : ''}>Choose its files…</button></div>
        <p class="status" aria-live="polite">${escapeHtml(importState.busy ?? '')}${importState.progress ? ` ${importState.progress}` : ''}${importState.error ? `<span class="error">${escapeHtml(importState.error)}</span>` : ''}</p>
        ${report ? `<div class="importReport"><p><strong>${escapeHtml(report.name)}</strong>: ${report.solids} solids and ${report.joints} joints, ${report.drives} driven by its actuators. Undo brings your drawing back.</p>
            ${report.leftOut.length ? `<p>Left out or taken approximately:</p><ul>${report.leftOut.map((line) => `<li>${escapeHtml(line[0].toUpperCase() + line.slice(1))}.</li>`).join('')}</ul>` : '<p>Nothing was left out.</p>'}</div>` : ''}`;
}

async function importWork(label, work) {
    if (importState.busy) return;
    importState.busy = label;
    importState.error = null;
    importState.progress = null;
    renderImport();
    try {
        await work();
    } catch (error) {
        // GitHub answers only so many requests an hour from one address without signing in, and says 403 or 429 past it.
        importState.error = /api\.github\.com answered (403|429)/.test(error.message)
            ? `${error.message} GitHub answers only so many requests an hour from one network without signing in; try again in a while.`
            : error.message;
    } finally {
        importState.busy = null;
        importState.progress = null;
        renderImport();
    }
}

// The licence of the robot at Menagerie's current commit, which a download then pins to.
function toggleLicence(folder) {
    if (importState.licences.has(folder)) { importState.licences.delete(folder); renderImport(); return null; }
    return importWork('Fetching its licence…', async () => {
        const commit = await latestCommit(fetchJson);
        const text = (await api.fetchText(rawUrl(commit, `${folder}/LICENSE`))).text;
        importState.licences.set(folder, { commit, name: licenceName(text), text });
    });
}

// The files the robot's model reads, at the licence's commit, into the store, each with where it belongs.
function download(folder) {
    const licence = importState.licences.get(folder);
    if (!licence) return null;
    return importWork('Downloading…', async () => {
        const files = await modelFilesFromModel(async (url) => (await api.fetchText(url)).text, licence.commit, folder);
        let done = 0;
        for (const file of files) {
            importState.progress = `${done} of ${files.length} files`;
            renderImport();
            await api.downloadFile(file.url, { name: storedName(file.path), details: { robot: folder, commit: licence.commit, path: file.path, licence: licence.name } });
            done += 1;
        }
        importState.licences.delete(folder);
        await refreshStored();
    });
}

// Files brought in from disk, kept as one batch, then imported.
function fromDisk() {
    return importWork('Bringing the files in…', async () => {
        const batch = `b${Date.now().toString(36)}`;
        const { files } = await api.storeFiles({ batch }, { accept: ['xml', 'stl', 'obj'], title: 'A MuJoCo model: its MJCF and its meshes' });
        if (!files.length) return;
        const main = files.find(({ name }) => /\.xml$/i.test(name));
        for (const file of files) await api.describeStoredFile(file.id, { batch, label: main?.name ?? file.name });
        await refreshStored();
        await importFiles(importState.stored.get(`disk:${batch}`), null);
    });
}

// Imports the stored files as the drawing: undoable, its looks shown and kept with the project.
async function importFiles(files, folder) {
    importState.busy = 'Importing…';
    renderImport();
    const robot = folder ? robotByFolder(folder) : null;
    const commit = files[0]?.details?.commit;
    await api.useStoredFiles('drawing', 'model', files.map(({ id }) => id));
    const answer = await api.runImport('drawing', {
        step: 'model', useBinaryFiles: true,
        paths: Object.fromEntries(files.filter(({ details }) => details?.path).map(({ name, details }) => [name, details.path])),
        ...(robot ? { name: robot.name, source: { kind: 'menagerie', repository: repository.url, folder, commit, licence: files[0].details.licence ?? null } } : {})
    });
    if (!answer.data?.drawing) throw new Error((answer.report?.errors ?? ['The model could not be imported.']).join(' '));
    const { drawing: imported, looks, report } = answer.data;
    state.looks = { ...state.looks, ...looks };
    view.setLooks(state.looks);
    state.selection = [];
    setTool('select');
    forgetPose();
    change(readDrawing(imported), `Import ${imported.name}`);
    view.fit();
    importState.report = { name: imported.name, ...report };
    toast(`Imported ${imported.name}: ${report.solids} solids and ${report.joints} joints.${state.history.undoStack.length > 1 ? ' Undo brings your drawing back.' : ''}`);
}

function importAction(action, row) {
    if (action === 'licence') return toggleLicence(row.dataset.folder);
    if (action === 'download') return download(row.dataset.folder);
    if (action === 'disk') return fromDisk();
    if (action === 'import') {
        const files = row.dataset.folder ? importState.stored.get(`menagerie:${row.dataset.folder}`) : importState.stored.get(`disk:${row.dataset.batch}`);
        return importWork('Importing…', () => importFiles(files, row.dataset.folder ?? null));
    }
    return null;
}

// The looks kept with the project, back for the imported solids when the window reopens.
async function restoreLooks() {
    if (!api?.runImport || !drawing().solids.some((solid) => solid.look?.key)) return;
    try {
        const answer = await api.runImport('drawing', { step: 'looks' });
        state.looks = { ...state.looks, ...(answer.data?.looks ?? {}) };
        view.setLooks(state.looks);
    } catch {
        // Without them the imported solids are shown as the boxes about them.
    }
}

// ---- help -------------------------------------------------------------------------------------------------------------

const helpRows = () => [
    ['Add a box, cylinder, sphere, capsule', 'The Add buttons; B, C, S, P'],
    ['Select', 'Click a solid or a joint, in the view or in Parts'],
    ['Select several', keys.addClick],
    ['Turn, move, zoom the view', 'Drag, right-drag (or Shift-drag), scroll'],
    ['Fit the robot in the view', 'Fit; F'],
    ['Move or turn a solid', 'Move (G) or Turn (R), then drag the handles; or type its place and turn'],
    ['Join two solids', 'Join (J), then click a point on each; start on the floor to fix one to the Ground. Shift-click the second to leave it where it is'],
    ['Change a joint', 'Select it: its kind, point and axis, limits, damping and drive (free, a torque, a servo or a motor)'],
    ['Pose the robot', 'The jog sliders under Pose; Escape or As drawn goes back to the robot as drawn'],
    ['Keyframe a pose', 'Add keyframe; K. Show, Replace or remove one in Keyframes, or click it; [ and ] show the one before and after'],
    ['Duplicate', `Duplicate; ${keys.duplicate}`],
    ['Delete', `Delete; ${keys.deleteKeys}`],
    ['Undo, redo', `${keys.undo}; ${keys.redo}`],
    ['Back out', 'Escape: stop joining, put the handles away, then clear the selection'],
    ['Import a robot from a MuJoCo model', 'Import robot…; I. From MuJoCo Menagerie (its licence shown first) or from its files on this computer'],
    ['Build into the canvas', `Build into canvas; ${keys.build}`],
    ['Will these motors do it?', 'Size; M opens and closes it, running the robot when it has not run as drawn. Run as drawn, or a what-if from a time; Show in canvas opens the runs there'],
    ['These keys', '?']
];

function showHelp() {
    $('#helpTable').innerHTML = helpRows().map(([action, how]) => `<tr><td>${escapeHtml(action)}</td><td>${escapeHtml(how)}</td></tr>`).join('');
    $('#helpDialog').showModal();
}

// ---- events -----------------------------------------------------------------------------------------------------------

for (const button of $$('[data-add]')) button.addEventListener('click', () => addShape(button.dataset.add));
for (const button of $$('[data-tool]')) button.addEventListener('click', () => setTool(button.dataset.tool));
$('#joinKind').addEventListener('change', () => renderHint());
$('#deleteButton').addEventListener('click', deleteSelection);
$('#duplicateButton').addEventListener('click', duplicateSelection);
$('#undoButton').addEventListener('click', undo);
$('#redoButton').addEventListener('click', redo);
$('#fitButton').addEventListener('click', () => view.fit());
$('#viewSide').addEventListener('change', (event) => { if (event.target.value) view.look(event.target.value); event.target.value = ''; });
$('#snapToggle').addEventListener('change', (event) => view.setSnapping(event.target.checked));
$('#buildButton').addEventListener('click', build);
$('#openButton').addEventListener('click', openDrawing);
$('#importButton').addEventListener('click', openImport);
$('#importBody').addEventListener('click', (event) => {
    const button = event.target.closest('button[data-import]');
    if (button) importAction(button.dataset.import, button.closest('.robotRow') ?? button);
});
$('#exampleSelect').innerHTML += examples.map(({ id, name, description }) => `<option value="${id}" title="${escapeHtml(description)}">${escapeHtml(name)}</option>`).join('');
$('#exampleSelect').addEventListener('change', (event) => {
    const chosen = examples.find(({ id }) => id === event.target.value);
    event.target.value = '';
    if (!chosen) return;
    state.selection = [];
    setTool('select');
    forgetPose();
    change(exampleDrawing(chosen.id), `Start from the ${chosen.name.toLowerCase()}`);
    view.fit();
    toast(`${chosen.name}: ${chosen.description}${state.history.undoStack.length > 1 ? ' Undo brings your drawing back.' : ''}`);
});
$('#helpButton').addEventListener('click', showHelp);
$('#sizeButton').addEventListener('click', toggleSizing);
$('#sizingPanel').addEventListener('click', (event) => {
    const action = event.target.closest('button[data-sizing]')?.dataset.sizing;
    if (action === 'run') runAsDrawn();
    else if (action === 'whatIf') runWhatIf();
    else if (action === 'show') showSizingInCanvas();
    else if (action === 'close') closeSizing();
});
$('#sizingPanel').addEventListener('change', (event) => {
    const name = event.target.dataset?.whatif;
    if (name) setWhatIf(name, event.target.value);
});
$('#sizingPanel').addEventListener('keydown', (event) => { if (event.key === 'Enter' && event.target.matches('input')) event.target.blur(); });
$('#robotName').addEventListener('change', (event) => {
    const name = event.target.value.trim();
    if (!name) { event.target.value = drawing().name; return; }
    change({ ...structuredClone(drawing()), name }, 'Name the robot', 'robotName');
});
$('#partsTree').addEventListener('click', (event) => {
    const item = event.target.closest('li[data-id]');
    if (!item || item.dataset.id === groundId) { select([]); return; }
    select(item.dataset.id, { add: addsToSelection(event, keys) });
});
$('#checksList').addEventListener('click', (event) => {
    const item = event.target.closest('li[data-id]');
    if (item) select(item.dataset.id);
});
$('#inspector').addEventListener('change', (event) => { if (event.target.name) inspectorChanged(event.target); });
$('#posePanel').addEventListener('input', (event) => { if (event.target.name === 'jog') jogged(event.target); });
$('#posePanel').addEventListener('change', (event) => {
    if (event.target.name !== 'keyframeTime') return;
    const id = event.target.closest('[data-keyframe]').dataset.keyframe;
    const time = Number(event.target.value);
    if (!(Number.isFinite(time) && time >= 0)) { toast('A keyframe\'s time is zero or more seconds.'); renderPose(); return; }
    change(updateKeyframe(drawing(), id, { time }), 'Move the keyframe', `keyframe:${id}`);
});
$('#posePanel').addEventListener('keydown', (event) => { if (event.key === 'Enter' && event.target.matches('input[type="number"]')) event.target.blur(); });
$('#posePanel').addEventListener('click', (event) => {
    const button = event.target.closest('button');
    if (button?.dataset.pose) { poseAction(button.dataset.pose); return; }
    const row = event.target.closest('[data-keyframe]');
    if (!row) return;
    if (button?.dataset.keyframeAction) { keyframeAction(row.dataset.keyframe, button.dataset.keyframeAction); return; }
    // Clicking a keyframe's row (not its time) shows it.
    if (!event.target.matches('input')) showKeyframe(row.dataset.keyframe);
});
$('#inspector').addEventListener('click', (event) => {
    const button = event.target.closest('button[data-action]');
    if (button) inspectorAction(button.dataset.action);
});
$('#inspector').addEventListener('keydown', (event) => {
    // Enter in a field applies it, as leaving it does.
    if (event.key === 'Enter' && event.target.matches('input')) event.target.blur();
});

const typing = (target) => target.matches?.('input[type="text"], input[type="number"], select, textarea') || target.isContentEditable;

document.addEventListener('keydown', (event) => {
    if ($('#helpDialog').open || $('#importDialog').open) return;
    const command = commandHeld(event, keys);
    const key = event.key.toLowerCase();
    if (command && key === 'z') { event.preventDefault(); if (event.shiftKey) redo(); else undo(); return; }
    if (command && key === 'y' && !keys.mac) { event.preventDefault(); redo(); return; }
    if (command && key === 'b') { event.preventDefault(); build(); return; }
    if (typing(event.target)) return;
    if (command && key === 'd') { event.preventDefault(); duplicateSelection(); return; }
    if (command || event.altKey) return;
    if (event.key === 'Escape') {
        if (state.pose.active) { poseAsDrawn(); return; }
        if (state.tool === 'join' && state.joinFirst) { state.joinFirst = null; view.showMarker('chosen', null); render(); return; }
        if (state.tool !== 'select') { setTool('select'); return; }
        if (state.sizing.open && !state.selection.length) { closeSizing(); return; }
        select([]);
        return;
    }
    if (event.key === 'Delete' || event.key === 'Backspace') { event.preventDefault(); deleteSelection(); return; }
    const shortcuts = { b: () => addShape('box'), c: () => addShape('cylinder'), s: () => addShape('sphere'), p: () => addShape('capsule'),
        j: () => setTool('join'), g: () => setTool('move'), r: () => setTool('turn'), f: () => view.fit(), '?': showHelp,
        k: addPoseKeyframe, '[': () => stepKeyframe(-1), ']': () => stepKeyframe(1), m: toggleSizing, i: openImport };
    if (shortcuts[event.key]) { event.preventDefault(); shortcuts[event.key](); }
});

// For tests: the window's state, read only.
window.roboticsWindow = { drawing, state, view };

await restore();
await restoreLooks();
