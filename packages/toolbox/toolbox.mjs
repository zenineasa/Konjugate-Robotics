/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The Robotics window: draw a robot from solids, join them at points chosen explicitly, and build it into the canvas,
// where each solid is a node and each joint an edge. The drawing (lib/drawing.mjs) is the window's whole state worth
// keeping; it is kept with the project and restored when the window opens again.
//
// Every action has a button or field (for a newcomer), a gesture in the view (for a regular user) and a key (for a
// power user), works on the selection and can be undone, on Windows, Linux and macOS alike (docs/interaction.md).

import {
    addFrameAt, addJoint, addSolid, checkDrawing, createDrawing, driveLabels, driveModes, duplicateSolids, frameById, groundId,
    jointById, jointKinds, jointLabels, massOf, materials, moveSolids, poseOf, readDrawing, removeJoints, removeSolids, reverseJoint, solidById,
    structure, treeOrder, turnSolids, updateFrame, updateJoint, updateSolid
} from './lib/drawing.mjs';
import { exampleDrawing, examples } from './lib/examples.mjs';
import { History } from './lib/history.mjs';
import { simulationLimits } from './lib/modelBuilder.mjs';
import { conjugate, fromRollPitchYaw, multiply, normalizeQuaternion, subtract, toRollPitchYaw } from './lib/math.mjs';
import { addsToSelection, commandHeld, platformKeys } from './lib/platform.mjs';
import { shapeKinds, shapeLabels, sizeFields, volume } from './lib/shapes.mjs';
import { DrawingView } from './lib/view.mjs';

const api = window.konjugateLauncher ?? null;
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
    built: null,
    restoring: false
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
    view.setDrawing(current);
    view.setSelection(state.selection);
    const solids = selectedSolids();
    view.setTool(state.tool, solids.length === 1 ? carriedBy(solids[0]) : []);
    renderToolbar();
    renderTree();
    renderChecks();
    renderInspector();
    renderHint();
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
    // What the drawing may hold but Build cannot simulate yet: ball joints, loops and solids joined to nothing.
    // Each says why it cannot be built, in place of what the drawing would otherwise say of that solid or joint.
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
        ${field('Step', numberInput('timeStep', state.settings.timeStep * 1000, { min: 0.1, step: 0.1, title: 'The engine\'s step. Stiff servos and light solids need a short one; a millisecond suits most drawings.' }), 'ms')}`;
}

function solidInspector(solid) {
    const pose = poseOf(drawing(), solid.id);
    const [roll, pitch, yaw] = toRollPitchYaw(pose.rotation).map(toDegrees);
    const material = solid.mass.by === 'mass' ? 'mass' : solid.mass.material ?? 'other';
    const sizeInputs = sizeFields[solid.shape].map(([key, label]) => field(label, numberInput(`size.${key}`, toMillimetres(solid.size[key]), { min: 0.001 }), 'mm')).join('');
    const frames = drawing().frames.filter((frame) => frame.solid === solid.id);
    return `<h3>${escapeHtml(solid.name)}</h3>
        ${field('Name', `<input type="text" name="name" value="${escapeHtml(solid.name)}" spellcheck="false">`)}
        ${field('Shape', `<select name="shape">${shapeKinds.map((kind) => `<option value="${kind}"${kind === solid.shape ? ' selected' : ''}>${shapeLabels[kind]}</option>`).join('')}</select>`)}
        ${sizeInputs}
        <h2>Place</h2>
        <div class="fieldRow"><span>Position</span>${['x', 'y', 'z'].map((axis, index) => `<label>${axis}${numberInput(`position.${index}`, toMillimetres(pose.position[index]), { step: 1 })}</label>`).join('')}</div>
        <p class="unit">Millimetres, from the middle of the floor; z is up.</p>
        <div class="fieldRow"><span>Turn</span>${[['roll', roll], ['pitch', pitch], ['yaw', yaw]].map(([name, value]) => `<label>${name[0]}${numberInput(`turn.${name}`, value, { step: 5 })}</label>`).join('')}</div>
        <p class="unit">Degrees: yaw about z, then pitch about y, then roll about x.</p>
        <h2>Mass</h2>
        ${field('Made of', `<select name="material">${materials.map((item) => `<option value="${item.id}"${item.id === material ? ' selected' : ''}>${escapeHtml(item.name)}</option>`).join('')}<option value="other"${material === 'other' ? ' selected' : ''}>Another density</option><option value="mass"${material === 'mass' ? ' selected' : ''}>A mass I give</option></select>`)}
        ${solid.mass.by === 'mass'
        ? field('Mass', numberInput('massValue', solid.mass.value, { min: 0 }), 'kg')
        : field('Density', numberInput('density', solid.mass.density, { min: 0, step: 10 }), 'kg/m³')}
        <p class="note">${solid.mass.by === 'mass' ? `Its density works out at ${Math.round(solid.mass.value / volume(solid.shape, solid.size))} kg/m³.` : `Its mass works out at ${massOf(solid).toFixed(3)} kg, from its density and its ${(volume(solid.shape, solid.size) * 1e6).toFixed(1)} cm³.`}</p>
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

function jointInspector(joint) {
    const current = drawing();
    const parentFrame = frameById(current, joint.parent);
    const childFrame = frameById(current, joint.child);
    const built = structure(current);
    const loop = built.loops.some(({ id }) => id === joint.id);
    const linear = joint.kind === 'slider';
    const moving = joint.kind === 'hinge' || joint.kind === 'slider';
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
        }
        driveFields.push(`<p class="note">${{
            free: 'Nothing drives it: it moves only as gravity and the rest of the robot move it.',
            torque: `A constant ${linear ? 'force' : 'torque'}, a live slider on the relationship in the canvas. Wire a motor to it there to drive it from a motor instead.`,
            servo: 'A position servo holds it at its setpoint, a live slider on the relationship in the canvas, pushing with up to its most torque.'
        }[drive.mode]}</p>`);
    }
    return `<h3>${escapeHtml(joint.name)}</h3>
        ${field('Name', `<input type="text" name="name" value="${escapeHtml(joint.name)}" spellcheck="false">`)}
        ${field('Kind', `<select name="kind">${jointKinds.map((kind) => `<option value="${kind}"${kind === joint.kind ? ' selected' : ''}>${jointLabels[kind]}</option>`).join('')}</select>`)}
        <p class="note">Joins <strong>${escapeHtml(nameOf(parentFrame.solid))}</strong> (its point and axis) to <strong>${escapeHtml(nameOf(childFrame.solid))}</strong>.${loop ? ' It closes a loop, which Build cannot simulate yet: remove it or another joint of the loop to build.' : ''}</p>
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
        if (name === 'runTime' && number > 0) state.settings.runTime = number;
        if (name === 'timeStep' && number >= 0.1) state.settings.timeStep = number / 1000;
        keepSessionSoon();
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
            apply({ limits: value ? (linear ? { lower: -0.05, upper: 0.05 } : { lower: -Math.PI / 2, upper: Math.PI / 2 }) : null }, 'Limits');
            return;
        }
        if (name === 'limits.lower' || name === 'limits.upper') { apply({ limits: { ...joint.limits, [name.slice(7)]: fromShown(number) } }, 'Limits'); return; }
        if (['damping', 'friction', 'armature'].includes(name)) {
            if (number < 0) { toast('This must be zero or more.'); render(); return; }
            apply({ [name]: number }, name[0].toUpperCase() + name.slice(1));
            return;
        }
        if (name === 'drive.mode') { apply({ drive: { mode: value } }, 'Drive'); return; }
        if (name === 'drive.torque') { apply({ drive: { torque: number } }, linear ? 'Force' : 'Torque'); return; }
        if (name === 'drive.setpoint') { apply({ drive: { setpoint: fromShown(number) } }, 'Setpoint'); return; }
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
    $('#buildButton').disabled = true;
    status.textContent = 'Building…';
    try {
        const answer = await api.runImport('drawing', { step: 'build', drawing: drawing(), runTime: state.settings.runTime, timeStep: state.settings.timeStep });
        if (!answer.imported) throw new Error((answer.report?.errors ?? ['The model could not be built.']).join(' '));
        await api.openInCanvas(null, { focus: true, silent: true, session: sessionState() });
        state.built = answer.data;
        status.textContent = `Built into the canvas: ${answer.data.nodes} nodes (the Ground and ${answer.data.solids} solids) and ${answer.data.edges} relationships (the joints). Run it there; its sliders are on the joints.`;
        toast('Built into the canvas.');
    } catch (error) {
        status.textContent = `Not built: ${error.message}`;
        toast(error.message, 'error');
    } finally {
        renderChecks();
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
        change(readDrawing(answer.data.drawing), `Open ${answer.data.name}`);
        view.fit();
    } catch (error) {
        toast(error.message, 'error');
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
    ['Change a joint', 'Select it: its kind, point and axis, limits, damping and drive'],
    ['Duplicate', `Duplicate; ${keys.duplicate}`],
    ['Delete', `Delete; ${keys.deleteKeys}`],
    ['Undo, redo', `${keys.undo}; ${keys.redo}`],
    ['Back out', 'Escape: stop joining, put the handles away, then clear the selection'],
    ['Build into the canvas', `Build into canvas; ${keys.build}`],
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
$('#exampleSelect').innerHTML += examples.map(({ id, name, description }) => `<option value="${id}" title="${escapeHtml(description)}">${escapeHtml(name)}</option>`).join('');
$('#exampleSelect').addEventListener('change', (event) => {
    const chosen = examples.find(({ id }) => id === event.target.value);
    event.target.value = '';
    if (!chosen) return;
    state.selection = [];
    setTool('select');
    change(exampleDrawing(chosen.id), `Start from the ${chosen.name.toLowerCase()}`);
    view.fit();
    toast(`${chosen.name}: ${chosen.description}${state.history.undoStack.length > 1 ? ' Undo brings your drawing back.' : ''}`);
});
$('#helpButton').addEventListener('click', showHelp);
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
    if ($('#helpDialog').open) return;
    const command = commandHeld(event, keys);
    const key = event.key.toLowerCase();
    if (command && key === 'z') { event.preventDefault(); if (event.shiftKey) redo(); else undo(); return; }
    if (command && key === 'y' && !keys.mac) { event.preventDefault(); redo(); return; }
    if (command && key === 'b') { event.preventDefault(); build(); return; }
    if (typing(event.target)) return;
    if (command && key === 'd') { event.preventDefault(); duplicateSelection(); return; }
    if (command || event.altKey) return;
    if (event.key === 'Escape') {
        if (state.tool === 'join' && state.joinFirst) { state.joinFirst = null; view.showMarker('chosen', null); render(); return; }
        if (state.tool !== 'select') { setTool('select'); return; }
        select([]);
        return;
    }
    if (event.key === 'Delete' || event.key === 'Backspace') { event.preventDefault(); deleteSelection(); return; }
    const shortcuts = { b: () => addShape('box'), c: () => addShape('cylinder'), s: () => addShape('sphere'), p: () => addShape('capsule'),
        j: () => setTool('join'), g: () => setTool('move'), r: () => setTool('turn'), f: () => view.fit(), '?': showHelp };
    if (shortcuts[event.key]) { event.preventDefault(); shortcuts[event.key](); }
});

// For tests: the window's state, read only.
window.roboticsWindow = { drawing, state, view };

await restore();
