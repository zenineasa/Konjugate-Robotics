/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Milestone D without the engine: a MuJoCo model read as MuJoCo reads it (tests/fixtures/arm), its inertias worked out
// as MuJoCo's compiler works them out, the drawing an import makes of it with what it leaves out, and the importer's
// steps (a model in, its looks kept with the project, a build that shows them).

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import importDrawing from '../../packages/toolbox/importers/drawing.mjs';
import { checkDrawing, massOf, poseOf, updateJoint } from '../../packages/toolbox/lib/drawing.mjs';
import { compose, rotate } from '../../packages/toolbox/lib/frames.mjs';
import { combine, meshInertia, placed, primitiveInertia, principal } from '../../packages/toolbox/lib/inertia.mjs';
import { bodyLook, modelPaths, parseXml, readModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { modelFilesFromModel } from '../../packages/toolbox/lib/menagerie.mjs';
import { importModel } from '../../packages/toolbox/lib/mjcfImport.mjs';
import { readStl } from '../../packages/toolbox/lib/meshes.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const fixture = join(roboticsRoot, 'tests', 'fixtures', 'arm');
const files = {
    'scene.xml': await readFile(join(fixture, 'scene.xml'), 'utf8'),
    'arm.xml': await readFile(join(fixture, 'arm.xml'), 'utf8'),
    'meshes/turret.stl': new Uint8Array(await readFile(join(fixture, 'meshes', 'turret.stl')))
};
const close = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} is not ${expected}`);
const closeVector = (actual, expected, tolerance = 1e-9) => actual.forEach((value, index) => close(value, expected[index], tolerance));
const sorted = (values) => [...values].sort((a, b) => b - a);

// MuJoCo 3.16's own compiled model of tests/fixtures/arm (body_mass, body_inertia, xipos at the reference pose).
const mujoco = {
    Base: { mass: 2, inertia: [0.015, 0.01, 0.01], centre: [0, 0, 0.05] },
    Turret: { mass: 0.5, inertia: [0.00306827803539, 0.00249922480476, 0.00193249715986], centre: [0.01, 0, 0.13] },
    'Upper arm': { mass: 0.391805280711, inertia: [0.00156026937717, 0.00152601202565, 0.000114343245556], centre: [0, 0.1, 0.15382843232] },
    Forearm: { mass: 0.3, inertia: [0.0008, 0.0008, 0.0001], centre: [0, 0.29, 0.15] },
    Tip: { mass: 0.1, inertia: [0.0001, 0.0001, 0.0001], centre: [0, 0.37, 0.15] },
    Mount: { mass: 0.2, inertia: [0.0001, 0.0001, 0.0001], centre: [0.4, 0, 0.3] },
    Bob: { mass: 0.2, inertia: [0.0003, 0.0002, 0.0001], centre: [0.4, 0.0171010071663, 0.233015368961] }
};

test('a model is read as MuJoCo reads it: included files, default classes, a frame, degrees, a ref, and each body\'s inertia', () => {
    const model = readModel(files, 'scene.xml');
    assert.equal(model.name, 'Test arm');
    assert.deepEqual(model.bodies.map(({ name }) => name), ['Base', 'Turret', 'Upper arm', 'Bracket', 'Forearm', 'Tip', 'Mount', 'Bob']);
    const body = (name) => model.bodies.find((candidate) => candidate.name === name);
    for (const [name, expected] of Object.entries(mujoco)) {
        const { mass, centre, tensor } = body(name).inertia;
        close(mass, expected.mass, 1e-10);
        closeVector(sorted(principal(tensor).moments), sorted(expected.inertia), 1e-11);
        // The centre in the world, with every joint at its ref.
        closeVector(compose(body(name).world, { position: centre, quaternion: [1, 0, 0, 0] }).position, expected.centre, 1e-10);
    }
    assert.equal(body('Upper arm').inertiaFrom, 'geoms', 'with no <inertial>, from its capsule and box');
    assert.equal(body('Turret').inertiaFrom, 'inertial');
    assert.equal(body('Bracket').inertia.mass, 0, 'a bracket with only a massless look has no mass');
    const shoulder = body('Upper arm').joints[0];
    close(shoulder.reference, Math.PI / 18);
    closeVector(shoulder.range, [-Math.PI / 2, Math.PI / 2]);
    // Its axis in its body's frame, and in the world turned with the body by the <frame> it sits in (90° about z); the
    // default class's damping and armature.
    closeVector(shoulder.axis, [0, 1, 0], 1e-12);
    closeVector(rotate(body('Upper arm').world.quaternion, shoulder.axis), [-1, 0, 0], 1e-12);
    assert.equal(shoulder.damping, 0.05);
    assert.equal(shoulder.armature, 0.001);
    assert.equal(body('Tip').joints[0].damping, 2, 'its own damping over the default\'s');
    assert.deepEqual(body('Bob').joints[0].kind, 'ball');
    assert.equal(body('Bob').joints[0].damping, 0.05, 'a body outside any childclass takes the main default class');
    const servo = model.actuators.find(({ name }) => name === 'Shoulder servo');
    assert.deepEqual([servo.kind, servo.gain, servo.damping, servo.gear, servo.forceRange], ['position', 40, 0.5, 2, [-5, 5]]);
    assert.deepEqual(model.actuators.find(({ name }) => name === 'Extend servo').forceRange, [-20, 20], 'its own force range over the default class\'s');
    assert.equal(model.actuators.find(({ name }) => name === 'Waist servo').dampratio, 1);
    assert.deepEqual(model.actuators.map(({ kind }) => kind), ['position', 'position', 'motor', 'other', 'position']);
    assert.deepEqual(model.leftOut, ['the joint constraint Lock, which starts inactive']);
});

test('the files a model reads are found from its MJCF: what it includes and the meshes they name, in its meshdir', async () => {
    const asked = [];
    const fetchText = async (path) => { asked.push(path); return typeof files[path] === 'string' ? files[path] : ''; };
    assert.deepEqual(await modelPaths(fetchText, 'scene.xml'), ['scene.xml', 'arm.xml', 'meshes/turret.stl']);
    assert.deepEqual(asked, ['scene.xml', 'arm.xml'], 'only the MJCF is read to find them');
    // From Menagerie: each at the commit, on the raw file host, with the licence.
    const fromMenagerie = await modelFilesFromModel(async (url) => files[url.split('/test_arm/')[1]], 'c'.repeat(40), 'test_arm');
    assert.deepEqual(fromMenagerie.map(({ path }) => path), ['scene.xml', 'arm.xml', 'meshes/turret.stl', 'LICENSE']);
    assert.equal(fromMenagerie[2].url, `https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/${'c'.repeat(40)}/test_arm/meshes/turret.stl`);
});

test('a model this reader cannot follow is refused with the reason', () => {
    assert.throws(() => parseXml('<mujoco><worldbody></mujoco>'), /closes <mujoco> where <worldbody> is open/);
    assert.throws(() => readModel({ 'm.xml': '<mujoco><worldbody><body><joint class="nothing"/></body></worldbody></mujoco>' }, 'm.xml'), /default class nothing, which it does not define/);
    assert.throws(() => readModel({ 'm.xml': '<mujoco><worldbody><replicate count="2"/></worldbody></mujoco>' }, 'm.xml'), /<replicate>, which this reader does not take yet/);
    assert.throws(() => readModel({ 'm.xml': '<mujoco><include file="other.xml"/></mujoco>' }, 'm.xml'), /no file other.xml/);
});

test('inertias as MuJoCo\'s compiler works them out: primitives, a closed mesh, several combined, and principal axes', () => {
    // A box mesh 40 x 30 x 30 mm off its centre: the box's own volume, centre and inertia.
    const box = meshInertia(readStl(files['meshes/turret.stl']).map((triangle) => triangle.map((corner) => corner.map((value) => value / 1000))));
    close(box.volume, 0.04 * 0.03 * 0.03, 1e-15);
    closeVector(box.centre, [0.01, 0, 0.01], 1e-12);
    const exact = primitiveInertia('box', [0.02, 0.015, 0.015]).unit;
    closeVector([box.tensor[0], box.tensor[4], box.tensor[8]], exact, 1e-12);
    closeVector([box.tensor[1], box.tensor[2], box.tensor[5]], [0, 0, 0], 1e-15);
    // Two equal points apart: the parallel axis theorem.
    const pair = combine([placed(1, [0, 0, 0, 0, 0, 0, 0, 0, 0], { position: [1, 0, 0], quaternion: [1, 0, 0, 0] }), placed(1, [0, 0, 0, 0, 0, 0, 0, 0, 0], { position: [-1, 0, 0], quaternion: [1, 0, 0, 0] })]);
    closeVector(pair.tensor, [0, 0, 0, 0, 2, 0, 0, 0, 2]);
    // A tensor turned 30° about z: its moments back, and the turn to them.
    const turn = [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)];
    const { moments, quaternion } = principal(placed(1, [3, 0, 0, 0, 2, 0, 0, 0, 1], { position: [0, 0, 0], quaternion: turn }).tensor);
    closeVector(moments, [3, 2, 1], 1e-12);
    closeVector(principal(placed(1, [3, 0, 0, 0, 2, 0, 0, 0, 1], { position: [0, 0, 0], quaternion }).tensor).moments, [3, 2, 1], 1e-12);
});

test('an import makes a drawing of the model: its solids at their centres of mass, welded or joined as the model has them, driven by its actuators', () => {
    const model = readModel(files, 'scene.xml');
    const looks = model.bodies.map((body) => bodyLook(model, body));
    const { drawing, looks: solidLooks, report } = importModel(model, looks, { name: 'Test arm' });
    assert.deepEqual(checkDrawing(drawing).errors, []);
    assert.deepEqual(drawing.solids.map(({ name }) => name), ['Base', 'Turret', 'Upper arm', 'Forearm', 'Tip', 'Mount', 'Bob'], 'the massless bracket is part of the upper arm');
    for (const solid of drawing.solids) {
        const expected = mujoco[solid.name];
        assert.equal(solid.shape, 'mesh');
        close(massOf(solid), expected.mass, 1e-10);
        closeVector(sorted(solid.unitInertia.map((value) => value * massOf(solid))), sorted(expected.inertia), 1e-11);
        closeVector(solid.position, expected.centre, 1e-10);
    }
    const joint = (name) => drawing.joints.find((candidate) => candidate.name === name);
    assert.deepEqual(drawing.joints.map(({ name, kind }) => [name, kind]), [
        ['Base weld', 'fixed'], ['Waist', 'hinge'], ['Shoulder', 'hinge'], ['Elbow', 'hinge'], ['Extend', 'slider'], ['Mount weld', 'fixed'], ['Swing', 'ball']
    ]);
    // Counted from the ref, where the model puts the upper arm.
    closeVector([joint('Shoulder').limits.lower, joint('Shoulder').limits.upper], [-Math.PI / 2 - Math.PI / 18, Math.PI / 2 - Math.PI / 18]);
    assert.deepEqual(joint('Swing').limits, { lower: 0, upper: 1.047197551197 });
    // Servos: kp gear², the control's target over its gear from the ref, the force range times the gear; kv as damping.
    // Servos: kp gear², the control's target over its gear from the ref, the force range times the gear, kv gear² as
    // the servo's own damping, within its most torque.
    assert.deepEqual([joint('Shoulder').drive.mode, joint('Shoulder').drive.gain, joint('Shoulder').drive.torqueLimit, joint('Shoulder').drive.damping], ['servo', 160, 10, 2]);
    close(joint('Shoulder').drive.setpoint, -Math.PI / 18, 1e-12);
    close(joint('Shoulder').damping, 0.05);
    assert.deepEqual([joint('Waist').drive.gain, joint('Waist').drive.torqueLimit], [30, 20]);
    // A dampratio, as MuJoCo works it out: twice the root of kp times the inertia the waist moves at the reference pose
    // with every other joint free (1 / (M⁻¹)ᵢᵢ), as MuJoCo 3.16 compiles this model: kv 2.420266056.
    close(joint('Waist').drive.damping, 2.420266056, 1e-9);
    // A control of 0 held to the control range: the slider's target 5 mm out.
    assert.deepEqual([joint('Elbow').drive.mode, joint('Elbow').drive.torque], ['torque', 0]);
    assert.deepEqual([joint('Extend').drive.mode, joint('Extend').drive.gain, joint('Extend').drive.setpoint], ['servo', 400, 0.005], 'the servo after the damper');
    assert.deepEqual(report, {
        solids: 7, joints: 7, drives: 4,
        leftOut: [
            'the joint constraint Lock, which starts inactive',
            'joint Shoulder\'s ref: its position here is counted from 0.174532925199 rad, where the model puts its body',
            'Extend brake, a damper actuator the drawing cannot take yet'
        ]
    });
    // The fixed tendon, its length counted as MuJoCo counts it: from the joints' own zeros, so the shoulder's ref is in
    // its offset; nothing on it yet but its parts.
    const coupling = drawing.tendons.find(({ name }) => name === 'Coupling');
    assert.deepEqual(coupling.parts.map(({ joint: id, coef }) => [drawing.joints.find((candidate) => candidate.id === id).name, coef]), [['Shoulder', 1], ['Elbow', -1]]);
    close(coupling.offset, 0.174532925199, 1e-9);
    assert.deepEqual([coupling.stiffness, coupling.damping, coupling.friction, coupling.limits, coupling.drive.mode], [0, 0, 0, null, 'free']);
    // A look is the body's visible geoms in its solid's frame: the turret's mesh box, corner for corner, where the model
    // puts it in the world.
    const turret = drawing.solids.find(({ name }) => name === 'Turret');
    const turretBody = model.bodies.find(({ name }) => name === 'Turret');
    const pose = poseOf(drawing, turret.id);
    const corners = [];
    for (let at = 0; at < solidLooks[turret.id].mesh.position.length; at += 3) {
        corners.push(compose({ position: pose.position, quaternion: pose.rotation }, { position: solidLooks[turret.id].mesh.position.slice(at, at + 3), quaternion: [1, 0, 0, 0] }).position);
    }
    const expectedCorner = compose(turretBody.world, { position: [0.03, 0.015, 0.025], quaternion: [1, 0, 0, 0] }).position;
    assert.ok(corners.some((corner) => Math.hypot(...corner.map((value, axis) => value - expectedCorner[axis])) < 1e-9), 'the mesh\'s far corner should be where the model puts it');
    // The upper arm's own geoms are not visible (group 3), so its look is the massless bracket's box, which is part of
    // it; the mount, with nothing visible, has none.
    assert.equal(solidLooks[drawing.solids.find(({ name }) => name === 'Upper arm').id].mesh.position.length, 8 * 3);
    assert.equal(solidLooks[drawing.solids.find(({ name }) => name === 'Mount').id], undefined);
    assert.ok(drawing.solids.find(({ name }) => name === 'Mount').size.x > 0, 'a solid with no look is shown as the box of its inertia');
});

test('the importer takes a model\'s files, keeps their looks for the project, and builds the robot looking as they say', async () => {
    // As stored from Menagerie: each file named for its path, with ^ for /.
    const stored = Object.entries(files).map(([path, content]) => ({ role: 'model', name: path.split('/').join('^'), data: typeof content === 'string' ? new TextEncoder().encode(content) : content }));
    const imported = await importDrawing({ files: stored, options: { step: 'model', source: { kind: 'menagerie', folder: 'test_arm' } } });
    assert.equal(imported.ok, true, JSON.stringify(imported.report));
    assert.equal(imported.data.main, 'scene.xml');
    assert.equal(imported.data.drawing.name, 'Test arm');
    assert.deepEqual(imported.data.drawing.source, { kind: 'menagerie', folder: 'test_arm' });
    assert.match(imported.report.summary, /7 solids and 7 joints, 4 driven/);
    const looks = imported.derived.find(({ role }) => role === 'looks');
    assert.deepEqual(JSON.parse(looks.text).looks, imported.data.looks);
    // Brought in from disk, the files lose their folders: a mesh is found by its name, and the model is the MJCF no
    // other includes.
    const flat = Object.entries(files).filter(([path]) => path !== 'scene.xml').map(([path, content]) => ({ role: 'model', name: path.split('/').pop(), data: typeof content === 'string' ? new TextEncoder().encode(content) : content }));
    const fromDisk = await importDrawing({ files: flat, options: { step: 'model' } });
    assert.equal(fromDisk.ok, true, JSON.stringify(fromDisk.report));
    assert.equal(fromDisk.data.main, 'arm.xml');
    // The looks kept with the project come back for the window, and a build shows them.
    const kept = [{ role: 'looks', name: 'looks.json', text: looks.text }];
    const restored = await importDrawing({ files: kept, options: { step: 'looks' } });
    assert.deepEqual(restored.data.looks, imported.data.looks);
    const built = await importDrawing({ files: kept, options: { step: 'build', drawing: imported.data.drawing, runTime: 1 } });
    assert.equal(built.ok, true, JSON.stringify(built.report));
    const turret = imported.data.drawing.solids.find(({ name }) => name === 'Turret');
    const node = built.document.nodes.find(({ name }) => name === 'Turret');
    assert.deepEqual(node.appearance.parts[0].mesh, imported.data.looks[turret.id].mesh);
    // Without them (a drawing opened from a file), the solid is shown as the box about it.
    const plain = await importDrawing({ files: [], options: { step: 'build', drawing: imported.data.drawing, runTime: 1 } });
    assert.equal(plain.document.nodes.find(({ name }) => name === 'Turret').appearance.parts[0].mesh.index.length, 36);
    // Nothing to import, or two models with neither named, is said so.
    assert.match((await importDrawing({ files: [], options: { step: 'model' } })).report.errors[0], /Choose a robot's model first/);
});

test('a body with several joints is a chain of links with no mass, built and written back as one body', async () => {
    const { writeMjcf } = await import('../../packages/toolbox/lib/mjcfWriter.mjs');
    const { buildModel } = await import('../../packages/toolbox/lib/modelBuilder.mjs');
    const gimbal = readModel({ 'gimbal.xml': await readFile(join(roboticsRoot, 'tests', 'fixtures', 'gimbal', 'gimbal.xml'), 'utf8') }, 'gimbal.xml');
    assert.ok(!gimbal.leftOut.some((line) => /joints after its first/.test(line)), 'every joint comes in');
    const { drawing, report } = importModel(gimbal, gimbal.bodies.map((body) => bodyLook(gimbal, body)), { name: 'Gimbal' });
    assert.equal(report.joints, 8);
    const head = drawing.solids.find(({ name }) => name === 'Head');
    const links = drawing.solids.filter(({ source }) => source?.link);
    assert.deepEqual(links.map(({ name, source }) => [name, source.body]), [['Yaw link', 'Arm'], ['Twist link', 'Sleeve'], ['Turn link', 'Head'], ['Nod link', 'Head']]);
    // Each link is where its body is, so the joints after it are drawn where the model has them.
    const headBody = gimbal.bodies.find(({ name }) => name === 'Head');
    for (const link of links.filter(({ source }) => source.body === 'Head')) assert.deepEqual(poseOf(drawing, link.id).position, headBody.world.position.map((value) => Math.round(value * 1e12) / 1e12 + 0));
    assert.equal(massOf(head), 0.1);
    assert.deepEqual(checkDrawing(drawing).errors, []);
    // Springs: MuJoCo's stiffness and springref (in degrees here), the rest counted from the joint's ref.
    const jointNamed = (name) => drawing.joints.find((joint) => joint.name === name);
    assert.deepEqual([jointNamed('Pitch').stiffness, jointNamed('Pitch').springRest], [0.2, Math.round((25 * Math.PI / 180) * 1e12) / 1e12]);
    assert.deepEqual([jointNamed('Reach').stiffness, jointNamed('Reach').springRest], [30, 0.02]);
    assert.equal(jointNamed('Yaw').stiffness, undefined);
    assert.ok(!report.leftOut.some((line) => /spring/.test(line)), 'a hinge\'s and a slider\'s springs come in');
    assert.match(checkDrawing(updateJoint(drawing, jointNamed('Pitch').id, { stiffness: -1 })).errors.map(({ message }) => message).join(' '), /Pitch: its spring's stiffness must be zero or more/);
    assert.match(writeMjcf(drawing), new RegExp(`<joint name="${jointNamed('Reach').id}" [^>]*stiffness="30" springref="0.02"`));
    assert.ok(buildModel(drawing, { timeStep: 0.001, runTime: 1 }).document.nodes.some(({ name }) => name === 'Nod link'));
    // Written back, Head's body holds its three joints in the model's order, and no link is a body.
    const mjcf = writeMjcf(drawing);
    const headXml = mjcf.slice(mjcf.indexOf(`<body name="${head.id}"`));
    const names = ['Turn', 'Nod', 'Roll'].map((name) => drawing.joints.find((joint) => joint.name === name).id);
    const at = names.map((id) => headXml.indexOf(`<joint name="${id}"`));
    assert.ok(at.every((index) => index > 0 && index < headXml.indexOf('</body>')) && at[0] < at[1] && at[1] < at[2], 'Head holds Turn, Nod and Roll in turn');
    for (const link of links) assert.ok(!mjcf.includes(`<body name="${link.id}"`));
});
