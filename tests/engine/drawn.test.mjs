/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Drawn robots, built as the window builds them (lib/modelBuilder.mjs) and run by Konjugate's engine alone: each solid
// a node, each joint a relationship carrying the articulated-body equations. They move as a second, independent
// solution of the same drawing moves (newtonEuler.mjs: the recursive Newton-Euler method, stepped the same way), to
// within rounding; their live parameters reach the motion; a joint stops at its limit; and a run resumed from a
// checkpoint carries on exactly. MuJoCo run on its own gives a third opinion, where a Python with MuJoCo is at hand.
//
// Needs Konjugate's engine build (see harness.mjs).

import assert from 'node:assert/strict';
import { mkdtemp } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { exampleDrawing, examples } from '../../packages/toolbox/lib/examples.mjs';
import { addFrameAt, addJoint, addKeyframe, addSolid, createDrawing, massOf, reverseJoint, structure, updateBattery, updateJoint, updateSolid } from '../../packages/toolbox/lib/drawing.mjs';
import { between, fromAxisAngle, scale } from '../../packages/toolbox/lib/math.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { motorById, servosToMotors } from '../../packages/toolbox/lib/motors.mjs';
import { konjugateModule } from '../../scripts/konjugatePaths.mjs';
import { mujocoMissing, runDocument, runReference } from './harness.mjs';
import { inverseDynamics, restCoordinates, simulate, treeOf } from './newtonEuler.mjs';

const directory = await mkdtemp(join(tmpdir(), 'robotics-drawn-'));
const build = (drawing, runTime = 3) => buildModel(drawing, { timeStep: 0.001, runTime }).document;
const largestDifference = (values, reference) => Math.max(...values.map((value, index) => Math.abs(value - reference[index])));
const jointNamed = (drawing, name) => drawing.joints.find((joint) => joint.name === name);
const solidNamed = (drawing, name) => drawing.solids.find((solid) => solid.name === name);
const coordinate = (joint) => (joint.kind === 'slider' ? 'jointPosition' : 'jointAngle');
const withoutMujoco = await mujocoMissing();

// A quaternion's rotation matrix, row by row.
const matrixOf = ([w, x, y, z]) => [
    [w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z]
];

// Runs `drawing` in the engine and in the Newton-Euler reference; every moving joint's coordinates (a ball's turn too)
// and every solid's centre and rotation agree.
async function matchesReference(name, drawing, carried, runTime = 3, document = build(drawing, runTime), tolerance = 1e-9) {
    const run = await runDocument(directory, name, document);
    const reference = simulate(drawing, run.times);
    for (const [jointName, solidName] of Object.entries(carried)) {
        const joint = jointNamed(drawing, jointName);
        if (joint.kind === 'ball') {
            ['jointTurnW', 'jointTurnX', 'jointTurnY', 'jointTurnZ'].forEach((symbol, index) => {
                const values = run.series(`${solidName}.${symbol}`);
                const difference = largestDifference(values, reference[joint.id].map((turn) => turn[index]));
                assert.ok(difference < tolerance, `${name}: ${jointName}'s ${symbol} differs from the reference by ${difference}.`);
            });
            continue;
        }
        const values = run.series(`${solidName}.${coordinate(joint)}`);
        assert.ok(largestDifference(values, reference[joint.id]) < tolerance, `${name}: ${jointName} differs from the reference by ${largestDifference(values, reference[joint.id])}.`);
    }
    for (const solid of drawing.solids) {
        for (const [axis, symbol] of ['x', 'y', 'z'].entries()) {
            const values = run.series(`${solid.name}.${symbol}`);
            const expected = reference[solid.id].map((place) => place[axis]);
            assert.ok(largestDifference(values, expected) < tolerance, `${name}: ${solid.name}'s ${symbol} differs from the reference by ${largestDifference(values, expected)} m.`);
        }
        const matrices = reference.rotations[solid.id].map(matrixOf);
        for (const row of [0, 1, 2]) for (const column of [0, 1, 2]) {
            const values = run.series(`${solid.name}.r${row + 1}${column + 1}`);
            const difference = largestDifference(values, matrices.map((matrix) => matrix[row][column]));
            assert.ok(difference < tolerance, `${name}: ${solid.name}'s rotation r${row + 1}${column + 1} differs from the reference by ${difference}.`);
        }
    }
    return { run, reference };
}

test('a drawn pendulum swings as the Newton-Euler reference swings it, and keeps its energy', async () => {
    let drawing = exampleDrawing('pendulum');
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0 });
    const { run } = await matchesReference('pendulum', drawing, { Pivot: 'Arm' }, 4);
    const heights = run.series('Arm.z');
    assert.ok(Math.min(...heights) < solidNamed(drawing, 'Arm').position[2] - 0.05, 'Let go level, the arm should swing down.');
    // Undamped, its energy stays within a tenth of a percent of where it started (the step's own error).
    const kinetic = run.series('Arm.kineticEnergy');
    const potential = run.series('Arm.potentialEnergy');
    const energy = kinetic.map((value, index) => value + potential[index]);
    const swing = Math.max(...kinetic);
    assert.ok(swing > 0.1, 'It should swing.');
    assert.ok(Math.max(...energy) - Math.min(...energy) < 0.01 * swing, `Its energy wandered by ${Math.max(...energy) - Math.min(...energy)} J.`);
});

test('a joint\'s torque, a live parameter of its edge, reaches the joint: m g L holds the arm level', async () => {
    let drawing = exampleDrawing('pendulum');
    const arm = solidNamed(drawing, 'Arm');
    // The arm's middle is 0.1 m out along x from the hinge, whose axis is y: gravity turns it about +y by m g 0.1.
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0, drive: { mode: 'torque', torque: -massOf(arm) * 9.81 * 0.1 } });
    const angles = (await runDocument(directory, 'held', build(drawing, 1))).series('Arm.jointAngle');
    assert.ok(Math.max(...angles.map(Math.abs)) < 1e-9, `Held by its torque, it moved ${Math.max(...angles.map(Math.abs))} rad.`);
});

test('servos hold a drawn two-link arm at its setpoints, moving as the reference moves it', async () => {
    const drawing = exampleDrawing('twoLinkArm');
    const { run } = await matchesReference('arm', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' });
    const elbow = run.series('Forearm.jointAngle').at(-1);
    assert.ok(Math.abs(elbow - 0.5) < 0.01, `The elbow should settle near its 0.5 rad setpoint, not ${elbow}.`);
});

test('a cart on a slider carries two pendulums, a branch of the tree each, as the reference does', async () => {
    const drawing = exampleDrawing('cartPendulums');
    const carried = { Track: 'Cart', 'Front swing': 'Front pendulum', 'Back swing': 'Back pendulum' };
    const { run } = await matchesReference('cart', drawing, carried);
    const back = run.series('Back pendulum.jointAngle');
    assert.ok(Math.max(...back.map(Math.abs)) > 0.1, 'The cart starting off should set the hanging pendulum swinging.');
    // The front swing joint read the other way round (its point and axis on the pendulum): the same robot, the same motion.
    const reversed = reverseJoint(drawing, jointNamed(drawing, 'Front swing').id);
    const { run: again } = await matchesReference('cartReversed', reversed, carried);
    assert.ok(largestDifference(again.series('Front pendulum.x'), run.series('Front pendulum.x')) < 1e-12);
});

test('an arm with a gripper turns at its waist and closes its fingers, held up against gravity, as the reference moves it', async () => {
    const drawing = exampleDrawing('gripperArm');
    const carried = {
        Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm', Wrist: 'Hand', 'Left finger slide': 'Left finger', 'Right finger slide': 'Right finger'
    };
    const { run } = await matchesReference('gripper', drawing, carried);
    const waist = run.series('Turret.jointAngle').at(-1);
    assert.ok(Math.abs(waist - 0.8) < 0.02, `The waist should turn to its 0.8 rad setpoint, not ${waist}.`);
    for (const [joint, solid] of [['Shoulder', 'Upper arm'], ['Elbow', 'Forearm'], ['Wrist', 'Hand']]) {
        const sag = Math.max(...run.series(`${solid}.jointAngle`).map(Math.abs));
        assert.ok(sag < 0.05, `The ${joint} servo should hold the arm where it is drawn against gravity, but it gave ${sag} rad.`);
    }
    for (const finger of ['Left finger', 'Right finger']) {
        const travel = run.series(`${finger}.jointPosition`).at(-1);
        assert.ok(Math.abs(travel + 0.015) < 0.002, `The ${finger.toLowerCase()} should close by 15 mm, not ${travel * 1000} mm.`);
    }
    const gap = (index) => Math.hypot(...['x', 'y', 'z'].map((axis) => run.series(`Left finger.${axis}`).at(index) - run.series(`Right finger.${axis}`).at(index)));
    assert.ok(gap(-1) < gap(0) - 0.025, `The fingers should close towards each other, from ${gap(0)} m apart to ${gap(-1)} m.`);
});

test('two links on ball joints swing and twist in three dimensions as the reference swings them, losing energy only to their damping', async () => {
    const drawing = exampleDrawing('ballPendulum');
    // Over a second: two linked spherical pendulums are chaotic, so rounding grows on longer runs.
    const { run } = await matchesReference('ball', drawing, { Shoulder: 'Upper link', Elbow: 'Lower link' }, 1);
    // Let go out of line, the lower link leaves every plane through the shoulder: its centre's three coordinates all move.
    for (const axis of ['x', 'y', 'z']) {
        const values = run.series(`Lower link.${axis}`);
        assert.ok(Math.max(...values) - Math.min(...values) > 0.02, `The lower link should move along ${axis}.`);
    }
    // Its quaternions keep unit length to within the step's error; its energy only falls (damped) beyond that error.
    const turn = ['W', 'X', 'Y', 'Z'].map((axis) => run.series(`Upper link.jointTurn${axis}`));
    const lengths = turn[0].map((_, index) => Math.hypot(...turn.map((series) => series[index])));
    assert.ok(Math.max(...lengths.map((value) => Math.abs(value - 1))) < 0.01, 'The turn drifted from unit length.');
    const energy = run.times.map((_, index) => ['Upper link', 'Lower link'].reduce((sum, name) => sum + run.series(`${name}.kineticEnergy`)[index] + run.series(`${name}.potentialEnergy`)[index], 0));
    const swing = Math.max(...run.series('Lower link.kineticEnergy'));
    assert.ok(Math.max(...energy) - energy[0] < 0.02 * swing, `Its energy rose by ${Math.max(...energy) - energy[0]} J.`);
});

// A body joined to nothing, with an arm hinged to it that a constant torque turns: they fall together, the body turning
// back against the arm.
function floatingDrawing() {
    let drawing = createDrawing('Floating');
    let added = addSolid(drawing, 'box', { name: 'Body', size: { x: 0.2, y: 0.12, z: 0.08 }, position: [0, 0, 1] });
    drawing = updateSolid(added.drawing, added.id, { rotation: fromAxisAngle([0.3, 0.2, 1], 0.4) });
    const body = added.id;
    const from = [0.1, 0, 1.05];
    const to = [0.3, 0.05, 1.1];
    const along = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
    added = addSolid(drawing, 'capsule', { name: 'Arm', size: { radius: 0.015, length: Math.hypot(...along) }, position: scale([from[0] + to[0], from[1] + to[1], from[2] + to[2]], 0.5) });
    drawing = updateSolid(added.drawing, added.id, { rotation: between([0, 0, 1], along) });
    const arm = added.id;
    const onBody = addFrameAt(drawing, body, from, [0, 1, 0]);
    const onArm = addFrameAt(onBody.drawing, arm, from, [0, -1, 0]);
    const joined = addJoint(onArm.drawing, onBody.id, onArm.id, { kind: 'hinge', mate: false, name: 'Shoulder' });
    return updateJoint(joined.drawing, joined.id, { damping: 0.001, drive: { mode: 'torque', torque: 0.02 } });
}

test('a body joined to nothing falls on a free joint, turning back as its arm turns, as the reference has it', async () => {
    const drawing = floatingDrawing();
    const { run } = await matchesReference('floating', drawing, { Shoulder: 'Arm' }, 1);
    assert.equal(build(drawing, 1).edges.find(({ name }) => name === 'Body floating').source.nodeId, build(drawing, 1).nodes.find(({ name }) => name === 'Ground').id);
    // Their centre of mass falls as g falls it, to within half a percent (the step's own error), and goes nowhere
    // sideways.
    const masses = drawing.solids.map(massOf);
    const total = masses[0] + masses[1];
    const centre = (axis, index) => drawing.solids.reduce((sum, solid, k) => sum + masses[k] * run.series(`${solid.name}.${axis}`)[index], 0) / total;
    const last = run.times.length - 1;
    const time = run.times[last];
    const fallen = centre('z', 0) - centre('z', last);
    assert.ok(Math.abs(fallen - 0.5 * 9.81 * time * time) < 5e-3 * fallen, `The centre of mass fell ${fallen} m, not as gravity alone falls it.`);
    for (const axis of ['x', 'y']) assert.ok(Math.abs(centre(axis, last) - centre(axis, 0)) < 1e-3, `The centre of mass moved ${centre(axis, last) - centre(axis, 0)} m along ${axis}.`);
    // The arm turned one way about the shoulder, so the body turned the other way about its own axis there.
    assert.ok(run.series('Arm.jointAngle')[last] > 1, 'The torque should turn the arm.');
    assert.ok(Math.abs(run.series('Body.w2')[last]) > 0.1 || Math.abs(run.series('Body.w1')[last]) > 0.1, 'The body should turn back against the arm.');
});

test('a four-bar linkage, a loop closed by its last pin, turns as the reference turns it, its pin held together', async () => {
    const drawing = exampleDrawing('fourBar');
    const carried = { 'Crank pivot': 'Crank', 'Rocker pivot': 'Rocker', 'Coupler pin': 'Coupler' };
    const { run } = await matchesReference('fourBar', drawing, carried, 3);
    assert.equal(structure(drawing).loops[0].name, 'Closing pin');
    // The crank goes all the way round, the rocker only rocks, and the pin's two halves stay within a twentieth of a
    // millimetre of each other, closing to nothing when the linkage slows.
    assert.ok(run.series('Crank.jointAngle').at(-1) > 2 * Math.PI, 'The crank should turn all the way round.');
    const rocker = run.series('Rocker.jointAngle');
    assert.ok(Math.max(...rocker) - Math.min(...rocker) > 0.3 && Math.max(...rocker.map(Math.abs)) < 1.5, 'The rocker should rock back and forth.');
    const gap = run.series('Rocker.closingPinGap');
    assert.ok(Math.max(...gap) < 5e-5, `The pin opened by ${Math.max(...gap)} m.`);
    assert.ok(Math.max(...run.series('Rocker.closingPinForce3').map(Math.abs)) > 0.01, 'The pin should carry a force.');
    // The force that closes it is out of the linkage's plane by nothing: the other pins already hold that.
    assert.ok(Math.max(...run.series('Rocker.closingPinForce2').map(Math.abs)) < 1e-6, 'The pin should need no force out of the plane.');
});

test('an arm pinned to the Ground at both ends, a loop through the Ground, holds still', async () => {
    let drawing = exampleDrawing('pendulum');
    const arm = solidNamed(drawing, 'Arm').id;
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0 });
    let frame = addFrameAt(drawing, 'ground', [0.2, 0.035, 0.18], [0, 1, 0]);
    const onGround = frame.id;
    frame = addFrameAt(frame.drawing, arm, [0.2, 0.035, 0.18], [0, -1, 0]);
    drawing = addJoint(frame.drawing, onGround, frame.id, { kind: 'hinge', mate: false, name: 'Pin' }).drawing;
    // Walked from the Ground, the arm hangs from the new pin, and the pivot closes the loop.
    assert.equal(structure(drawing).loops[0].name, 'Pivot');
    const { run } = await matchesReference('pinned', drawing, { Pin: 'Arm' }, 1);
    for (const axis of ['x', 'y', 'z']) {
        const values = run.series(`Arm.${axis}`);
        assert.ok(Math.max(...values) - Math.min(...values) < 1e-6, `Held at both ends, the arm should not move along ${axis}.`);
    }
    // The pivot holds half the arm's weight.
    const holding = run.series('Arm.pivotForce3').at(-1);
    assert.ok(Math.abs(Math.abs(holding) - massOf(solidNamed(drawing, 'Arm')) * 9.81 / 2) < 1e-3, `The pivot should hold half the arm's weight, not ${holding} N.`);
});

// The same, with a dry friction on the pin: its rows are found in the joints' coordinates, where the pivot's five rows
// hold more than the arm's one coordinate can move, so taking them out leaves nothing free.
test('an arm pinned at both ends with a dry friction on its pin, its loop holding more than it can move, holds still as the reference holds it', async () => {
    let drawing = exampleDrawing('pendulum');
    const arm = solidNamed(drawing, 'Arm').id;
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0 });
    let frame = addFrameAt(drawing, 'ground', [0.2, 0.035, 0.18], [0, 1, 0]);
    const onGround = frame.id;
    frame = addFrameAt(frame.drawing, arm, [0.2, 0.035, 0.18], [0, -1, 0]);
    drawing = addJoint(frame.drawing, onGround, frame.id, { kind: 'hinge', mate: false, name: 'Pin' }).drawing;
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pin').id, { friction: 0.01 });
    const { run } = await matchesReference('pinnedFriction', drawing, { Pin: 'Arm' }, 1);
    const values = run.series('Arm.z');
    assert.ok(Math.max(...values) - Math.min(...values) < 1e-6, 'Held at both ends, the arm should not move.');
});

// Drawing helpers for the loops below, as a user's clicks make them.
function placed(drawing, shape, { name, size, position, rotation }) {
    const added = addSolid(drawing, shape, { name, size, position });
    return { drawing: updateSolid(added.drawing, added.id, { position, ...(rotation ? { rotation } : {}) }), id: added.id };
}
function joined(drawing, parent, child, point, axis, kind, name) {
    const onParent = addFrameAt(drawing, parent, point, axis);
    const onChild = addFrameAt(onParent.drawing, child, point, scale(axis, -1));
    return addJoint(onChild.drawing, onParent.id, onChild.id, { kind, mate: false, name });
}
function rodBetween(drawing, name, from, to, radius = 0.008) {
    const along = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
    return placed(drawing, 'capsule', { name, size: { radius, length: Math.hypot(...along) }, position: scale([from[0] + to[0], from[1] + to[1], from[2] + to[2]], 0.5), rotation: between([0, 0, 1], along) });
}

test('two loops in one robot, a four-bar and a dyad hung from its coupler, move as the reference moves them, together', async () => {
    let drawing = exampleDrawing('fourBar');
    const coupler = solidNamed(drawing, 'Coupler');
    const base = solidNamed(drawing, 'Base').id;
    const followerPivot = [0.34, 0, 0.2];
    const followerPin = [0.33, 0, 0.36];
    let result = rodBetween(drawing, 'Follower', followerPivot, followerPin);
    drawing = joined(result.drawing, base, result.id, followerPivot, [0, 1, 0], 'hinge', 'Follower pivot').drawing;
    const follower = result.id;
    result = rodBetween(drawing, 'Link', coupler.position, followerPin);
    drawing = joined(result.drawing, coupler.id, result.id, coupler.position, [0, 1, 0], 'hinge', 'Link pin').drawing;
    drawing = joined(drawing, result.id, follower, followerPin, [0, 1, 0], 'hinge', 'Follower pin').drawing;
    assert.equal(structure(drawing).loops.length, 2);
    const { run, reference } = await matchesReference('sixBar', drawing, { 'Crank pivot': 'Crank', 'Rocker pivot': 'Rocker', 'Follower pivot': 'Follower' }, 2);
    assert.ok(Math.max(...reference.gaps) < 5e-5, `A pin opened by ${Math.max(...reference.gaps)} m.`);
    assert.ok(run.series('Crank.jointAngle').at(-1) > Math.PI, 'The crank should turn.');
    const follows = run.series('Follower.jointAngle');
    assert.ok(Math.max(...follows) - Math.min(...follows) > 0.1, 'The second loop should move with the first.');
});

test('a fixed joint closing a loop welds its solids: two arms on one axis, welded to a bar, swing as one', async () => {
    let drawing = createDrawing('Welded arms');
    let result = placed(drawing, 'box', { name: 'Post', size: { x: 0.04, y: 0.1, z: 0.3 }, position: [0, 0, 0.15] });
    drawing = joined(result.drawing, 'ground', result.id, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
    const post = result.id;
    const arms = [];
    for (const [name, side] of [['Front arm', 0.04], ['Back arm', -0.04]]) {
        result = rodBetween(drawing, name, [0, side, 0.28], [0.2, side, 0.2]);
        drawing = joined(result.drawing, post, result.id, [0, side, 0.28], [0, 1, 0], 'hinge', `${name} pivot`).drawing;
        arms.push(result.id);
    }
    result = placed(drawing, 'box', { name: 'Bar', size: { x: 0.02, y: 0.1, z: 0.02 }, position: [0.2, 0, 0.2] });
    drawing = joined(result.drawing, arms[0], result.id, [0.2, 0.04, 0.2], [0, 1, 0], 'fixed', 'Front weld').drawing;
    drawing = joined(drawing, result.id, arms[1], [0.2, -0.04, 0.2], [0, 1, 0], 'fixed', 'Back weld').drawing;
    assert.equal(structure(drawing).loops[0].kind, 'fixed');
    const { run } = await matchesReference('welded', drawing, { 'Front arm pivot': 'Front arm', 'Back arm pivot': 'Back arm' }, 1.5);
    assert.ok(largestDifference(run.series('Front arm.jointAngle'), run.series('Back arm.jointAngle')) < 1e-6, 'Welded, the two arms should swing as one.');
    assert.ok(Math.max(...run.series('Front arm.jointAngle')) > 0.5, 'They should swing.');
    assert.ok(Math.max(...run.series('Back arm.backWeldTwist')) < 1e-6, 'The weld should not twist.');
});

test('a slider closing a loop keeps its block on its rail: a crank and connecting rod drive it back and forth', async () => {
    const drawing = exampleDrawing('crankSlider');
    assert.equal(structure(drawing).loops[0].name, 'Guide');
    const { run } = await matchesReference('crankSlider', drawing, { 'Crank pivot': 'Crank', 'Crank pin': 'Connecting rod', 'Wrist pin': 'Block' }, 2);
    for (const axis of ['y', 'z']) {
        const values = run.series(`Block.${axis}`);
        assert.ok(Math.max(...values) - Math.min(...values) < 1e-4, `The block should stay on the rail's line, not move ${Math.max(...values) - Math.min(...values)} m along ${axis}.`);
    }
    const along = run.series('Block.x');
    assert.ok(Math.max(...along) - Math.min(...along) > 0.06, 'The block should travel along the rail, by twice the crank.');
    assert.ok(Math.max(...run.series('Block.guideTwist')) < 1e-6, 'The block should not turn on its rail.');
});

test('a hinge closing a loop holds its axis, not just its point: an arm on a ball, pinned by a hinge across it, holds still', async () => {
    const pinnedArm = (kind) => {
        let drawing = createDrawing('Door');
        let result = placed(drawing, 'box', { name: 'Post', size: { x: 0.02, y: 0.02, z: 0.3 }, position: [-0.01, 0, 0.15] });
        drawing = joined(result.drawing, 'ground', result.id, [-0.01, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        const post = result.id;
        // The bottom pin's own post, so the tree reaches the arm through the top one and the bottom pin closes the loop.
        result = placed(drawing, 'box', { name: 'Second post', size: { x: 0.02, y: 0.02, z: 0.3 }, position: [0.21, 0, 0.15] });
        drawing = joined(result.drawing, 'ground', result.id, [0.21, 0, 0], [0, 0, 1], 'fixed', 'Second mount').drawing;
        const secondPost = result.id;
        result = placed(drawing, 'box', { name: 'Arm', size: { x: 0.2, y: 0.06, z: 0.02 }, position: [0.1, 0.03, 0.29] });
        const arm = result.id;
        drawing = joined(result.drawing, post, arm, [0, 0, 0.3], [1, 0, 0], 'ball', 'Top').drawing;
        // The hinge's axis runs across the line between the two points, so a hinge there stops the arm turning about that
        // line, as a door's second hinge would if it were turned across the door.
        return joined(drawing, secondPost, arm, [0.2, 0, 0.3], [0, 1, 0], kind, 'Bottom').drawing;
    };
    const held = pinnedArm('hinge');
    assert.equal(structure(held).loops[0].name, 'Bottom');
    const { run } = await matchesReference('heldDoor', held, { Top: 'Arm' }, 1);
    assert.ok(Math.max(...run.series('Arm.z').map((value) => Math.abs(value - 0.29))) < 1e-6, 'Held by the hinge across it, the arm should not swing.');
    const swinging = pinnedArm('ball');
    // Two points held on one solid ask for no force along the line between them, which the compliance leaves to rounding:
    // to within 1e-8.
    const { run: free } = await matchesReference('swingingDoor', swinging, { Top: 'Arm' }, 1, build(swinging, 1), 1e-8);
    assert.ok(Math.max(...free.series('Arm.z').map((value) => Math.abs(value - 0.29))) > 0.005, 'Held at two points only, the arm should swing about the line between them.');
});

test('a ball joint is driven: a torque turns it, a servo turns it to its setpoint, and its limit stops it at its most turn', async () => {
    const base = exampleDrawing('ballPendulum');
    const shoulder = jointNamed(base, 'Shoulder').id;
    const elbow = jointNamed(base, 'Elbow').id;
    const turnOf = (run) => {
        const [w, x, y, z] = ['W', 'X', 'Y', 'Z'].map((axis) => run.series(`Upper link.jointTurn${axis}`));
        return w.map((value, index) => {
            const size = Math.hypot(x[index], y[index], z[index]);
            const angle = 2 * Math.atan2(size, Math.abs(value));
            return [x[index], y[index], z[index]].map((component) => (size ? (component / size) * angle * Math.sign(value || 1) : 0));
        });
    };
    let drawing = updateJoint(base, shoulder, { damping: 0.03, drive: { mode: 'servo', setpoint: [0.3, -0.4, 0.2], gain: 40, torqueLimit: 5 } });
    drawing = updateJoint(drawing, elbow, { damping: 0.02 });
    const { run: servo } = await matchesReference('ballServo', drawing, { Shoulder: 'Upper link', Elbow: 'Lower link' }, 3);
    const reached = turnOf(servo).at(-1);
    assert.ok(Math.hypot(reached[0] - 0.3, reached[1] + 0.4, reached[2] - 0.2) < 0.02, `The servo should turn the link to its setpoint, not to ${reached}.`);
    // Driven hard, the two links whirl, and rounding grows fast (they are chaotic): to within 1e-8 over half a second.
    const torqued = updateJoint(base, shoulder, { drive: { mode: 'torque', torque: [0.01, -0.02, 0.015] } });
    await matchesReference('ballTorque', torqued, { Shoulder: 'Upper link', Elbow: 'Lower link' }, 0.5, build(torqued, 0.5), 1e-8);
    // The reference has no limits; the limit is checked by what it does.
    const limited = updateJoint(base, shoulder, { limits: { lower: 0, upper: 0.3 } });
    const stopped = await runDocument(directory, 'ballLimit', build(limited, 2));
    const angles = turnOf(stopped).map((vector) => Math.hypot(...vector));
    assert.ok(Math.max(...angles) > 0.28 && Math.max(...angles) < 0.34, `The limit should stop the turn at 0.3 rad, not ${Math.max(...angles)}.`);
});

// The two-link arm with its shoulder and elbow on motors, keyframed with its waist servo from rest to a pose.
function motorArm() {
    let drawing = exampleDrawing('twoLinkArm');
    const id = (name) => jointNamed(drawing, name).id;
    drawing = updateJoint(drawing, id('Shoulder'), { drive: { mode: 'motor', setpoint: 0 } });
    drawing = updateJoint(drawing, id('Elbow'), { drive: { mode: 'motor', setpoint: 0 } });
    drawing = addKeyframe(drawing, { time: 0, setpoints: { [id('Waist')]: 0, [id('Shoulder')]: 0, [id('Elbow')]: 0 } }).drawing;
    return addKeyframe(drawing, { time: 1, setpoints: { [id('Waist')]: 0.5, [id('Shoulder')]: 0.4, [id('Elbow')]: 0.8 } }).drawing;
}

test('a joint on a motor follows its keyframes, drawing current from the battery and heating its winding, as the reference has it', async () => {
    const drawing = motorArm();
    const { run, reference } = await matchesReference('motorArm', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' }, 2);
    for (const name of ['Shoulder', 'Elbow']) {
        const motor = reference.motors[jointNamed(drawing, name).id];
        for (const [state, key] of [['duty', 'duty'], ['current', 'current'], ['torque', 'torque'], ['windingTemperature', 'temperature']]) {
            const difference = largestDifference(run.series(`${name} motor.${state}`), motor[key]);
            assert.ok(difference < 1e-9, `${name}'s motor's ${state} differs from the reference by ${difference}.`);
        }
    }
    assert.ok(largestDifference(run.series('Battery.stateOfCharge'), reference.charge) < 1e-12, 'The battery\'s charge differs from the reference.');
    // Each joint reaches its last keyframe; the waist's servo moves along its schedule, in a straight line to it.
    const end = run.times.length - 1;
    assert.ok(Math.abs(run.series('Upper arm.jointAngle')[end] - 0.4) < 0.01 && Math.abs(run.series('Forearm.jointAngle')[end] - 0.8) < 0.01, 'The motors should reach their keyframes.');
    const half = run.times.findIndex((time) => Math.abs(time - 0.5) < 1e-9);
    assert.ok(Math.abs(run.series('Turret.jointAngle')[half] - 0.25) < 0.01, 'Halfway, the waist should be halfway to its keyframe.');
    // Held still at the end, each motor's torque holds its joint up against gravity: the torque the arm's own weight
    // needs there, as the reference works it out at that pose.
    const bodies = treeOf(drawing);
    const q = restCoordinates(bodies);
    const speeds = bodies.reduce((sum, body) => sum + (body.kind === 'fixed' ? 0 : 1), 0);
    bodies.forEach((body, index) => { if (body.kind === 'hinge') q[index] = [run.series(`${drawing.solids.find(({ id }) => id === body.id).name}.jointAngle`)[end]]; });
    const needed = inverseDynamics(bodies, q, Array(speeds).fill(0), Array(speeds).fill(0));
    const elbowIndex = bodies.filter(({ kind }) => kind !== 'fixed').findIndex(({ joint }) => joint.name === 'Elbow');
    assert.ok(Math.abs(run.series('Elbow motor.torque')[end] - needed[elbowIndex]) < 0.01 * Math.abs(needed[elbowIndex]) + 1e-3,
        `The elbow's motor should hold ${needed[elbowIndex]} N·m, not ${run.series('Elbow motor.torque')[end]}.`);
    // The schedule is the keyframes', a shared parameter like any other.
    const document = build(drawing, 2);
    const shoulder = document.sharedParameters.find(({ name }) => name === 'Shoulder setpoint');
    assert.deepEqual(shoulder.schedule, { interpolation: 'linear', samples: [[0, 0], [1, 0.4]] });
});

// The two-link arm's servos made Feetech STS3215s on the SO-ARM100 kit's 5 V supply (lib/motors.mjs), keyframed
// from rest to a pose: the gears passing on part of the motor's torque, a supply that does not sag.
test('servos made catalog motors run as the reference runs them, their gears passing on part of the torque, on a supply held to its most current', async () => {
    let drawing = exampleDrawing('twoLinkArm');
    const id = (name) => jointNamed(drawing, name).id;
    drawing = addKeyframe(drawing, { time: 0, setpoints: { [id('Waist')]: 0, [id('Shoulder')]: 0, [id('Elbow')]: 0 } }).drawing;
    drawing = addKeyframe(drawing, { time: 1, setpoints: { [id('Waist')]: 0.5, [id('Shoulder')]: 0.3, [id('Elbow')]: 0.6 } }).drawing;
    const done = servosToMotors(drawing, 'sts3215-c001', 'mains5');
    // A supply of 0.8 A, less than the kit's, so the arm's start asks more of it than it gives.
    drawing = updateBattery(done.drawing, { currentLimit: 0.8 });
    assert.deepEqual(done.turned.map(({ name }) => name), ['Waist', 'Shoulder', 'Elbow']);
    assert.ok(drawing.joints.every((joint) => joint.drive.mode !== 'servo'));
    assert.ok(drawing.joints.find(({ name }) => name === 'Shoulder').drive.motor.gearEfficiency < 1);
    const { run, reference } = await matchesReference('stsArm', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' }, 2);
    for (const name of ['Waist', 'Shoulder', 'Elbow']) {
        const motor = reference.motors[jointNamed(drawing, name).id];
        for (const [state, key] of [['current', 'current'], ['torque', 'torque'], ['supplyVoltage', 'voltage'], ['windingTemperature', 'temperature']]) {
            const difference = largestDifference(run.series(`${name} motor.${state}`), motor[key]);
            assert.ok(difference < 1e-9, `${name}'s motor's ${state} differs from the reference by ${difference}.`);
        }
    }
    // The mains do not run down: the supply stays at 5 V. Starting off, the three motors would draw more than its 0.8 A,
    // so it gives 0.8 A and its voltage at the motors sags; it never gives more.
    assert.ok(Math.max(...run.series('Battery.voltage').map((volts) => Math.abs(volts - 5))) < 1e-9, 'The supply should stay at 5 V.');
    const given = run.times.map((_, index) => ['Waist', 'Shoulder', 'Elbow'].reduce((sum, name) => sum + run.series(`${name} motor.duty`)[index] * run.series(`${name} motor.current`)[index], 0));
    assert.ok(Math.max(...given) <= 0.8 + 1e-9, `The supply should give no more than 0.8 A, not ${Math.max(...given)}.`);
    assert.ok(Math.max(...given) > 0.8 - 1e-9 && Math.min(...run.series('Shoulder motor.supplyVoltage')) < 4.9, `The supply should reach its most current and sag: it gives up to ${Math.max(...given)} A, down to ${Math.min(...run.series('Shoulder motor.supplyVoltage'))} V.`);
    // Held still at the end, a motor gives no more than its stall torque on 5 V: 5 / 7.4 of the datasheet's.
    const stall = (motorById('sts3215-c001').stallTorque * 5) / 7.4;
    assert.ok(Math.max(...run.series('Shoulder motor.torque').map(Math.abs)) <= stall + 1e-9, 'The shoulder should give no more than its stall torque.');
    const end = run.times.length - 1;
    assert.ok(Math.abs(run.series('Upper arm.jointAngle')[end] - 0.3) < 0.02 && Math.abs(run.series('Forearm.jointAngle')[end] - 0.6) < 0.02, 'The servos made motors should reach their keyframes.');
});

// The pick-and-place arm on a battery with an internal resistance: its voltage at the motors sags as they draw, as the
// reference has it.
test('a battery with an internal resistance sags under its motors\' load, as the reference has it', async () => {
    const drawing = updateBattery(exampleDrawing('pickAndPlace'), { internalResistance: 0.5 });
    const carried = { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm', Wrist: 'Hand', 'Left finger slide': 'Left finger', 'Right finger slide': 'Right finger' };
    const { run, reference } = await matchesReference('saggingBattery', drawing, carried, 3);
    for (const name of ['Waist', 'Shoulder', 'Elbow', 'Wrist']) {
        const motor = reference.motors[jointNamed(drawing, name).id];
        for (const [state, key] of [['current', 'current'], ['supplyVoltage', 'voltage']]) {
            const difference = largestDifference(run.series(`${name} motor.${state}`), motor[key]);
            assert.ok(difference < 1e-9, `${name}'s motor's ${state} differs from the reference by ${difference}.`);
        }
    }
    const sag = Math.max(...run.times.map((_, index) => run.series('Battery.voltage')[index] - run.series('Shoulder motor.supplyVoltage')[index]));
    assert.ok(sag > 0.01, `The voltage at the motors should sag below the battery's own, by up to ${sag} V.`);
});

test('a pick-and-place cycle on motors runs as the reference runs it: the arm turns to each keyframe, the motors heat and the battery drains', async () => {
    const drawing = exampleDrawing('pickAndPlace');
    const carried = { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm', Wrist: 'Hand', 'Left finger slide': 'Left finger', 'Right finger slide': 'Right finger' };
    const { run, reference } = await matchesReference('pickAndPlace', drawing, carried, 7);
    for (const name of ['Waist', 'Shoulder', 'Elbow', 'Wrist']) {
        const motor = reference.motors[jointNamed(drawing, name).id];
        assert.ok(largestDifference(run.series(`${name} motor.current`), motor.current) < 1e-9, `${name}'s motor's current differs from the reference.`);
        assert.ok(largestDifference(run.series(`${name} motor.windingTemperature`), motor.temperature) < 1e-9, `${name}'s motor's winding differs from the reference.`);
    }
    // At the end of each hold (1 to 1.5 s at the pick, 4.5 to 5 s at the place) the arm is near where the keyframes say,
    // within the controllers' sag under the part's weight.
    const at = (time) => run.times.findIndex((candidate) => Math.abs(candidate - time) < 1e-9);
    const keyframes = drawing.keyframes;
    for (const keyframe of keyframes.filter(({ time }) => [1.5, 5].includes(time))) {
        for (const [name, solid] of [['Waist', 'Turret'], ['Shoulder', 'Upper arm'], ['Elbow', 'Forearm']]) {
            const wanted = keyframe.setpoints[jointNamed(drawing, name).id];
            const angle = run.series(`${solid}.jointAngle`)[at(keyframe.time)];
            assert.ok(Math.abs(angle - wanted) < 0.05, `At ${keyframe.time} s, ${name} should be near ${wanted} rad, not ${angle}.`);
        }
    }
    // The shoulder holds the arm and the part up the whole time: its winding warms and the battery's charge falls.
    assert.ok(run.series('Shoulder motor.windingTemperature').at(-1) > run.series('Shoulder motor.windingTemperature')[0], 'The shoulder\'s winding should warm.');
    assert.ok(run.series('Battery.stateOfCharge').at(-1) < 100, 'The battery should drain.');
});

test('friction, armature, a torque drive and every shape reach the motion as drawn', async () => {
    let drawing = exampleDrawing('twoLinkArm');
    drawing = updateJoint(drawing, jointNamed(drawing, 'Waist').id, { friction: 0.01, armature: 0.002, drive: { mode: 'free' } });
    drawing = updateJoint(drawing, jointNamed(drawing, 'Shoulder').id, { friction: 0.02, armature: 0.001, limits: null, drive: { mode: 'torque', torque: 0.3 } });
    drawing = updateSolid(drawing, solidNamed(drawing, 'Forearm').id, { shape: 'sphere', size: { radius: 0.03 } });
    drawing = updateSolid(drawing, solidNamed(drawing, 'Turret').id, { shape: 'capsule', size: { radius: 0.03, length: 0.03 } });
    await matchesReference('mixed', drawing, { Waist: 'Turret', Shoulder: 'Upper arm', Elbow: 'Forearm' }, 2);
});

test('a joint\'s damping and a solid\'s mass, parameters in the canvas, reach the motion as if drawn', async () => {
    const drawing = exampleDrawing('pendulum');
    const pivot = jointNamed(drawing, 'Pivot');
    const arm = solidNamed(drawing, 'Arm');
    const document = build(drawing, 2);
    // Change the edge's damping and the arm's shared mass in the model, as a slider or a fit would, not the drawing.
    document.edges.find((edge) => edge.name === 'Pivot').parameters.find(({ symbol }) => symbol === 'damping').value = 0.05;
    document.sharedParameters.find(({ name }) => name === 'Arm mass').value = 2 * massOf(arm);
    let drawn = updateJoint(drawing, pivot.id, { damping: 0.05 });
    drawn = updateSolid(drawn, arm.id, { mass: { by: 'mass', value: 2 * massOf(arm) } });
    await matchesReference('changed', drawn, { Pivot: 'Arm' }, 2, document);
});

// A limit and a dry friction are held as MuJoCo holds them: soft rows, gathered into the Ground with every other
// joint's and found there each step from the solids' inertia alone, the damping taken after (lib/modelBuilder.mjs).
// The reference finds them the same way, to rounding; and MuJoCo agrees to about 1e-11 rad as the arm bounces off its
// limit and sticks.
const softBound = 1e-9;
test('a joint stops at its limit and its dry friction holds it, as the reference and MuJoCo have them', async () => {
    let drawing = exampleDrawing('pendulum');
    // Let go level, the arm swings down to positive angles; its limit stops it at 0.4 rad, softly, and its friction
    // holds it there.
    drawing = updateJoint(drawing, jointNamed(drawing, 'Pivot').id, { damping: 0.002, friction: 0.01, limits: { lower: -0.4, upper: 0.4 } });
    const { run, reference } = await matchesReference('limited', drawing, { Pivot: 'Arm' }, 2);
    const angles = run.series('Arm.jointAngle');
    assert.ok(Math.max(...angles) < 0.45, `It passed its limit, to ${Math.max(...angles)} rad.`);
    assert.ok(Math.abs(angles.at(-1) - 0.4) < 0.02, `It should rest against its limit, not at ${angles.at(-1)} rad.`);
    assert.ok(reference);
    if (withoutMujoco) return;
    const pivot = jointNamed(drawing, 'Pivot');
    const mujoco = await runReference(directory, { mjcf: writeMjcf(drawing), joints: [{ name: pivot.id }], times: run.times });
    const difference = largestDifference(angles, mujoco[pivot.id]);
    console.log(`A limit and a dry friction: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)} rad.`);
    assert.ok(difference < softBound, `The pendulum at its limit differs from MuJoCo by ${difference} rad.`);
});

// A loop with joint rows: the loop's rows found with the constraint rows, taken out exactly, as MuJoCo finds its
// equality constraints with its limits and friction.
test('a four-bar with a dry friction on its crank and a limit on its rocker moves as the reference moves it', async () => {
    let drawing = exampleDrawing('fourBar');
    const tree = structure(drawing);
    const hinges = drawing.joints.filter((joint) => joint.kind === 'hinge' && !tree.loops.includes(joint));
    drawing = updateJoint(drawing, hinges[0].id, { friction: 0.02 });
    drawing = updateJoint(drawing, hinges.at(-1).id, { limits: { lower: -0.3, upper: 0.3 } });
    assert.ok(hinges.length >= 2);
    const carried = Object.fromEntries(hinges.map((joint) => [joint.name, drawing.solids.find(({ id }) => id === drawing.frames.find(({ id: frameId }) => frameId === joint.child).solid).name]));
    const { run } = await matchesReference('fourBarRows', drawing, carried, 2);
    assert.ok(run.series(`${carried[hinges.at(-1).name]}.${coordinate(hinges.at(-1))}`).some((angle) => Math.abs(angle) > 0.29), 'The rocker should reach its limit.');
});

// Where the sweeps leave a row's force short of where it settles, polishing finds it exactly; and the loops' forces are
// worked out from the rows' forces as found, not from the sweeps'. So more sweeps change nothing.
test('a four-bar\'s rows found from five sweeps and polishing are as three hundred sweeps find them', () => {
    let drawing = exampleDrawing('fourBar');
    const tree = structure(drawing);
    const hinges = drawing.joints.filter((joint) => joint.kind === 'hinge' && !tree.loops.includes(joint));
    drawing = updateJoint(drawing, hinges[0].id, { friction: 0.02 });
    drawing = updateJoint(drawing, hinges.at(-1).id, { limits: { lower: -0.3, upper: 0.3 } });
    const times = Array.from({ length: 200 }, (_, index) => (index + 1) * 0.01);
    const few = simulate(drawing, times);
    const many = simulate(drawing, times, 0.001, { sweeps: 300 });
    for (const joint of hinges) {
        const difference = largestDifference(few[joint.id], many[joint.id]);
        assert.ok(difference < 1e-12, `${joint.name} differs by ${difference} with more sweeps.`);
    }
});

test('a drawn robot resumed from a checkpoint carries on exactly as the run that never stopped, a closed loop and ball joints too', async () => {
    const cases = [
        ['cartPendulums', ['Cart.jointPosition', 'Front pendulum.jointAngle', 'Back pendulum.yaw']],
        ['fourBar', ['Crank.jointAngle', 'Rocker.closingPinForce1', 'Coupler.yaw']],
        ['ballPendulum', ['Lower link.jointTurnX', 'Upper link.jointSpeed2']]
    ];
    for (const [id, keys] of cases) {
        const drawing = exampleDrawing(id);
        const whole = await runDocument(directory, `${id}Whole`, build(drawing, 2));
        const checkpoint = whole.result.checkpoints.find((candidate) => Math.abs(candidate.time - 1) < 1e-9);
        assert.ok(checkpoint, `${id}: the run kept no checkpoint at 1 s.`);
        const resumed = await runDocument(directory, `${id}Resumed`, build(drawing, 2), { startCheckpoint: checkpoint });
        for (const key of keys) {
            const original = new Map(whole.times.map((time, index) => [time.toFixed(6), whole.series(key)[index]]));
            resumed.times.forEach((time, index) => assert.equal(resumed.series(key)[index], original.get(time.toFixed(6)), `${id}: ${key} at ${time} s.`));
        }
    }
});

// MuJoCo steps a tree as Konjugate does: speeds from the accelerations, then positions from the new speeds, a joint's
// damping taken at the step's end speed, its servos as position actuators of the same stiffness and most torque. So
// they agree to rounding, measured at about 1e-11 over these runs (with MuJoCo 3.16), bounded here at 1e-8.
const mujocoBound = 1e-8;
test('MuJoCo on its own swings the same pendulum, holds the same arm and carries the same cart, to rounding', { skip: withoutMujoco ?? false }, async () => {
    for (const id of ['pendulum', 'twoLinkArm', 'cartPendulums']) {
        const drawing = exampleDrawing(id);
        const run = await runDocument(directory, `mujoco-${id}`, build(drawing, 3));
        const moving = drawing.joints.filter(({ kind }) => kind === 'hinge' || kind === 'slider');
        const controls = Object.fromEntries(moving.filter(({ drive }) => drive?.mode === 'servo').map(({ id: jointId, drive }) => [jointId, drive.setpoint]));
        const reference = await runReference(directory, { mjcf: writeMjcf(drawing), joints: moving.map(({ id: jointId }) => ({ name: jointId })), controls, times: run.times });
        for (const joint of moving) {
            const solidId = drawing.frames.find(({ id: frameId }) => frameId === joint.child).solid;
            const values = run.series(`${drawing.solids.find(({ id: candidate }) => candidate === solidId).name}.${coordinate(joint)}`);
            const difference = largestDifference(values, reference[joint.id]);
            console.log(`${id} ${joint.name}: Konjugate and MuJoCo differ by up to ${difference.toExponential(2)}.`);
            assert.ok(difference < mujocoBound, `${id}: ${joint.name} differs from MuJoCo alone by ${difference}.`);
        }
    }
});

// The same, for what MuJoCo can hold as Konjugate does: ball joints, a body joined to nothing (MuJoCo's free joint) and
// loops closed by a pin or a weld (its connect and weld equalities). Here the two step a little differently: MuJoCo
// turns a ball or a free joint by the exact rotation of its step, where Konjugate steps its quaternion one component
// after another, and MuJoCo holds an equality softly, where Konjugate holds a loop by Baumgarte's terms. Each solid's
// centre is compared every 10 ms: measured within 0.62 mm (the ball pendulum), 0.16 mm (the four-bar), 0.04 mm (the
// weld) and 0.013 mm (the falling body), each about as far as either is from a run at a twentieth of the step.
const mujocoLoopBound = 1e-3;
test('MuJoCo on its own moves ball joints, a body joined to nothing and closed loops as Konjugate does, to within a stated bound', { skip: withoutMujoco ?? false }, async () => {
    const welded = (() => {
        let drawing = createDrawing('Welded arms');
        let result = placed(drawing, 'box', { name: 'Post', size: { x: 0.04, y: 0.1, z: 0.3 }, position: [0, 0, 0.15] });
        drawing = joined(result.drawing, 'ground', result.id, [0, 0, 0], [0, 0, 1], 'fixed', 'Mount').drawing;
        const post = result.id;
        const arms = [];
        for (const [name, side] of [['Front arm', 0.04], ['Back arm', -0.04]]) {
            result = rodBetween(drawing, name, [0, side, 0.28], [0.2, side, 0.2]);
            drawing = joined(result.drawing, post, result.id, [0, side, 0.28], [0, 1, 0], 'hinge', `${name} pivot`).drawing;
            arms.push(result.id);
        }
        result = placed(drawing, 'box', { name: 'Bar', size: { x: 0.02, y: 0.1, z: 0.02 }, position: [0.2, 0, 0.2] });
        drawing = joined(result.drawing, arms[0], result.id, [0.2, 0.04, 0.2], [0, 1, 0], 'fixed', 'Front weld').drawing;
        return joined(drawing, result.id, arms[1], [0.2, -0.04, 0.2], [0, 1, 0], 'fixed', 'Back weld').drawing;
    })();
    const cases = [['ballPendulum', exampleDrawing('ballPendulum'), 1], ['floating', floatingDrawing(), 1], ['fourBar', exampleDrawing('fourBar'), 2], ['welded', welded, 1.5]];
    for (const [id, drawing, runTime] of cases) {
        const run = await runDocument(directory, `mujoco-${id}`, build(drawing, runTime));
        // A torque drive is a force applied along its joint, held throughout.
        const loopIds = new Set(structure(drawing).loops.map(({ id: jointId }) => jointId));
        const torqued = drawing.joints.filter((joint) => !loopIds.has(joint.id) && (joint.kind === 'hinge' || joint.kind === 'slider') && joint.drive?.mode === 'torque');
        const reference = await runReference(directory, {
            mjcf: writeMjcf(drawing), joints: torqued.map(({ id: jointId }) => ({ name: jointId })),
            forces: Object.fromEntries(torqued.map((joint) => [joint.id, joint.drive.torque])), controls: {},
            bodies: drawing.solids.map(({ id: solidId }) => solidId), times: run.times
        });
        let worst = 0;
        for (const solid of drawing.solids) {
            for (const [axis, symbol] of ['x', 'y', 'z'].entries()) worst = Math.max(worst, largestDifference(run.series(`${solid.name}.${symbol}`), reference[solid.id].map((place) => place[axis])));
        }
        console.log(`${id}: Konjugate's and MuJoCo's solids are within ${(worst * 1000).toFixed(3)} mm of each other.`);
        assert.ok(worst < mujocoLoopBound, `${id}: a solid is ${worst} m from where MuJoCo alone has it.`);
    }
});

// Every equation Build writes can be edited by a user in the canvas: its LaTeX reads back, in Konjugate's own parser, as
// an expression giving the same value as the MathJSON the engine runs.
test('every equation a build writes reads back in the canvas as the same expression', async () => {
    const { validateEquationLatex } = await import(pathToFileURL(konjugateModule('src/equationModel.mjs')));
    const evaluate = (expression, values) => {
        if (typeof expression === 'number') return expression;
        if (typeof expression === 'string') return expression === 'True' ? 1 : values.get(expression);
        const [operator, ...operands] = expression;
        if (operator === 'Which') {
            for (let index = 0; index < operands.length; index += 2) if (evaluate(operands[index], values)) return evaluate(operands[index + 1], values);
            return Number.NaN;
        }
        const v = operands.map((operand) => evaluate(operand, values));
        return {
            Add: () => v.reduce((a, b) => a + b, 0), Multiply: () => v.reduce((a, b) => a * b, 1), Negate: () => -v[0], Subtract: () => v[0] - v[1],
            Divide: () => v[0] / v[1], Power: () => v[0] ** v[1], Sqrt: () => Math.sqrt(v[0]), Sin: () => Math.sin(v[0]), Cos: () => Math.cos(v[0]),
            Tanh: () => Math.tanh(v[0]), Arcsin: () => Math.asin(v[0]), Arctan2: () => Math.atan2(v[0], v[1]), Min: () => Math.min(...v), Max: () => Math.max(...v),
            Less: () => Number(v[0] < v[1]), Greater: () => Number(v[0] > v[1]), LessEqual: () => Number(v[0] <= v[1]), GreaterEqual: () => Number(v[0] >= v[1])
        }[operator]();
    };
    let seed = 3;
    const random = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) * 1.6 - 0.8;
    // Every example, and a solid joined to nothing (its free joint's six axes solved through L D Lᵀ): every relationship's
    // equations (several setting values, or one adding a rate) and every node's source terms (a motor's winding, the
    // battery's voltage).
    for (const [id, drawing] of [...examples.map(({ id: exampleId }) => [exampleId, exampleDrawing(exampleId)]), ['floating', floatingDrawing()]]) {
        const { document } = buildModel(drawing);
        const terms = document.nodes.flatMap((node) => node.sourceTerms.map(({ expressionModel }) => ({ name: `${node.name}'s source term`, equationModel: {
            bindings: expressionModel.bindings, definitions: [], outputs: [{ latex: expressionModel.latex, mathJson: expressionModel.mathJson }]
        } })));
        const single = (edge) => (edge.equationModel.outputs ? edge : { ...edge, equationModel: {
            bindings: edge.equationModel.bindings, definitions: [], outputs: [{ latex: edge.equationModel.latex, mathJson: edge.equationModel.mathJson }]
        } });
        for (const edge of [...document.edges.map(single), ...terms]) {
            const { bindings, definitions, outputs } = edge.equationModel;
            const readable = bindings.map(({ symbol }) => ({ symbol }));
            const values = new Map(bindings.map(({ symbol }) => [symbol, random()]));
            for (const item of [...definitions, ...outputs]) {
                const validation = validateEquationLatex(item.latex, readable);
                assert.equal(validation.valid, true, `${id}, ${edge.name}: ${item.latex} ${validation.errors.join(' ')}`);
                const expected = evaluate(item.mathJson, values);
                const actual = evaluate(validation.mathJson, values);
                // A root of a negative random value is NaN either way, which reads back as the same expression.
                const same = (Number.isNaN(expected) && Number.isNaN(actual)) || Math.abs(expected - actual) <= 1e-9 * Math.max(1, Math.abs(expected));
                assert.ok(same, `${id}, ${edge.name}: ${item.latex} gives ${actual}, not ${expected}.`);
                if (item.symbol) { readable.push({ symbol: item.symbol }); values.set(item.symbol, expected); }
            }
        }
    }
});
