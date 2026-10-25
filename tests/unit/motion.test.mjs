/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Milestone B in the drawing and the build, without the engine: motors and their battery, keyframes, and the jog
// panel's posed robot.

import assert from 'node:assert/strict';
import test from 'node:test';
import {
    addKeyframe, batteryOf, checkDrawing, defaultBattery, drivenJoints, keyframesOf, nextKeyframeTime, removeKeyframe, updateBattery, updateJoint, updateKeyframe
} from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing, examples } from '../../packages/toolbox/lib/examples.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { jogRange, posedDrawing } from '../../packages/toolbox/lib/pose.mjs';
import { place, restCoordinates, treeOf } from '../engine/newtonEuler.mjs';

const jointNamed = (drawing, name) => drawing.joints.find((joint) => joint.name === name);

test('keyframes are kept in time order, added a second after the last, retimed, replaced and removed', () => {
    let drawing = exampleDrawing('twoLinkArm');
    const elbow = jointNamed(drawing, 'Elbow').id;
    assert.equal(nextKeyframeTime(drawing), 0);
    let added = addKeyframe(drawing, { time: 0, setpoints: { [elbow]: 0.1 } });
    drawing = added.drawing;
    const first = added.id;
    assert.equal(nextKeyframeTime(drawing), 1);
    added = addKeyframe(drawing, { time: 1, setpoints: { [elbow]: 0.5 } });
    drawing = added.drawing;
    // Moved before the first, it comes first.
    drawing = updateKeyframe(drawing, added.id, { time: 0.5 });
    drawing = updateKeyframe(drawing, first, { time: 2 });
    assert.deepEqual(keyframesOf(drawing).map(({ time }) => time), [0.5, 2]);
    drawing = updateKeyframe(drawing, first, { setpoints: { [elbow]: -0.2 } });
    assert.equal(keyframesOf(drawing)[1].setpoints[elbow], -0.2);
    assert.deepEqual(keyframesOf(removeKeyframe(drawing, added.id)).map(({ time }) => time), [2]);
});

test('checks name what a keyframe or a motor gets wrong', () => {
    let drawing = exampleDrawing('twoLinkArm');
    const elbow = jointNamed(drawing, 'Elbow').id;
    const mount = jointNamed(drawing, 'Mount').id;
    drawing = addKeyframe(drawing, { time: 1, setpoints: { [elbow]: 0.2 } }).drawing;
    drawing = addKeyframe(drawing, { time: 1, setpoints: { [elbow]: 0.3, [mount]: 0 } }).drawing;
    const { errors, warnings } = checkDrawing(drawing);
    assert.match(errors.map(({ message }) => message).join(' '), /Two keyframes are at 1 s/);
    assert.match(warnings.map(({ message }) => message).join(' '), /sets Mount, which no servo or motor drives now; Build leaves it out/);
    // Motor mode brings its motor and controller with it; each must make sense.
    let motorised = updateJoint(exampleDrawing('twoLinkArm'), elbow, { drive: { mode: 'motor' } });
    assert.ok(jointNamed(motorised, 'Elbow').drive.motor.gearRatio > 0);
    assert.deepEqual(checkDrawing(motorised).errors, []);
    motorised = updateJoint(motorised, elbow, { drive: { motor: { resistance: 0 }, proportional: -1 } });
    const motorErrors = checkDrawing(motorised).errors.map(({ message }) => message).join(' ');
    assert.match(motorErrors, /Elbow: its motor's winding resistance must be more than zero/);
    assert.match(motorErrors, /proportional gain must be zero or more/);
    // The battery: its defaults, and a full voltage that must be at or above its empty one.
    assert.deepEqual(batteryOf(motorised), { ...defaultBattery });
    assert.match(checkDrawing(updateBattery(motorised, { fullVoltage: 5 })).errors.map(({ message }) => message).join(' '), /full voltage must be at or above its empty voltage/);
});

test('the joints keyframes set are those a servo or a motor drives, in the tree', () => {
    const drawing = exampleDrawing('pickAndPlace');
    assert.deepEqual(drivenJoints(drawing).map(({ name }) => name), ['Waist', 'Shoulder', 'Elbow', 'Wrist', 'Left finger slide', 'Right finger slide']);
    assert.deepEqual(drivenJoints(exampleDrawing('fourBar')), []);
    for (const { id } of examples) assert.deepEqual(checkDrawing(exampleDrawing(id)).errors, [], id);
});

test('Build gives each motor its node and relationships, the robot one battery, and each keyframed setpoint its schedule', () => {
    const drawing = exampleDrawing('pickAndPlace');
    const { document } = buildModel(drawing);
    const names = document.nodes.map(({ name }) => name);
    for (const name of ['Battery', 'Waist motor', 'Shoulder motor', 'Elbow motor', 'Wrist motor']) assert.ok(names.includes(name), name);
    const edges = document.edges.map(({ name }) => name);
    for (const part of ['supply', 'controller', 'windings', 'drain']) assert.ok(edges.includes(`Shoulder ${part}`), part);
    // The windings set the motor's torque on the solid the joint carries, where the joint's own relationship reads it.
    const upper = document.nodes.find(({ name }) => name === 'Upper arm');
    const motorTorque = upper.states.find(({ symbol }) => symbol === 'motorTorque');
    const windings = document.edges.find(({ name }) => name === 'Shoulder windings');
    assert.ok(windings.equationModel.outputs.some(({ stateId }) => stateId === motorTorque.id));
    const shoulder = document.edges.find(({ name }) => name === 'Shoulder');
    assert.ok(shoulder.equationModel.bindings.some(({ stateId }) => stateId === motorTorque.id));
    // The rotor's inertia, through the gearbox, is on the joint's armature.
    const motor = jointNamed(drawing, 'Shoulder').drive.motor;
    const armature = shoulder.parameters.find(({ symbol }) => symbol === 'armature');
    assert.ok(Math.abs(armature.value - motor.rotorInertia * motor.gearRatio ** 2) < 1e-12);
    // A keyframed setpoint is a shared parameter with the keyframes as its schedule, linked from the controller (a motor)
    // or the joint's own relationship (a servo).
    const shared = document.sharedParameters.find(({ name }) => name === 'Shoulder setpoint');
    assert.equal(shared.schedule.interpolation, 'linear');
    assert.deepEqual(shared.schedule.samples, keyframesOf(drawing).map(({ time, setpoints }) => [time, setpoints[jointNamed(drawing, 'Shoulder').id]]));
    const controller = document.edges.find(({ name }) => name === 'Shoulder controller');
    assert.equal(controller.parameters.find(({ symbol }) => symbol === 'setpoint').sharedParameterId, shared.id);
    const finger = document.edges.find(({ name }) => name === 'Left finger slide');
    assert.equal(finger.parameters.find(({ symbol }) => symbol === 'setpoint').sharedParameterId, document.sharedParameters.find(({ name }) => name === 'Left finger slide setpoint').id);
});

test('the jog panel poses the robot as the built model would place it, and leaves the drawing alone', () => {
    for (const { id } of examples) {
        const drawing = exampleDrawing(id);
        const bodies = treeOf(drawing);
        const q = restCoordinates(bodies);
        const values = {};
        bodies.forEach((body, index) => {
            if (body.kind === 'hinge') { q[index] = [0.3 + 0.1 * index]; values[body.joint.id] = q[index][0]; }
            if (body.kind === 'slider') { q[index] = [0.012]; values[body.joint.id] = 0.012; }
        });
        const poses = place(bodies, q);
        const posed = posedDrawing(drawing, values);
        bodies.forEach((body, index) => {
            const solid = posed.solids.find(({ id: solidId }) => solidId === body.id);
            assert.ok(solid.position.every((value, axis) => Math.abs(value - poses[index].position[axis]) < 1e-12), `${id}: ${solid.name}`);
        });
        assert.deepEqual(exampleDrawing(id), drawing, 'posing leaves the drawing as it was');
    }
    assert.deepEqual(jogRange({ kind: 'hinge', limits: null }), { lower: -Math.PI, upper: Math.PI });
    assert.deepEqual(jogRange({ kind: 'slider', limits: { lower: -0.02, upper: 0.005 } }), { lower: -0.02, upper: 0.005 });
});
