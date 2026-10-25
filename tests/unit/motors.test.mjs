/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The motor and supply catalog (lib/motors.mjs): each motor's model gives its datasheet's figures, a joint's motor is
// known by them, and servos made motors hold as they held.

import assert from 'node:assert/strict';
import test from 'node:test';
import { batteryOf, checkDrawing, defaultMotor, updateBattery, updateJoint } from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { catalogMotorOf, motorById, motorCatalog, motorDriveForServo, motorFromDatasheet, servosToMotors, supplyCatalog, supplyOf } from '../../packages/toolbox/lib/motors.mjs';
import { envelope } from '../../packages/toolbox/lib/sizing.mjs';

const close = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)), `${actual} is not ${expected}.`);
const jointNamed = (drawing, name) => drawing.joints.find((joint) => joint.name === name);

test('a catalog motor\'s model gives its datasheet\'s stall torque, stall current and no-load speed, and settles at its hottest at its rated torque', () => {
    for (const entry of motorCatalog) {
        const motor = motorFromDatasheet(entry);
        const band = envelope(motor, entry.voltage);
        close(band.stallTorque, entry.stallTorque);
        close(band.noLoadSpeed, entry.noLoadSpeed);
        close(entry.voltage / motor.resistance, entry.stallCurrent);
        assert.ok(motor.gearEfficiency > 0 && motor.gearEfficiency < 1, `${entry.name}'s gears should pass on part of its torque.`);
        const ratedCurrent = entry.ratedTorque / (motor.gearEfficiency * motor.torqueConstant * motor.gearRatio);
        close(motor.ambientTemperature + motor.thermalResistance * motor.resistance * ratedCurrent ** 2, motor.windingLimit);
        // On a lower supply it gives that share of its stall torque and no-load speed.
        close(envelope(motor, 5).stallTorque, (entry.stallTorque * 5) / entry.voltage);
        assert.deepEqual(checkDrawing(updateJoint(exampleDrawing('twoLinkArm'), jointNamed(exampleDrawing('twoLinkArm'), 'Shoulder').id, { drive: { mode: 'motor', motor } })).errors, []);
    }
    // The STS3215 as the SO-ARM100 has it: 1:345, its armature at the joint MuJoCo Menagerie's 0.1 kg·m².
    const sts = motorFromDatasheet(motorById('sts3215-c001'));
    assert.equal(sts.gearRatio, 345);
    close(sts.rotorInertia * sts.gearRatio ** 2, 0.1);
    close(sts.windingLimit, 273.15 + 70);
});

test('a joint\'s motor is known as the catalog motor it is, until a field is changed', () => {
    for (const entry of motorCatalog) assert.equal(catalogMotorOf(motorFromDatasheet(entry))?.id, entry.id);
    assert.equal(catalogMotorOf({ ...motorFromDatasheet(motorCatalog[0]), resistance: 3 }), null);
    assert.equal(catalogMotorOf(defaultMotor('hinge')), null);
    assert.equal(supplyOf(batteryOf({}))?.id, 'lipo3s');
    for (const entry of supplyCatalog) assert.equal(supplyOf(entry)?.id, entry.id);
    assert.equal(supplyOf({ fullVoltage: 7, emptyVoltage: 6, capacity: 1 }), null);
});

test('a servo made a motor holds as stiffly and with the damping it had, near its setpoint', () => {
    const motor = motorFromDatasheet(motorById('sts3215-c001'));
    const voltage = 5;
    const servo = { mode: 'servo', setpoint: 0.3, gain: 50, damping: 2, torqueLimit: 3.5 };
    const { drive, stallTorque } = motorDriveForServo(servo, motor, voltage);
    assert.equal(drive.mode, 'motor');
    assert.equal(drive.setpoint, 0.3);
    close(stallTorque, envelope(motor, voltage).stallTorque);
    // The motor's torque below full duty, at a position and speed near the setpoint, is the servo's.
    const constant = motor.torqueConstant * motor.gearRatio;
    const torqueAt = (position, speed) => {
        const duty = drive.proportional * (drive.setpoint - position) - drive.derivative * speed;
        assert.ok(Math.abs(duty) < 1);
        return (motor.gearEfficiency * constant * (duty * voltage - constant * speed)) / motor.resistance;
    };
    for (const [position, speed] of [[0.29, 0], [0.3, 0.05], [0.305, -0.02]]) close(torqueAt(position, speed), servo.gain * (servo.setpoint - position) - servo.damping * speed);
    // A servo with less damping than the motor's back-EMF gives keeps none of its own.
    assert.equal(motorDriveForServo({ ...servo, damping: 0 }, motor, voltage).drive.derivative, 0);
});

test('every servo on a hinge becomes a catalog motor on a chosen supply, a slider\'s left as it was', () => {
    const drawing = exampleDrawing('gripperArm');
    const done = servosToMotors(drawing, 'sts3215-c001', 'mains5');
    assert.deepEqual(done.turned.map(({ name }) => name), ['Waist', 'Shoulder', 'Elbow', 'Wrist']);
    assert.deepEqual(done.left, ['Left finger slide', 'Right finger slide']);
    assert.equal(done.voltage, 5);
    assert.deepEqual(batteryOf(done.drawing), { fullVoltage: 5, emptyVoltage: 5, capacity: 3600000 });
    for (const name of ['Waist', 'Shoulder', 'Elbow', 'Wrist']) {
        const joint = jointNamed(done.drawing, name);
        assert.equal(joint.drive.mode, 'motor');
        assert.equal(catalogMotorOf(joint.drive.motor)?.id, 'sts3215-c001');
        assert.equal(joint.drive.setpoint, jointNamed(drawing, name).drive.setpoint);
    }
    assert.equal(jointNamed(done.drawing, 'Left finger slide').drive.mode, 'servo');
    assert.deepEqual(checkDrawing(done.drawing).errors, []);
    // Keeping the battery as it is: on its full voltage.
    assert.equal(servosToMotors(drawing, 'sts3215-c018').voltage, batteryOf(drawing).fullVoltage);
    assert.throws(() => servosToMotors(drawing, 'nothing'), /no motor nothing/);
    assert.throws(() => servosToMotors(drawing, 'sts3215-c001', 'nothing'), /no supply nothing/);
});

test('a gear efficiency is more than nothing and at most all, and a supply may hold one voltage', () => {
    const drawing = exampleDrawing('pickAndPlace');
    const shoulder = jointNamed(drawing, 'Shoulder').id;
    const messages = (next) => checkDrawing(next).errors.map(({ message }) => message).join(' ');
    assert.match(messages(updateJoint(drawing, shoulder, { drive: { motor: { gearEfficiency: 1.2 } } })), /gear efficiency is at most 100%/);
    assert.match(messages(updateJoint(drawing, shoulder, { drive: { motor: { gearEfficiency: 0 } } })), /gear efficiency must be more than zero/);
    assert.deepEqual(checkDrawing(updateBattery(drawing, { fullVoltage: 12, emptyVoltage: 12 })).errors, []);
});
