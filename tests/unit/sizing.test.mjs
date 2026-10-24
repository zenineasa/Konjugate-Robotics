/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Milestone C without the engine: a motor's band at its joint, a run summed up motor by motor with its verdict, the
// what-ifs as the scenarios take them, and the live parameters a build gives the forks to change.

import assert from 'node:assert/strict';
import test from 'node:test';
import { batteryOf, checkDrawing, defaultMotor, massOf, updateJoint } from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing } from '../../packages/toolbox/lib/examples.mjs';
import { buildModel } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { envelope, motorsOf, summarize, verdictOf, whatIf } from '../../packages/toolbox/lib/sizing.mjs';

const close = (actual, expected, tolerance = 1e-12) => assert.ok(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)), `${actual} is not ${expected}`);

test('a motor\'s band at its joint: its stall torque, its no-load speed, and the torque it can give either way at a speed', () => {
    const motor = { ...defaultMotor('hinge'), resistance: 2, torqueConstant: 0.02, gearRatio: 50 };
    const band = envelope(motor, 12);
    close(band.stallTorque, (50 * 0.02 * 12) / 2);
    close(band.noLoadSpeed, 12 / (0.02 * 50));
    close(band.most(band.noLoadSpeed), 0);
    close(band.most(0), band.stallTorque);
    close(band.least(0), -band.stallTorque);
    // At full duty either way the current is (±V - kt N ω) / R, and the torque N kt times it.
    close(band.most(3), 50 * 0.02 * (12 - 0.02 * 50 * 3) / 2);
    close(band.least(3), 50 * 0.02 * (-12 - 0.02 * 50 * 3) / 2);
});

test('the robot\'s motors are its joints in motor mode, each with the node it is in the model and the solid it turns', () => {
    const drawing = exampleDrawing('pickAndPlace');
    assert.deepEqual(motorsOf(drawing).map(({ name, node, solid }) => [name, node, solid]), [
        ['Waist', 'Waist motor', 'Turret'], ['Shoulder', 'Shoulder motor', 'Upper arm'], ['Elbow', 'Elbow motor', 'Forearm'], ['Wrist', 'Wrist motor', 'Hand']
    ]);
    assert.deepEqual(motorsOf(exampleDrawing('fourBar')), []);
});

// A run of the pick-and-place arm, as the host answers with it: the shoulder at full duty for the middle half of 4 s and
// its winding past its limit; the others resting cool.
function syntheticRun(drawing) {
    const times = [0, 1, 2, 3, 4];
    const series = (values) => times.map((time, index) => [time, values[index]]);
    const answer = { series: { Battery: { stateOfCharge: series([100, 99.5, 99, 98.6, 98.2]), voltage: series([12.6, 12.58, 12.57, 12.55, 12.54]) } } };
    for (const { node, solid, name, motor } of motorsOf(drawing)) {
        const shoulder = name === 'Shoulder';
        answer.series[node] = {
            duty: series(shoulder ? [0.2, 1, 1, 0.3, 0.1] : [0, 0.1, 0, -0.1, 0]),
            torque: series(shoulder ? [0.5, 3, 3.2, 1, 0.4] : [0, 0.1, 0, -0.1, 0]),
            current: series(shoulder ? [0.5, 3, 3.2, 1, 0.4] : [0, 0.05, 0, -0.05, 0]),
            windingTemperature: series(shoulder ? [293.15, 330, 380, 410, 405] : [293.15, 293.2, 293.3, 293.3, 293.3])
        };
        answer.series[solid] = { jointSpeed: series(shoulder ? [0, 0.5, -0.4, 0.2, 0] : [0, 0.1, 0, -0.1, 0]) };
        assert.ok(motor.windingLimit === 403.15);
    }
    return answer;
}

test('a run summed up motor by motor: its operating points, how hard and how hot, its verdict; and the battery', () => {
    const drawing = exampleDrawing('pickAndPlace');
    const { motors, battery, supply } = summarize(syntheticRun(drawing), drawing);
    const shoulder = motors.find(({ name }) => name === 'Shoulder');
    assert.deepEqual(shoulder.points[1], [0.5, 3, 1]);
    close(shoulder.peakTorque, 3.2);
    close(shoulder.peakSpeed, 0.5);
    close(shoulder.peakCurrent, 3.2);
    close(shoulder.hottest, 410);
    // At full duty at 1 s and 2 s, each standing for a second of the 4: half the run.
    close(shoulder.atFullDuty, 0.5);
    close(shoulder.stallTorque, envelope(shoulder.motor, batteryOf(drawing).fullVoltage).stallTorque);
    assert.equal(shoulder.verdict.level, 'over');
    assert.deepEqual(shoulder.verdict.reasons, ['its winding reaches 137 °C, past its 130 °C', 'it is at full duty 50% of the run, giving all it can']);
    assert.ok(motors.filter(({ name }) => name !== 'Shoulder').every(({ verdict }) => verdict.level === 'ok' && !verdict.reasons.length));
    close(battery.chargeUsed, 1.8);
    close(battery.lowestVoltage, 12.54);
    assert.deepEqual(supply, batteryOf(drawing));
});

test('a verdict: warm or now and then at full duty is close; past its limit or at full duty for much of the run is over', () => {
    const motor = defaultMotor('hinge');
    assert.deepEqual(verdictOf({ motor, hottest: 300, atFullDuty: 0 }), { level: 'ok', reasons: [] });
    assert.equal(verdictOf({ motor, hottest: motor.windingLimit - 5, atFullDuty: 0 }).level, 'close');
    assert.equal(verdictOf({ motor, hottest: 300, atFullDuty: 0.05 }).level, 'close');
    assert.equal(verdictOf({ motor, hottest: 300, atFullDuty: 0.3 }).level, 'over');
    assert.equal(verdictOf({ motor, hottest: motor.windingLimit - 5, atFullDuty: 0.3 }).level, 'over', 'over wins over close');
});

test('a what-if is the scenario\'s options, a sentence of what changed and the robot with the change made', () => {
    const drawing = exampleDrawing('pickAndPlace');
    const heavier = whatIf(drawing, { scenarioId: 'heavierSolid', forkAt: 2, runTime: 10, solid: 'Part', mass: 0.6 });
    assert.equal(heavier.change, 'Part weighs 600 g from 2 s');
    assert.deepEqual(heavier.options.supplied, { byParameter: { solidMass: { entities: ['Part'], samples: { Part: [[0, 0.6], [8, 0.6]] } } } });
    assert.equal(heavier.options.forkAt, 2);
    assert.equal(heavier.options.runTime, 10);
    close(massOf(heavier.changed.solids.find(({ name }) => name === 'Part')), 0.6);
    assert.notEqual(massOf(drawing.solids.find(({ name }) => name === 'Part')), 0.6, 'the drawing itself is left alone');
    const weaker = whatIf(drawing, { scenarioId: 'weakerMotor', forkAt: 0, runTime: 10, motor: 'Shoulder', share: 0.5 });
    assert.equal(weaker.change, 'Shoulder\'s motor gives 50% of its torque per ampere');
    const shoulder = drawing.joints.find(({ name }) => name === 'Shoulder');
    close(weaker.options.supplied.byParameter.torqueConstant.samples.Shoulder[0][1], shoulder.drive.motor.torqueConstant / 2);
    close(weaker.changed.joints.find(({ name }) => name === 'Shoulder').drive.motor.torqueConstant, shoulder.drive.motor.torqueConstant / 2);
    const tired = whatIf(drawing, { scenarioId: 'tiredBattery', forkAt: 1, runTime: 10, drop: 2 });
    assert.equal(tired.change, 'the battery gives 2 V less from 1 s');
    const { fullVoltage, emptyVoltage } = batteryOf(drawing);
    assert.deepEqual(tired.options.supplied.byParameter, {
        batteryFullVoltage: { entities: ['Battery'], samples: { Battery: [[0, fullVoltage - 2], [9, fullVoltage - 2]] } },
        batteryEmptyVoltage: { entities: ['Battery'], samples: { Battery: [[0, emptyVoltage - 2], [9, emptyVoltage - 2]] } }
    });
    assert.deepEqual(batteryOf(tired.changed), { ...batteryOf(drawing), fullVoltage: fullVoltage - 2, emptyVoltage: emptyVoltage - 2 });
    // What it refuses, and why.
    assert.throws(() => whatIf(drawing, { scenarioId: 'heavierSolid', forkAt: 10, runTime: 10, solid: 'Part', mass: 1 }), /before the run ends at 10 s/);
    assert.throws(() => whatIf(drawing, { scenarioId: 'heavierSolid', forkAt: 0, runTime: 10, solid: 'Nothing', mass: 1 }), /Choose the solid/);
    assert.throws(() => whatIf(drawing, { scenarioId: 'weakerMotor', forkAt: 0, runTime: 10, motor: 'Left finger slide', share: 0.5 }), /Choose the motor/);
    assert.throws(() => whatIf(drawing, { scenarioId: 'tiredBattery', forkAt: 0, runTime: 10, drop: emptyVoltage }), /less than the battery's empty voltage/);
});

test('a build makes every solid\'s mass, each motor\'s torque constant and the battery\'s voltages live, indexed for the forks', () => {
    const drawing = exampleDrawing('pickAndPlace');
    const { document, parameterIndex } = buildModel(drawing);
    const keys = (key) => parameterIndex.filter((entry) => entry.key === key).map(({ entity }) => entity);
    assert.deepEqual(keys('solidMass').sort(), drawing.solids.map(({ name }) => name).sort());
    assert.deepEqual(keys('torqueConstant'), ['Waist', 'Shoulder', 'Elbow', 'Wrist']);
    assert.deepEqual(keys('batteryFullVoltage'), ['Battery']);
    assert.deepEqual(keys('batteryEmptyVoltage'), ['Battery']);
    for (const entry of parameterIndex) {
        const shared = document.sharedParameters.find(({ id }) => id === entry.sharedParameterId);
        assert.equal(shared.mode, 'live', entry.name);
        assert.equal(entry.live, true);
        assert.ok(entry.minimum < shared.value && shared.value < entry.maximum, `${entry.name}: its slider should hold its value`);
    }
    const part = parameterIndex.find(({ key, entity }) => key === 'solidMass' && entity === 'Part');
    close(part.value, massOf(drawing.solids.find(({ name }) => name === 'Part')));
    assert.ok(part.maximum >= part.value + 5, 'a part can be made 5 kg heavier at least');
});

test('a motor\'s hottest winding: 130 °C by default, kept by drawings from before it, and above the ambient', () => {
    let drawing = exampleDrawing('pickAndPlace');
    const shoulder = drawing.joints.find(({ name }) => name === 'Shoulder');
    assert.equal(defaultMotor('hinge').windingLimit, 403.15);
    // A drawing kept before the field was added builds and checks as before.
    const older = structuredClone(drawing);
    for (const joint of older.joints) if (joint.drive?.motor) delete joint.drive.motor.windingLimit;
    assert.deepEqual(checkDrawing(older).errors, []);
    assert.equal(motorsOf(older).find(({ name }) => name === 'Shoulder').motor.windingLimit, 403.15);
    drawing = updateJoint(drawing, shoulder.id, { drive: { motor: { windingLimit: 280 } } });
    assert.match(checkDrawing(drawing).errors.map(({ message }) => message).join(' '), /Shoulder: its motor's hottest winding must be above the ambient temperature/);
});
