/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A scenario's branches side by side, servo by servo (lib/comparison.mjs).

import assert from 'node:assert/strict';
import test from 'node:test';
import { compareServos, comparisonHeadline, comparisonSignals, forkChange } from '../../packages/toolbox/lib/comparison.mjs';

const servos = [{ name: 'Pitch', key: 'pitch' }, { name: 'Jaw', key: 'jaw' }];
const series = (values) => values.map((value, index) => [index * 0.5, value]);

function answer({ pitch, jaw, pitchWinding, jawWinding, charge }) {
    return {
        series: {
            Arm: { pitchServoTorque: series(pitch), jawServoTorque: series(jaw) },
            'Pitch winding': { windingTemperature: series(pitchWinding) },
            'Jaw winding': { windingTemperature: series(jawWinding) },
            Battery: { stateOfCharge: series(charge) }
        }
    };
}

const baseline = answer({ pitch: [0, 1, 1.5, 1], jaw: [0, 0.5, 0.5, 0.5], pitchWinding: [293.15, 294, 295, 294.5], jawWinding: [293.15, 293.2, 293.3, 293.3], charge: [100, 99.9, 99.8, 99.75] });
const loaded = answer({ pitch: [0, 2, -2, 1], jaw: [0, 0.5, 0.5, 0.5], pitchWinding: [293.15, 295, 297.5, 297], jawWinding: [293.15, 293.2, 293.3, 293.3], charge: [100, 99.85, 99.7, 99.6] });

test('each servo\'s peak torque, time at its limit, hottest winding, and the charge given, branch by branch', () => {
    const comparison = compareServos({ branches: [{ id: 'baseline', ...baseline }, { id: 'scenario', ...loaded }] }, { servos, mechanism: 'Arm', limit: () => 2 });
    const pitch = comparison.servos[0];
    assert.deepEqual(pitch.baseline, { peakTorque: 1.5, secondsAtLimit: 0, peakWinding: 295 - 273.15 });
    // At its limit at 0.5 s and 1 s, each point standing for half a second.
    assert.equal(pitch.scenario.peakTorque, 2);
    assert.equal(pitch.scenario.secondsAtLimit, 1);
    assert.ok(Math.abs(pitch.scenario.peakWinding - (297.5 - 273.15)) < 1e-12);
    assert.ok(Math.abs(comparison.battery.baseline - 0.25) < 1e-12);
    assert.ok(Math.abs(comparison.battery.scenario - 0.4) < 1e-12);
    assert.match(comparisonHeadline(comparison), /^Pitch spends longer at its limit; Pitch by about 1\.00 s\.$/);
});

test('a limit that changes in the fork is the one counted there', () => {
    const limit = (branch, joint, time) => (branch === 'scenario' && joint === 'Jaw' && time >= 0.5 ? 0.5 : 2);
    const comparison = compareServos({ branches: [{ id: 'baseline', ...baseline }, { id: 'scenario', ...baseline }] }, { servos, mechanism: 'Arm', limit });
    assert.equal(comparison.servos[1].baseline.secondsAtLimit, 0);
    assert.equal(comparison.servos[1].scenario.secondsAtLimit, 1.25);
});

test('the same branches twice say nothing changed, and the signals asked for are the ones read', () => {
    const comparison = compareServos({ branches: [{ id: 'baseline', ...baseline }, { id: 'scenario', ...baseline }] }, { servos, mechanism: 'Arm', limit: () => 2 });
    assert.match(comparisonHeadline(comparison), /as they met the baseline/);
    assert.deepEqual(comparisonSignals(servos), ['pitchServoTorque', 'jawServoTorque', 'windingTemperature', 'stateOfCharge']);
});

const built = { mechanism: 'Arm', servos, stallTorque: 1.91, payload: { body: 'Jaw_Body', own: 0.09, mass: 0.3 }, runTime: 9 };

test('a heavier payload is supplied as the gripper\'s own mass and the payload, held from the fork to the end', () => {
    const fork = forkChange(built, { scenarioId: 'heavierPayload', forkAt: '2.5', mass: '0.6' });
    assert.equal(fork.run.forkAt, 2.5);
    assert.equal(fork.run.runTime, 9);
    assert.deepEqual(fork.run.supplied, { byParameter: { payloadMass: { entities: ['Jaw_Body'], samples: { Jaw_Body: [[0, 0.69], [6.5, 0.69]] } } } });
    assert.deepEqual(fork.run.signals, comparisonSignals(servos));
    assert.equal(fork.limit('scenario', 'Pitch', 5), 1.91);
    assert.equal(fork.change, '0.60 kg in the gripper from 2.5 s, against 0.30 kg as built');
    assert.throws(() => forkChange({ ...built, payload: null }, { scenarioId: 'heavierPayload', forkAt: 0, mass: 1 }), /without a payload/);
});

test('a weaker servo is held to its new limit from the fork, and only there is that limit counted', () => {
    const fork = forkChange(built, { scenarioId: 'weakerServo', forkAt: 3, joint: 'Jaw', torqueLimit: '0.8' });
    assert.deepEqual(fork.run.supplied, { byParameter: { servoTorqueLimit: { entities: ['Jaw'], samples: { Jaw: [[0, 0.8], [6, 0.8]] } } } });
    assert.equal(fork.limit('scenario', 'Jaw', 3), 0.8);
    assert.equal(fork.limit('scenario', 'Jaw', 2.9), 1.91, 'Before the fork the servo is as built.');
    assert.equal(fork.limit('baseline', 'Jaw', 5), 1.91, 'The baseline is as built.');
    assert.equal(fork.limit('scenario', 'Pitch', 5), 1.91, 'Only the chosen servo is weaker.');
    // A fork past the end is held to the end, with a path still of some length; a joint with no servo is refused.
    assert.equal(forkChange(built, { scenarioId: 'weakerServo', forkAt: 20, joint: 'Jaw', torqueLimit: 1 }).run.supplied.byParameter.servoTorqueLimit.samples.Jaw[1][0], 0.01);
    assert.throws(() => forkChange(built, { scenarioId: 'weakerServo', forkAt: 0, joint: 'Wrist', torqueLimit: 1 }), /no servo/);
    assert.throws(() => forkChange(built, { scenarioId: 'colderDay', forkAt: 0 }), /no scenario/);
});
