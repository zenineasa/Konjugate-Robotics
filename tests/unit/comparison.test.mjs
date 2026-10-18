/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A scenario's branches side by side, servo by servo (lib/comparison.mjs).

import assert from 'node:assert/strict';
import test from 'node:test';
import { compareServos, comparisonHeadline, comparisonSignals } from '../../packages/toolbox/lib/comparison.mjs';

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
